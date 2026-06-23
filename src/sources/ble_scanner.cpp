#include "ble_scanner.h"
#include "esp_log.h"
#include "esp_timer.h"
#include <cstring>
#include <cstdio>

// ── NimBLE includes (guarded) ─────────────────────────────────────────────────
// The CONFIG_BT_NIMBLE_ENABLED guard is required because these headers only
// exist after sdkconfig.defaults is applied and a clean build is run.
// Without CONFIG_BT_NIMBLE_ENABLED=y in sdkconfig.defaults + clean build,
// all public functions are safe no-ops.
//
// AES-128-CTR: mbedTLS is already in the build (CONFIG_MBEDTLS_CERTIFICATE_BUNDLE).
// Using mbedtls_aes_crypt_ctr() instead of an external library.
// Design note: scottp/victronble was considered but not used — mbedTLS is already
// linked for TLS notifications; adding a separate AES lib would increase binary
// size with no benefit. The decryption algorithm is identical either way.
#ifdef CONFIG_BT_NIMBLE_ENABLED
#include "host/ble_hs.h"
#include "host/ble_gap.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "mbedtls/aes.h"
#endif

static const char* TAG = "ble_scanner";

// ── Victron Instant Readout advertisement format ──────────────────────────────
//
// NimBLE mfg_data[] includes the company ID (unlike bleak/esphome which strip it).
// Two formats are in the wild; the parser handles both:
//
// NEW format ("Product Advertisement", 2022+ firmware, md[2] == 0x10):
//   md[0-1]  company ID = 0x02E1 (Victron Energy, LE: E1 02)
//   md[2]    manufacturer_record_type = 0x10 (PRODUCT_ADVERTISEMENT)
//   md[3]    record_length
//   md[4-5]  product_id (uint16 LE)
//   md[6]    record_type: 0x01=SOLAR_CHARGER, 0x02=BATTERY_MONITOR, …
//   md[7]    data_counter_lsb   (IV / nonce LSB)
//   md[8]    data_counter_msb   (IV / nonce MSB)
//   md[9]    encryption_key_0   (key-check byte; must == advertisement_key[0])
//   md[10+]  AES-128-CTR encrypted payload
//
// OLD format (pre-2022 firmware, md[2] == 0x01/0x02):
//   md[0-1]  company ID = 0x02E1
//   md[2]    record_type: 0x01=MPPT, 0x02=SmartShunt
//   md[3]    IV/nonce byte (single byte)
//   md[4+]   AES-128-CTR encrypted payload
//
// Decryption (AES-128-CTR via mbedTLS):
//   Key:   16 bytes from VictronConnect (stored in NVS)
//   Nonce: new format → { data_counter_lsb, data_counter_msb, 0…0 } (16 bytes, LE counter)
//          old format → { iv_byte, 0…0, 0,0,0,0,0,0,0,1 }
//
// Solar Charger (SOLAR_CHARGER, 0x01) decrypted payload layout (new Product Advertisement):
//   plain[0]     charge_state  (uint8; 0=off, 3=bulk, 4=absorption, 5=float)
//   plain[1]     charger_error (uint8)
//   plain[2-3]   battery_voltage (int16 LE, /100 → V; 0x7FFF = no data)
//   plain[4-5]   battery_charging_current (int16 LE, /10 → A; 0x7FFF = no data)
//   plain[6-7]   yield_today (uint16 LE, ×10 → Wh; 0xFFFF = no data)
//   plain[8-9]   solar_power / pv_power (uint16 LE, W; 0xFFFF = no data)      ← transmitted
//   plain[10-11] pv_voltage / charger_voltage (uint16 LE, /100 → V; 0xFFFF = no data) ← transmitted
//   plain[12-13] pv_current / charger_current (int16 LE, /10 → A; 0x7FFF = no data)   ← transmitted
//
// Reference: keshavdv/victron-ble solar_charger.py (charger_voltage, charger_current);
//            Fabian-Schmidt/esphome-victron_ble extended Solar Charger struct.
// All three of {pv_power, pv_voltage, pv_current} are transmitted directly.
// pv_v and pv_i are present when enc_len >= 12/14; decoded conditionally (not fatal if absent).
//
// SmartShunt (BATTERY_MONITOR, 0x02) decrypted payload layout (old-format):
//   plain[0-1] battery_voltage  (uint16 LE, /100 → V)
//   plain[2-3] battery_current  (int16 LE, /1000 → A)
//   plain[4-5] remaining_Ah     (uint16 LE, /10 → Ah)
//   plain[6-7] SoC              (uint16 LE, /10 → %)
//   NOTE: new-format SmartShunt field layout not yet verified; check
//         keshavdv/victron-ble battery_monitor.py if decrypted values look wrong.
//
// References:
//   keshavdv/victron-ble (Python, authoritative Instant Readout spec)
//   Fabian-Schmidt/esphome-victron_ble (C++ struct VICTRON_BLE_RECORD_BASE)
//   Victron "extra-manufacturer-data-2022-12-14.pdf"

#ifdef CONFIG_BT_NIMBLE_ENABLED

namespace {

// ── Module state ─────────────────────────────────────────────────────────────
static bool s_active = false;
static sources::ShuntSource* s_shunt = nullptr;
static sources::MpptSource*  s_mppt  = nullptr;
// Diagnostic counters: all advertisements received (any device), Victron company
// ID advertisements, and MPPT (type 0x01) advertisements (before MAC check).
static uint32_t s_gap_event_count    = 0;
static uint32_t s_victron_adv_count  = 0;
static uint32_t s_mppt_adv_count     = 0;
// Per-stage counters for the MPPT filter funnel:
//   s_mppt_adv_count     = Victron advs where record_type == 0x01
//   s_mppt_mac_match     = type 0x01 advs where MAC also matches configured target
//   s_mppt_decrypt_ok    = type 0x01 + MAC match + AES decrypt succeeded
static uint32_t s_mppt_mac_match     = 0;
static uint32_t s_mppt_decrypt_ok    = 0;

// ── Per-advertisement debug ring buffer ───────────────────────────────────────
// Written from the NimBLE host task (Core 1) for every advertisement that passes
// the Victron company ID check, before any further filter is applied.
// Read from the httpd task (Core 0) without a mutex — torn reads are acceptable
// for diagnostic data; the head index is volatile to prevent reordering.
struct AdvRingEntry {
  uint8_t  ble_addr[6];    // raw bytes from disc.addr.val (BLE LSB-first order)
  int8_t   rssi;
  uint8_t  record_type;    // md[2]: 0x01=MPPT, 0x02=SmartShunt, other=unknown
  bool     mac_match;      // matched configured MPPT target after byte reversal
  bool     record_type_ok; // record_type == 0x01
  bool     decrypt_ok;     // AES-CTR decrypt succeeded (n/a if !mac_match || !record_type_ok)
  bool     valid;          // slot is populated
};
static constexpr int ADV_RING_SIZE = 8;
static AdvRingEntry s_adv_ring[ADV_RING_SIZE] = {};
static volatile uint8_t s_adv_ring_head = 0;  // next write slot (wraps mod ADV_RING_SIZE)

// Key bytes decoded from Config hex strings at startup.
static uint8_t s_shunt_key[16] = {};
static uint8_t s_mppt_key[16]  = {};
static bool    s_shunt_key_valid = false;
static bool    s_mppt_key_valid  = false;

// Target MAC addresses: 6 bytes each (from config string "AA:BB:CC:DD:EE:FF").
static uint8_t s_shunt_mac[6] = {};
static uint8_t s_mppt_mac[6]  = {};
static bool    s_shunt_mac_valid = false;
static bool    s_mppt_mac_valid  = false;
static bool    s_shunt_enabled   = false;
static bool    s_mppt_enabled    = false;
// Set by pause_scan(); cleared by resume_scan(). Prevents a spurious
// restart in resume_scan() if pause_scan() was never actually called.
static bool    s_scan_paused     = false;

// ── Helpers ───────────────────────────────────────────────────────────────────

// Convert one hex nibble to its integer value. Returns -1 on non-hex char.
static int hex_nibble(char c) {
  if (c >= '0' && c <= '9') return c - '0';
  if (c >= 'a' && c <= 'f') return c - 'a' + 10;
  if (c >= 'A' && c <= 'F') return c - 'A' + 10;
  return -1;
}

// Decode 32-char hex string into 16 key bytes. Returns false on bad input.
static bool parse_hex_key(const char* hex, uint8_t* out) {
  if (!hex || strlen(hex) < 32) return false;
  for (int i = 0; i < 16; i++) {
    int hi = hex_nibble(hex[2 * i]);
    int lo = hex_nibble(hex[2 * i + 1]);
    if (hi < 0 || lo < 0) return false;
    out[i] = (uint8_t)((hi << 4) | lo);
  }
  return true;
}

// MAC parsing delegates to storage::mac_normalize which accepts colon, hyphen,
// or bare 12-hex-char input and is the single source of truth for this logic.

// AES-128-CTR decrypt using mbedTLS.
// nonce_counter_init: caller-built 16-byte initial counter block.
//   New format: { data_counter_lsb, data_counter_msb, 0…0 }  (LE 16-bit counter)
//   Old format: { iv_byte, 0…0, 0,0,0,0,0,0,0,1 }
static bool aes_ctr_decrypt(const uint8_t* key, const uint8_t nonce_counter_init[16],
                             const uint8_t* in, uint8_t* out, size_t len) {
  mbedtls_aes_context ctx;
  mbedtls_aes_init(&ctx);

  if (mbedtls_aes_setkey_enc(&ctx, key, 128) != 0) {
    mbedtls_aes_free(&ctx);
    return false;
  }

  // mbedTLS mutates nonce_counter during CTR streaming; copy to keep caller's buffer clean.
  uint8_t nonce_counter[16];
  memcpy(nonce_counter, nonce_counter_init, 16);

  uint8_t stream_block[16] = {};
  size_t  nc_off = 0;

  int rc = mbedtls_aes_crypt_ctr(&ctx, len, &nc_off, nonce_counter,
                                  stream_block, in, out);
  mbedtls_aes_free(&ctx);
  return (rc == 0);
}

// ── Decode MPPT / Solar Charger (record_type 0x01) ───────────────────────────
// new_fmt=true  → Product Advertisement layout (md[7-8]=IV, md[9]=key_check, md[10+]=enc)
// new_fmt=false → Old single-byte-IV layout (md[3]=IV, md[4+]=enc)
// Returns true when AES decrypt succeeded and values were pushed to MpptSource.
static bool decode_mppt(const uint8_t* md, size_t len, bool new_fmt, uint32_t now_ms) {
  if (!s_mppt || !s_mppt_key_valid) return false;

  const uint8_t* enc;
  size_t enc_len;
  uint8_t nonce[16] = {};

  if (new_fmt) {
    // Minimum: company(2)+type(1)+len(1)+product_id(2)+rec_type(1)+iv(2)+key_check(1)+payload(1) = 11
    if (len < 11) return false;
    // Key-check byte: plaintext byte that must equal advertisement_key[0].
    // Mismatch means the wrong instant-readout key is configured.
    if (md[9] != s_mppt_key[0]) {
      ESP_LOGW(TAG, "MPPT key_check mismatch: pkt=0x%02X key[0]=0x%02X — wrong instant-readout key?",
               md[9], s_mppt_key[0]);
      return false;
    }
    // Nonce: 16-bit little-endian counter { lsb, msb, 0…0 }
    nonce[0] = md[7];  // data_counter_lsb
    nonce[1] = md[8];  // data_counter_msb
    enc     = md + 10;
    enc_len = len - 10;
    // Need at least plain[0..9] for pv_power at bytes 8-9.
    // pv_voltage at bytes 10-11 and pv_current at bytes 12-13 are read
    // conditionally if enc_len allows (not fatal if absent — older firmware).
    if (enc_len < 10) return false;
  } else {
    if (len < 5) return false;
    nonce[0]  = md[3];  // single IV byte
    nonce[15] = 1;       // old-format big-endian counter starting at 1
    enc     = md + 4;
    enc_len = len - 4;
    if (enc_len < 7) return false;
  }

  uint8_t plain[32] = {};
  size_t  dec_len = enc_len < sizeof(plain) ? enc_len : sizeof(plain);
  if (!aes_ctr_decrypt(s_mppt_key, nonce, enc, plain, dec_len)) return false;

  // ── Decode fields and check Victron not-available sentinels ─────────────────
  // Sentinels per keshavdv/victron-ble + esphome-victron_ble:
  //   signed int16 fields  → 0x7FFF = not available
  //   unsigned uint16 fields → 0xFFFF = not available
  // Check raw value BEFORE scaling to avoid phantom readings (e.g. 3276.7 A at night).

  uint8_t charge_state  = 0;
  float pv_power_w      = 0.0f;
  float pv_voltage_v    = 0.0f;
  float pv_current_a    = 0.0f;
  float batt_volt_v     = 0.0f;
  float batt_curr_a     = 0.0f;
  float yield_today_wh  = 0.0f;
  bool  pv_power_valid  = false;
  bool  pv_v_valid      = false;
  bool  pv_i_valid      = false;
  bool  batt_v_valid    = false;
  bool  batt_i_valid    = false;
  bool  yield_valid     = false;

  if (new_fmt) {
    charge_state = plain[0];

    int16_t  batt_v_raw   = (int16_t) ((uint16_t)plain[2] | ((uint16_t)plain[3] << 8));
    int16_t  batt_i_raw   = (int16_t) ((uint16_t)plain[4] | ((uint16_t)plain[5] << 8));
    uint16_t yield_raw    = (uint16_t)((uint16_t)plain[6] | ((uint16_t)plain[7] << 8));
    uint16_t solar_p_raw  = (uint16_t)((uint16_t)plain[8] | ((uint16_t)plain[9] << 8));

    batt_v_valid   = (batt_v_raw  != (int16_t)0x7FFF);
    batt_i_valid   = (batt_i_raw  != (int16_t)0x7FFF);
    yield_valid    = (yield_raw   != 0xFFFFu);
    pv_power_valid = (solar_p_raw != 0xFFFFu);

    if (batt_v_valid)   batt_volt_v   = (float)batt_v_raw  / 100.0f;
    if (batt_i_valid)   batt_curr_a   = (float)batt_i_raw  / 10.0f;
    if (yield_valid)    yield_today_wh = (float)yield_raw   * 10.0f;
    if (pv_power_valid) pv_power_w    = (float)solar_p_raw;

    // pv_voltage (charger_voltage) and pv_current (charger_current) are real measured
    // fields from the MPPT input-side sensors, present at bytes 10-13 when enc_len >= 12/14.
    // Reference: keshavdv/victron-ble solar_charger.py (charger_voltage, charger_current).
    if (dec_len >= 12) {
      uint16_t pv_v_raw = (uint16_t)((uint16_t)plain[10] | ((uint16_t)plain[11] << 8));
      pv_v_valid = (pv_v_raw != 0xFFFFu);
      if (pv_v_valid) pv_voltage_v = (float)pv_v_raw / 100.0f;
    }
    if (dec_len >= 14) {
      int16_t pv_i_raw = (int16_t)((uint16_t)plain[12] | ((uint16_t)plain[13] << 8));
      pv_i_valid = (pv_i_raw != (int16_t)0x7FFF);
      if (pv_i_valid) pv_current_a = (float)pv_i_raw / 10.0f;
    }
  } else {
    // Old format:
    //   plain[0]   device_state
    //   plain[1-2] PV power (uint16 LE, ×1 W; sentinel 0xFFFF)
    //   plain[3-4] battery_voltage (uint16 LE, /100 V; sentinel 0xFFFF)
    //   plain[5-6] battery_current (int16 LE, /10 A; sentinel 0x7FFF)
    // Old format does not carry pv_voltage, pv_current, or yield_today.
    charge_state = plain[0];

    uint16_t pv_p_raw   = (uint16_t)((uint16_t)plain[1] | ((uint16_t)plain[2] << 8));
    uint16_t batt_v_raw = (uint16_t)((uint16_t)plain[3] | ((uint16_t)plain[4] << 8));
    int16_t  batt_i_raw = (int16_t) ((uint16_t)plain[5] | ((uint16_t)plain[6] << 8));

    pv_power_valid = (pv_p_raw   != 0xFFFFu);
    batt_v_valid   = (batt_v_raw != 0xFFFFu);
    batt_i_valid   = (batt_i_raw != (int16_t)0x7FFF);

    if (pv_power_valid) pv_power_w  = (float)pv_p_raw;
    if (batt_v_valid)   batt_volt_v = (float)batt_v_raw / 100.0f;
    if (batt_i_valid)   batt_curr_a = (float)batt_i_raw / 10.0f;
    // pv_v_valid / pv_i_valid remain false — old format does not carry these fields.
  }

  s_mppt->set_decoded_values(charge_state,
                              pv_power_w, pv_voltage_v, pv_current_a,
                              batt_volt_v, batt_curr_a, yield_today_wh,
                              pv_power_valid, pv_v_valid, pv_i_valid,
                              batt_v_valid, batt_i_valid, yield_valid,
                              now_ms);

  ESP_LOGD(TAG, "MPPT(%s): cs=%u pv=%.0fW %.2fV %.2fA  batt=%.2fV %.2fA  yield=%.0fWh"
                "  (pp=%d pv_v=%d pv_i=%d)",
           new_fmt ? "new" : "old", (unsigned)charge_state,
           pv_power_w, pv_voltage_v, pv_current_a,
           batt_volt_v, batt_curr_a, yield_today_wh,
           (int)pv_power_valid, (int)pv_v_valid, (int)pv_i_valid);
  return true;
}

// ── Decode SmartShunt / Battery Monitor (record_type 0x02) ───────────────────
// new_fmt flag selects correct IV/payload offsets (same as decode_mppt).
// Field layout below matches the old-format spec; new-format field layout for
// BATTERY_MONITOR has not been hardware-verified — check keshavdv/victron-ble
// battery_monitor.py if values look wrong with a new-firmware SmartShunt.
static void decode_shunt(const uint8_t* md, size_t len, bool new_fmt, uint32_t now_ms) {
  if (!s_shunt || !s_shunt_key_valid) return;

  const uint8_t* enc;
  size_t enc_len;
  uint8_t nonce[16] = {};

  if (new_fmt) {
    if (len < 11) return;
    nonce[0] = md[7];
    nonce[1] = md[8];
    enc     = md + 10;
    enc_len = len - 10;
    if (enc_len < 8) return;
  } else {
    if (len < 4) return;
    nonce[0]  = md[3];
    nonce[15] = 1;
    enc     = md + 4;
    enc_len = len - 4;
    if (enc_len < 8) return;
  }

  uint8_t plain[32] = {};
  size_t  dec_len = enc_len < sizeof(plain) ? enc_len : sizeof(plain);
  if (!aes_ctr_decrypt(s_shunt_key, nonce, enc, plain, dec_len)) return;

  uint16_t batt_v_raw  = (uint16_t)((uint16_t)plain[0] | ((uint16_t)plain[1] << 8));
  int16_t  curr_raw    = (int16_t) ((uint16_t)plain[2] | ((uint16_t)plain[3] << 8));
  // plain[4-5]: remaining_Ah * 10 (not used)
  uint16_t soc_raw     = (uint16_t)((uint16_t)plain[6] | ((uint16_t)plain[7] << 8));

  float voltage_v = (float)batt_v_raw / 100.0f;   // V
  float current_a = (float)curr_raw   / 1000.0f;  // A (mA resolution)
  float soc_pct   = (float)soc_raw    / 10.0f;    // %

  s_shunt->set_decoded_values(current_a, voltage_v, soc_pct, now_ms);

  ESP_LOGD(TAG, "Shunt(%s): %.3f A  %.2f V  %.1f%%",
           new_fmt ? "new" : "old", current_a, voltage_v, soc_pct);
}

// ── MAC address comparison ────────────────────────────────────────────────────
// NimBLE address bytes are in reversed order compared to the "AA:BB:CC:DD:EE:FF"
// string representation (BLE is LSB-first).
static bool mac_matches(const uint8_t* ble_addr, const uint8_t* target_mac) {
  for (int i = 0; i < 6; i++) {
    if (ble_addr[i] != target_mac[5 - i]) return false;
  }
  return true;
}

// ── GAP scan event callback ───────────────────────────────────────────────────
static int gap_event_cb(struct ble_gap_event* event, void* arg) {
  (void)arg;
  if (event->type != BLE_GAP_EVENT_DISC) return 0;
  s_gap_event_count++;  // count every advertisement, not just Victron

  const struct ble_gap_disc_desc& disc = event->disc;
  uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000LL);

  // Parse advertisement fields looking for Manufacturer Specific Data (type 0xFF).
  struct ble_hs_adv_fields fields = {};
  int rc = ble_hs_adv_parse_fields(&fields, disc.data, disc.length_data);
  if (rc != 0 || fields.mfg_data_len < 5) return 0;

  const uint8_t* md = fields.mfg_data;

  // Check Victron Energy company ID (0x02E1, little-endian: E1 02).
  if (md[0] != 0xE1 || md[1] != 0x02) return 0;

  s_victron_adv_count++;

  // Detect advertisement format from md[2]:
  //   0x10 = PRODUCT_ADVERTISEMENT (new format, 2022+ firmware)
  //          → actual record_type is at md[6], IV at md[7-8], key-check at md[9]
  //   0x01/0x02/… = old format, md[2] IS the record_type directly
  static constexpr uint8_t PRODUCT_ADVERTISEMENT = 0x10;
  bool    new_fmt     = (md[2] == PRODUCT_ADVERTISEMENT);
  uint8_t record_type;

  if (new_fmt) {
    if (fields.mfg_data_len < 7) return 0;  // need md[6] for record_type
    record_type = md[6];
  } else {
    record_type = md[2];
  }

  // ── Populate per-advertisement debug ring buffer ──────────────────────────
  // record_type here is the semantic device type (0x01=SOLAR_CHARGER, etc.)
  // regardless of advertisement format, so the diagnostic UI sees a consistent value.
  {
    uint8_t slot = s_adv_ring_head % ADV_RING_SIZE;
    AdvRingEntry& e = s_adv_ring[slot];
    memcpy(e.ble_addr, disc.addr.val, 6);
    e.rssi           = disc.rssi;
    e.record_type    = record_type;
    e.mac_match      = false;
    e.record_type_ok = (record_type == 0x01);
    e.decrypt_ok     = false;
    e.valid          = true;
    s_adv_ring_head  = (uint8_t)((slot + 1) % ADV_RING_SIZE);
  }
  uint8_t written_slot = (uint8_t)((s_adv_ring_head + ADV_RING_SIZE - 1) % ADV_RING_SIZE);
  AdvRingEntry& cur = s_adv_ring[written_slot];

  if (record_type == 0x01) {
    s_mppt_adv_count++;
    if (s_mppt_enabled && s_mppt_mac_valid) {
      bool matched = mac_matches(disc.addr.val, s_mppt_mac);
      cur.mac_match = matched;
      if (matched) {
        s_mppt_mac_match++;
        bool ok = decode_mppt(md, fields.mfg_data_len, new_fmt, now_ms);
        cur.decrypt_ok = ok;
        if (ok) s_mppt_decrypt_ok++;
      } else {
        if (s_mppt_adv_count % 10 == 1) {
          ESP_LOGI(TAG, "MPPT adv (rec=0x01 fmt=%s) MAC %02X:%02X:%02X:%02X:%02X:%02X "
                        "does not match configured target (victron_adv=%lu mppt_adv=%lu)",
                   new_fmt ? "new" : "old",
                   disc.addr.val[5], disc.addr.val[4], disc.addr.val[3],
                   disc.addr.val[2], disc.addr.val[1], disc.addr.val[0],
                   (unsigned long)s_victron_adv_count,
                   (unsigned long)s_mppt_adv_count);
        }
      }
    } else if (s_mppt_enabled) {
      ESP_LOGW(TAG, "MPPT adv seen but MAC/key not valid — check config");
    }
  } else if (record_type == 0x02) {
    if (s_shunt_enabled && s_shunt_mac_valid) {
      if (mac_matches(disc.addr.val, s_shunt_mac)) {
        decode_shunt(md, fields.mfg_data_len, new_fmt, now_ms);
      }
    }
  } else {
    // Unknown device record type within a Victron advertisement.
    // Known types: 0x01=SOLAR_CHARGER, 0x02=BATTERY_MONITOR, 0x03=Lynx,
    //              0x04=Multi/Quattro, 0x06=Inverter, 0x0B=SmartSolar-extended
    static uint8_t s_last_unknown_type = 0xFF;
    if (record_type != s_last_unknown_type) {
      s_last_unknown_type = record_type;
      ESP_LOGI(TAG, "Unknown Victron record_type 0x%02X (fmt=%s) from %02X:%02X:%02X:%02X:%02X:%02X",
               record_type, new_fmt ? "new" : "old",
               disc.addr.val[5], disc.addr.val[4], disc.addr.val[3],
               disc.addr.val[2], disc.addr.val[1], disc.addr.val[0]);
    }
  }

  return 0;
}

// ── Scan start helper ─────────────────────────────────────────────────────────
static void start_scan() {
  struct ble_gap_disc_params disc_params = {};
  disc_params.passive = 1;  // no SCAN_REQ (observer-only)

  // filter_duplicates=0: report every advertisement from every device.
  // filter_duplicates=1 was added in dev.5 to prevent Core 0 CPU starvation
  // from dense BLE environments. That starvation is now solved structurally:
  //   - NimBLE host task is on Core 1 (not Core 0 with WiFi/httpd)
  //   - Scan window is 50 ms / 2000 ms = 2.5% duty cycle
  // Keeping filter_duplicates=1 causes Victron data to only refresh when a new
  // scan session starts (i.e. after each TLS pause/resume). With no alerts
  // firing, MPPT/shunt values would never update. filter_duplicates=0 restores
  // continuous refresh at ~1 Hz per device (Victron's advertisement rate).
  disc_params.filter_duplicates = 0;

  // Fix 3 (dev.5): reduce scan window 200 ms → 50 ms (duty 10% → 2.5%).
  // Victron beacons repeat multiple times per 2 s interval (typically every
  // 250-500 ms), so every scan period captures a fresh reading at 50 ms window.
  // Benefit: RF starvation windows shrink 4x; NimBLE host CPU load per scan
  // cycle also reduces (fewer advertisement report events per window).
  disc_params.itvl   = 3200;  // 2000 ms / 0.625 ms (unchanged)
  disc_params.window =   80;  //   50 ms / 0.625 ms  (was 320 / 200 ms)

  // Log effective parameters at runtime. Verify these in serial log to confirm
  // the clean build applied the changes correctly.
  uint32_t itvl_ms   = disc_params.itvl   * 625 / 1000;
  uint32_t window_ms = disc_params.window * 625 / 1000;
  ESP_LOGI(TAG, "scan params: interval=%lu ms  window=%lu ms  duty=%.1f%%  filter_dup=%d",
           (unsigned long)itvl_ms, (unsigned long)window_ms,
           100.0f * (float)window_ms / (float)itvl_ms,
           disc_params.filter_duplicates);

  int rc = ble_gap_disc(BLE_OWN_ADDR_PUBLIC, BLE_HS_FOREVER, &disc_params,
                        gap_event_cb, nullptr);
  if (rc != 0) {
    ESP_LOGW(TAG, "ble_gap_disc failed: %d — will retry on next sync", rc);
  }
}

// ── NimBLE host task ──────────────────────────────────────────────────────────
static void nimble_host_task(void* param) {
  (void)param;
  // Read the actual hardware core at task entry. This is the ground-truth check
  // that CONFIG_BT_NIMBLE_PINNED_TO_CORE=1 in sdkconfig.defaults actually took
  // effect after a clean build. If this logs "Core 0" the build was not clean —
  // rm sdkconfig.esp32s3 && pio run is required.
  int actual_core = (int)xPortGetCoreID();
  if (actual_core != 1) {
    ESP_LOGW(TAG, "NimBLE host task on Core %d — expected Core 1. "
                  "Did you rm sdkconfig.esp32s3 && pio run?", actual_core);
  } else {
    ESP_LOGI(TAG, "NimBLE host task on Core %d (correct — coexistence fix applied)",
             actual_core);
  }
  nimble_port_run();         // blocks until nimble_port_stop() is called
  nimble_port_freertos_deinit();
}

// ── NimBLE on-sync callback (called when host and controller are synced) ─────
static void on_sync() {
  ESP_LOGI(TAG, "NimBLE synced — starting passive scan");
  // Observer-only: using BLE_OWN_ADDR_PUBLIC so no random address needed.
  start_scan();
  s_active = true;
}

// ── NimBLE on-reset callback ──────────────────────────────────────────────────
static void on_reset(int reason) {
  s_active = false;
  ESP_LOGW(TAG, "NimBLE reset (reason %d) — scan halted", reason);
}

}  // anonymous namespace

#endif  // CONFIG_BT_NIMBLE_ENABLED

// ── Public API ────────────────────────────────────────────────────────────────

namespace sources::ble_scanner {

bool start(const Config& cfg, ShuntSource* shunt, MpptSource* mppt) {
#ifndef CONFIG_BT_NIMBLE_ENABLED
  (void)cfg; (void)shunt; (void)mppt;
  ESP_LOGW(TAG, "BLE requested but CONFIG_BT_NIMBLE_ENABLED is not set. "
               "Run: rm sdkconfig.esp32s3 && pio run -t clean && pio run");
  return false;
#else
  s_shunt         = shunt;
  s_mppt          = mppt;
  s_shunt_enabled = cfg.ble_shunt_enabled;
  s_mppt_enabled  = cfg.ble_mppt_enabled;

  // Parse MAC addresses and keys. Log parse errors but never log key values.
  // storage::mac_normalize accepts any of: "AA:BB:CC:DD:EE:FF", "AA-BB-CC-DD-EE-FF",
  // "aabbccddeeff" (bare 12 hex chars) — forgiving on load in case an old
  // non-canonical NVS value is present before the config save path normalized it.
  if (s_shunt_enabled) {
    char canonical[18];
    s_shunt_mac_valid = (cfg.ble_shunt_mac[0] != '\0') &&
                        storage::mac_normalize(cfg.ble_shunt_mac, canonical, s_shunt_mac, nullptr, 0) &&
                        (canonical[0] != '\0');
    s_shunt_key_valid = parse_hex_key(cfg.ble_shunt_key, s_shunt_key);
    if (!s_shunt_mac_valid)
      ESP_LOGW(TAG, "shunt: MAC not configured or invalid — shunt decode disabled");
    if (!s_shunt_key_valid)
      ESP_LOGW(TAG, "shunt: invalid key (check 32-char hex) — shunt decode disabled");
  }
  if (s_mppt_enabled) {
    char canonical[18];
    s_mppt_mac_valid = (cfg.ble_mppt_mac[0] != '\0') &&
                       storage::mac_normalize(cfg.ble_mppt_mac, canonical, s_mppt_mac, nullptr, 0) &&
                       (canonical[0] != '\0');
    s_mppt_key_valid = parse_hex_key(cfg.ble_mppt_key, s_mppt_key);
    if (!s_mppt_mac_valid)
      ESP_LOGW(TAG, "mppt: MAC not configured or invalid — mppt decode disabled");
    if (!s_mppt_key_valid)
      ESP_LOGW(TAG, "mppt: invalid key (check 32-char hex) — mppt decode disabled");
  }

  // Initialise NimBLE host. esp_bt_controller_init() is called internally by
  // nimble_port_init(); the ~80 KB heap allocation happens here.
  esp_err_t err = nimble_port_init();
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "nimble_port_init failed: %s — BLE disabled", esp_err_to_name(err));
    return false;
  }

  ble_hs_cfg.sync_cb  = on_sync;
  ble_hs_cfg.reset_cb = on_reset;

  // NimBLE host task on Core 1, low priority (below ControlTask on Core 0).
  // Stack 4 KB is sufficient for observer-only operation.
  nimble_port_freertos_init(nimble_host_task);

  ESP_LOGI(TAG, "NimBLE started — shunt=%s mppt=%s",
           s_shunt_enabled ? "enabled" : "off",
           s_mppt_enabled  ? "enabled" : "off");
  return true;
#endif
}

bool is_active() {
#ifdef CONFIG_BT_NIMBLE_ENABLED
  return s_active;
#else
  return false;
#endif
}

const char* stack_name() {
#ifdef CONFIG_BT_NIMBLE_ENABLED
  return "nimble";
#else
  return "none";
#endif
}

void pause_scan() {
#ifdef CONFIG_BT_NIMBLE_ENABLED
  if (!s_active) return;
  int rc = ble_gap_disc_cancel();
  if (rc != 0) {
    // rc != 0 is harmless: either the scan had already stopped (BLE_HS_EALREADY)
    // or NimBLE is mid-reset. Either way, the handshake proceeds safely.
    ESP_LOGD(TAG, "pause_scan: disc_cancel rc=%d (harmless)", rc);
  }
  s_scan_paused = true;
  ESP_LOGD(TAG, "BLE scan paused for TLS handshake");
#endif
}

void resume_scan() {
#ifdef CONFIG_BT_NIMBLE_ENABLED
  if (!s_scan_paused) return;
  s_scan_paused = false;
  if (!s_active) {
    // NimBLE reset while TLS was running. on_sync() will restart the scan
    // automatically when the stack re-syncs. No action needed here.
    ESP_LOGD(TAG, "resume_scan: NimBLE reset during TLS pause — scan resumes on re-sync");
    return;
  }
  start_scan();
  ESP_LOGD(TAG, "BLE scan resumed after TLS handshake");
#endif
}

uint32_t gap_event_count() {
#ifdef CONFIG_BT_NIMBLE_ENABLED
  return s_gap_event_count;
#else
  return 0;
#endif
}

uint32_t victron_adv_count() {
#ifdef CONFIG_BT_NIMBLE_ENABLED
  return s_victron_adv_count;
#else
  return 0;
#endif
}

uint32_t mppt_adv_count() {
#ifdef CONFIG_BT_NIMBLE_ENABLED
  return s_mppt_adv_count;
#else
  return 0;
#endif
}

void get_adv_debug(AdvDebugState& out) {
  memset(&out, 0, sizeof(out));
#ifdef CONFIG_BT_NIMBLE_ENABLED
  out.victron_total    = s_victron_adv_count;
  out.mppt_type_match  = s_mppt_adv_count;
  out.mppt_mac_match   = s_mppt_mac_match;
  out.mppt_decrypt_ok  = s_mppt_decrypt_ok;
  out.mppt_mac_valid   = s_mppt_mac_valid;

  // Render the configured MPPT MAC exactly as the internal comparison bytes
  // (network order, MSB first — same representation as the config string).
  if (s_mppt_mac_valid) {
    snprintf(out.configured_mac, sizeof(out.configured_mac),
             "%02x:%02x:%02x:%02x:%02x:%02x",
             s_mppt_mac[0], s_mppt_mac[1], s_mppt_mac[2],
             s_mppt_mac[3], s_mppt_mac[4], s_mppt_mac[5]);
  }

  // Snapshot the ring into out.entries[].  The head may advance during the
  // copy (single-writer, we are the reader), but each individual entry write
  // is at most a few bytes — torn reads give stale but not corrupted data.
  // Walk the ring oldest-first so entries[0] is the oldest.
  uint8_t head = s_adv_ring_head;
  out.count = 0;
  for (int i = 0; i < ADV_RING_SIZE; i++) {
    uint8_t slot = (uint8_t)((head + i) % ADV_RING_SIZE);
    const AdvRingEntry& src = s_adv_ring[slot];
    if (!src.valid) continue;
    AdvDebugEntry& dst = out.entries[out.count++];
    // Convert BLE LSB-first address to canonical "AA:BB:CC:DD:EE:FF" string.
    snprintf(dst.mac_str, sizeof(dst.mac_str),
             "%02x:%02x:%02x:%02x:%02x:%02x",
             src.ble_addr[5], src.ble_addr[4], src.ble_addr[3],
             src.ble_addr[2], src.ble_addr[1], src.ble_addr[0]);
    dst.rssi           = src.rssi;
    dst.record_type    = src.record_type;
    dst.mac_match      = src.mac_match;
    dst.record_type_ok = src.record_type_ok;
    dst.decrypt_ok     = src.decrypt_ok;
    dst.valid          = true;
  }
#endif
}

}  // namespace sources::ble_scanner
