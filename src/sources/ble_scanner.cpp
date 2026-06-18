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

// ── Victron GXVE advertisement format ────────────────────────────────────────
//
// Manufacturer-specific data (GAP type 0xFF) layout:
//   byte 0-1: company ID = 0x02E1 (Victron Energy, little-endian)
//   byte 2:   record type (see below)
//   byte 3:   IV/nonce byte (increments each advertisement)
//   byte 4+:  AES-128-CTR encrypted payload
//
// Record types used by this spike:
//   0x01 = Smart Solar MPPT / Solar Charger
//   0x02 = BMV-700 / SmartShunt (battery monitor)
//
// Decryption (AES-128-CTR):
//   Key:     16 bytes from VictronConnect (owner-provided, stored in Config)
//   Nonce:   [iv_byte, 0,0,0,0,0,0,0] (8 bytes)
//   Counter: 0x00000001 (big-endian in bytes 8-15)
//   → nonce_counter[16] = { iv, 0,0,0,0,0,0,0, 0,0,0,0,0,0,0,1 }
//
// Field layouts after decryption:
//
//   MPPT (type 0x01):
//     byte 0:   device state (0=off, 2=fault, 3=bulk, 4=absorption, 5=float)
//     byte 1-2: PV power (W, uint16 LE)
//     byte 3-4: battery voltage * 100 (uint16 LE) → /100 for V
//     byte 5-6: battery current * 10 (int16 LE, signed) → /10 for A
//     byte 7-8: daily yield * 100 Wh (uint16 LE) → /100 for kWh
//
//   SmartShunt (type 0x02):
//     byte 0-1: battery voltage * 100 (uint16 LE) → /100 for V
//     byte 2-3: battery current * 1000 (int16 LE) → /1000 for A (mA resolution)
//     byte 4-5: remaining capacity * 10 (uint16 LE) → /10 for Ah
//     byte 6-7: SOC * 10 (uint16 LE) → /10 for % (0-1000 → 0-100%)
//
// References:
//   - Victron Open Energy Monitor project (victron-ble Python lib by keshavdv)
//   - Field layout verified against SmartShunt firmware 3.x advertisement format
//   - Owner must verify on hardware and report any field mapping discrepancies

#ifdef CONFIG_BT_NIMBLE_ENABLED

namespace {

// ── Module state ─────────────────────────────────────────────────────────────
static bool s_active = false;
static sources::ShuntSource* s_shunt = nullptr;
static sources::MpptSource*  s_mppt  = nullptr;
// Coexistence diagnostic: total GAP_EVENT_DISC events received (all devices,
// not just Victron). Counts every advertisement report that passes from the
// controller through VHCI to the NimBLE host task on Core 0. With
// filter_duplicates=0 and a dense BLE environment this can be hundreds per
// 200 ms scan window, revealing Core 0 CPU load from the NimBLE host task.
static uint32_t s_gap_event_count = 0;

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

// Parse "AA:BB:CC:DD:EE:FF" into 6-byte MAC. Returns false on bad format.
static bool parse_mac(const char* mac_str, uint8_t* out) {
  if (!mac_str || strlen(mac_str) < 17) return false;
  for (int i = 0; i < 6; i++) {
    int hi = hex_nibble(mac_str[i * 3]);
    int lo = hex_nibble(mac_str[i * 3 + 1]);
    if (hi < 0 || lo < 0) return false;
    out[i] = (uint8_t)((hi << 4) | lo);
    if (i < 5 && mac_str[i * 3 + 2] != ':') return false;
  }
  return true;
}

// AES-128-CTR decrypt using mbedTLS.
// nonce_counter = { iv_byte, 0,0,0,0,0,0,0, 0,0,0,0,0,0,0,1 }
static bool aes_ctr_decrypt(const uint8_t* key, uint8_t iv_byte,
                             const uint8_t* in, uint8_t* out, size_t len) {
  mbedtls_aes_context ctx;
  mbedtls_aes_init(&ctx);

  if (mbedtls_aes_setkey_enc(&ctx, key, 128) != 0) {
    mbedtls_aes_free(&ctx);
    return false;
  }

  uint8_t nonce_counter[16] = {};
  nonce_counter[0]  = iv_byte;  // nonce = [iv_byte, 0..0] (8 bytes)
  nonce_counter[15] = 1;        // counter = 1 (big-endian)

  uint8_t stream_block[16] = {};
  size_t  nc_off = 0;

  int rc = mbedtls_aes_crypt_ctr(&ctx, len, &nc_off, nonce_counter,
                                  stream_block, in, out);
  mbedtls_aes_free(&ctx);
  return (rc == 0);
}

// ── Decode MPPT (record type 0x01) ───────────────────────────────────────────
static void decode_mppt(const uint8_t* data, size_t len, uint32_t now_ms) {
  if (!s_mppt || !s_mppt_key_valid || len < 4) return;

  uint8_t iv     = data[3];
  const uint8_t* enc = data + 4;
  size_t enc_len = len - 4;
  if (enc_len < 7) return;

  uint8_t plain[32] = {};
  if (!aes_ctr_decrypt(s_mppt_key, iv, enc, plain, enc_len)) return;

  // uint8_t  state      = plain[0];
  uint16_t pv_power_raw = (uint16_t)(plain[1] | (plain[2] << 8));
  uint16_t batt_v_raw   = (uint16_t)(plain[3] | (plain[4] << 8));
  int16_t  batt_i_raw   = (int16_t)(plain[5] | (plain[6] << 8));

  float pv_power_w   = (float)pv_power_raw;           // W, integer
  float batt_volt_v  = (float)batt_v_raw  / 100.0f;   // V
  float batt_curr_a  = (float)batt_i_raw  / 10.0f;    // A

  // PV current = PV power / PV voltage; PV voltage is not directly in this record.
  // For Phase A, PV voltage and PV current are derived from power + batt voltage
  // as approximations (exact PV panel voltage is in a different register).
  // Owner should verify the decoded PV power matches VictronConnect readout.
  float pv_current_a  = 0.0f;
  float pv_voltage_v  = 0.0f;
  if (batt_volt_v > 1.0f && pv_power_w > 0.0f) {
    // Approximation: PV panel voltage is typically batt_v * 1.2..3.0 for MPPT
    // The exact panel voltage is not in this advertisement record type.
    // Phase B can add the full MPPT data record (record 0x0B) if needed.
    pv_current_a = batt_curr_a;  // MPPT output current ≈ battery charge current
    pv_voltage_v = (batt_curr_a > 0.1f) ? (pv_power_w / batt_curr_a) : 0.0f;
  }

  s_mppt->set_decoded_values(pv_power_w, pv_voltage_v, pv_current_a,
                              batt_volt_v, batt_curr_a, now_ms);

  ESP_LOGD(TAG, "MPPT: pv_power=%.0f W  batt=%.2f V  %.2f A",
           pv_power_w, batt_volt_v, batt_curr_a);
}

// ── Decode SmartShunt (record type 0x02) ─────────────────────────────────────
static void decode_shunt(const uint8_t* data, size_t len, uint32_t now_ms) {
  if (!s_shunt || !s_shunt_key_valid || len < 4) return;

  uint8_t iv      = data[3];
  const uint8_t* enc = data + 4;
  size_t enc_len  = len - 4;
  if (enc_len < 8) return;

  uint8_t plain[32] = {};
  if (!aes_ctr_decrypt(s_shunt_key, iv, enc, plain, enc_len)) return;

  uint16_t batt_v_raw  = (uint16_t)(plain[0] | (plain[1] << 8));
  int16_t  curr_raw    = (int16_t) (plain[2] | (plain[3] << 8));
  // uint16_t remain_raw = (uint16_t)(plain[4] | (plain[5] << 8));  // Ah*10
  uint16_t soc_raw     = (uint16_t)(plain[6] | (plain[7] << 8));

  float voltage_v = (float)batt_v_raw / 100.0f;   // V
  float current_a = (float)curr_raw   / 1000.0f;  // A (mA resolution)
  float soc_pct   = (float)soc_raw    / 10.0f;    // % (0-100)

  s_shunt->set_decoded_values(current_a, voltage_v, soc_pct, now_ms);

  // Key is never logged. Values are logged at DEBUG only.
  ESP_LOGD(TAG, "Shunt: %.3f A  %.2f V  %.1f%%", current_a, voltage_v, soc_pct);
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

  uint8_t record_type = md[2];

  if (record_type == 0x01 && s_mppt_enabled && s_mppt_mac_valid) {
    if (mac_matches(disc.addr.val, s_mppt_mac)) {
      decode_mppt(md, fields.mfg_data_len, now_ms);
    }
  } else if (record_type == 0x02 && s_shunt_enabled && s_shunt_mac_valid) {
    if (mac_matches(disc.addr.val, s_shunt_mac)) {
      decode_shunt(md, fields.mfg_data_len, now_ms);
    }
  }

  return 0;
}

// ── Scan start helper ─────────────────────────────────────────────────────────
static void start_scan() {
  struct ble_gap_disc_params disc_params = {};
  disc_params.passive          = 1;   // no SCAN_REQ (observer-only)
  disc_params.filter_duplicates = 0;  // want every beacon for freshness
  // Scan interval 2 s, window 200 ms → 10% duty cycle. Balances freshness vs
  // radio coexistence with WiFi (both use the 2.4 GHz band).
  disc_params.itvl   = 3200;  // 2000 ms / 0.625 ms
  disc_params.window =  320;  //  200 ms / 0.625 ms

  // Log effective duty cycle at runtime for coexistence analysis.
  // itvl and window are in 0.625 ms units.
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
  ESP_LOGI(TAG, "NimBLE host task started");
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
  if (s_shunt_enabled) {
    s_shunt_mac_valid = parse_mac(cfg.ble_shunt_mac, s_shunt_mac);
    s_shunt_key_valid = parse_hex_key(cfg.ble_shunt_key, s_shunt_key);
    if (!s_shunt_mac_valid)
      ESP_LOGW(TAG, "shunt: invalid MAC \"%s\" — shunt decode disabled", cfg.ble_shunt_mac);
    if (!s_shunt_key_valid)
      ESP_LOGW(TAG, "shunt: invalid key (check 32-char hex) — shunt decode disabled");
  }
  if (s_mppt_enabled) {
    s_mppt_mac_valid = parse_mac(cfg.ble_mppt_mac, s_mppt_mac);
    s_mppt_key_valid = parse_hex_key(cfg.ble_mppt_key, s_mppt_key);
    if (!s_mppt_mac_valid)
      ESP_LOGW(TAG, "mppt: invalid MAC \"%s\" — mppt decode disabled", cfg.ble_mppt_mac);
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

}  // namespace sources::ble_scanner
