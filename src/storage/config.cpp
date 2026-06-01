#include "config.h"
#include <array>
#include <cstring>

// ── CRC-32/IEEE lookup table (constexpr, computed at compile time) ────────────
// Polynomial: 0xEDB88320 (reflected form). No IDF dependency; host-portable.
// Verification: crc32("123456789") == 0xCBF43926.
namespace {

constexpr std::array<uint32_t, 256> make_crc32_table() {
  std::array<uint32_t, 256> t{};
  for (uint32_t i = 0; i < 256; ++i) {
    uint32_t c = i;
    for (int k = 0; k < 8; ++k) {
      c = (c & 1u) ? (0xEDB88320u ^ (c >> 1)) : (c >> 1);
    }
    t[i] = c;
  }
  return t;
}

static constexpr auto k_crc_table = make_crc32_table();

}  // namespace

// ── DEFAULT_CONFIG ────────────────────────────────────────────────────────────
// Waveshare pin map from V2.67 legacy code; thresholds from V2.67 defaults.
// Zero-initialise the struct first so all char arrays start as empty strings.
static Config make_default() {
  Config c{};
  c.schema_version      = CURRENT_SCHEMA_VERSION;
  c.board_preset        = Config::BoardPreset::Waveshare;

  // Waveshare ESP32-S3 RS485/CAN Controller pin assignments (from legacy)
  c.pins.rs485_tx  = 17;
  c.pins.rs485_rx  = 18;
  c.pins.rs485_dir = 21;
  c.pins.can_tx    = 15;
  c.pins.can_rx    = 16;
  c.pins.led       = 38;
  c.rs485_enabled  = true;

  c.bms_count         = 2;
  c.force_cell_count  = 0;         // auto-detect

  c.can_protocol      = Config::CanProtocol::Victron;
  c.can_enabled       = true;

  // V2.67 defaults (g_charge_amps, g_discharge_amps, g_cvl_voltage)
  c.charge_amps_per_pack    = 30.0f;
  c.discharge_amps_per_pack = 30.0f;
  c.cvl_voltage             = 52.5f;

  // Safety thresholds (V2.67: g_safe_volt, g_safe_cell, g_safe_drift)
  c.safe_pack_volt   = 53.25f;
  c.safe_cell_volt   = 3.55f;
  c.safe_cell_drift  = 0.20f;

  // Spike filter (V2.67: g_spike_volt, g_spike_cur, g_spike_soc)
  c.spike_volt_max = 5.0f;
  c.spike_curr_max = 250.0f;
  c.spike_soc_max  = 20;

  // Temperature limits (V2.67: g_tc_min/max, g_td_min/max)
  c.charge_temp_min    = 5.0f;
  c.charge_temp_max    = 50.0f;
  c.discharge_temp_min = -20.0f;
  c.discharge_temp_max = 60.0f;
  c.temp_soft_zone     = 5.0f;
  c.temp_mode          = Config::TempMode::Hottest;

  c.soc_mode             = Config::SocMode::Calculated;
  c.setup_mode           = Config::SetupMode::Wizard;
  c.auto_from_bms_applied = false;

  c.maint_charge_enabled  = false;
  c.maint_target_voltage  = 55.2f;   // 16S × 3.45 V

  c.auto_balance_enabled  = false;
  c.auto_balance_last_ts  = 0;

  // wifi_ssid: empty (captive portal fills it on first boot)
  strncpy(c.ntp_server, "pool.ntp.org", sizeof(c.ntp_server) - 1);
  c.timezone_offset_h = 0;

  c.mqtt_enabled         = false;
  c.mqtt_port            = 1883;
  c.mqtt_level           = Config::MqttLevel::StatusOnly;
  c.mqtt_diag_enabled    = false;
  c.ha_discovery_enabled = false;
  c.mqtt_full_publish    = false;

  c.auth_enabled = false;
  strncpy(c.auth_user, "admin", sizeof(c.auth_user) - 1);
  // auth_hash: empty (auth disabled by default)

  c.theme_id       = 0;   // light
  c.chart_series_a = 0;   // power
  c.chart_series_b = 1;   // voltage

  c.ui_poll_live_ms   = 1500;
  c.ui_poll_diag_ms   = 5000;
  c.ui_poll_alerts_ms = 30000;

  c.last_reset_ts = 0;

  c.serial_debug_enabled = false;
  c.spy_persist_default  = false;

  c.notify_telegram_enabled  = false;
  // token and chat_id start empty (zero-init from Config{})

  return c;
}

const Config DEFAULT_CONFIG = make_default();

// ── Schema migration ──────────────────────────────────────────────────────────
//
// Pattern for every schema bump: define a struct that exactly mirrors the old
// layout, add a migrate_vN_to_vM() function that starts from DEFAULT_CONFIG
// (guarantees all new fields have safe values) then overlays every field that
// existed in the old schema. Dispatch from deserialize() on schema_version.
//
// v1 → v2 alignment note:
//   v1 ends the hardware block at offset 12 (can_enabled) with 3 padding bytes
//   before charge_amps_per_pack at offset 16.
//   v2 inserts rs485_enabled (bool, 1B) at offset 9; the padding shrinks to 2B,
//   so charge_amps_per_pack stays at offset 16. Overall struct size is 540 B in
//   both (verified by the static_assert below).
//
// v2 → v3:
//   Adds notify_telegram_enabled (1B), notify_telegram_token[80], and
//   notify_telegram_chat_id[24] at the end of the struct.  Struct grows from
//   540 B to 645 B (with alignment padding).  Explicit field-copy migration:
//   start from DEFAULT_CONFIG, overlay all v2 fields from the blob, new notify
//   fields take safe defaults (disabled, empty strings).

namespace {

// Historical Config layout at schema version 1.
// The only structural difference from Config (v2) is the absence of rs485_enabled
// between pins and bms_count.
struct Config_v1 {
  uint16_t                 schema_version;
  Config::BoardPreset      board_preset;
  Config::PinMap           pins;
  uint8_t                  bms_count;
  uint8_t                  force_cell_count;
  Config::CanProtocol      can_protocol;
  bool                     can_enabled;
  float                    charge_amps_per_pack;
  float                    discharge_amps_per_pack;
  float                    cvl_voltage;
  float                    safe_pack_volt;
  float                    safe_cell_volt;
  float                    safe_cell_drift;
  float                    spike_volt_max;
  float                    spike_curr_max;
  uint8_t                  spike_soc_max;
  float                    charge_temp_min;
  float                    charge_temp_max;
  float                    discharge_temp_min;
  float                    discharge_temp_max;
  float                    temp_soft_zone;
  Config::TempMode         temp_mode;
  Config::SocMode          soc_mode;
  Config::SetupMode        setup_mode;
  bool                     auto_from_bms_applied;
  bool                     maint_charge_enabled;
  float                    maint_target_voltage;
  bool                     auto_balance_enabled;
  uint32_t                 auto_balance_last_ts;
  char                     wifi_ssid[33];
  char                     ntp_server[64];
  int8_t                   timezone_offset_h;
  bool                     mqtt_enabled;
  char                     mqtt_host[64];
  uint16_t                 mqtt_port;
  char                     mqtt_user[32];
  char                     mqtt_pass_obf[64];
  char                     mqtt_base_topic[64];
  Config::MqttLevel        mqtt_level;
  bool                     mqtt_diag_enabled;
  bool                     ha_discovery_enabled;
  bool                     mqtt_full_publish;
  bool                     auth_enabled;
  char                     auth_user[32];
  char                     auth_hash[65];
  uint8_t                  theme_id;
  uint8_t                  chart_series_a;
  uint8_t                  chart_series_b;
  uint16_t                 ui_poll_live_ms;
  uint16_t                 ui_poll_diag_ms;
  uint16_t                 ui_poll_alerts_ms;
  uint32_t                 last_reset_ts;
  bool                     serial_debug_enabled;
  bool                     spy_persist_default;
};

// Config_v1 and Config_v2 have the same byte layout (rs485_enabled steals a
// padding byte, so net size is unchanged at 540 B).  Config (v3) is larger
// due to the notify fields appended at the end.
static_assert(sizeof(Config_v1) == 540,
    "Config_v1 size drifted — check alignment after any Config field change");

static bool migrate_v1_to_v2(const uint8_t* buf, size_t len, Config& out) {
  if (len < sizeof(Config_v1)) return false;

  Config_v1 v1;
  memcpy(&v1, buf, sizeof(Config_v1));

  // Start from DEFAULT_CONFIG so rs485_enabled and every other new v2 field
  // has a safe value before we overlay the preserved v1 settings.
  out = DEFAULT_CONFIG;

  // Hardware — board type and pins carry forward unchanged.
  // rs485_enabled: v2-only field; default true preserves existing RS485 behaviour
  // (all v1 devices had RS485 hardware). board_preset=1 was LilyGo in v1 (renamed
  // Manual in v2); value is binary-compatible, so no translation needed.
  out.board_preset          = v1.board_preset;
  out.pins                  = v1.pins;
  out.rs485_enabled         = true;

  // Battery topology
  out.bms_count             = v1.bms_count;
  out.force_cell_count      = v1.force_cell_count;

  // Inverter / CAN
  out.can_protocol          = v1.can_protocol;
  out.can_enabled           = v1.can_enabled;

  // Charge / discharge limits
  out.charge_amps_per_pack    = v1.charge_amps_per_pack;
  out.discharge_amps_per_pack = v1.discharge_amps_per_pack;
  out.cvl_voltage             = v1.cvl_voltage;

  // Safety cutoffs
  out.safe_pack_volt        = v1.safe_pack_volt;
  out.safe_cell_volt        = v1.safe_cell_volt;
  out.safe_cell_drift       = v1.safe_cell_drift;

  // Spike filter
  out.spike_volt_max        = v1.spike_volt_max;
  out.spike_curr_max        = v1.spike_curr_max;
  out.spike_soc_max         = v1.spike_soc_max;

  // Temperature
  out.charge_temp_min       = v1.charge_temp_min;
  out.charge_temp_max       = v1.charge_temp_max;
  out.discharge_temp_min    = v1.discharge_temp_min;
  out.discharge_temp_max    = v1.discharge_temp_max;
  out.temp_soft_zone        = v1.temp_soft_zone;
  out.temp_mode             = v1.temp_mode;

  // SOC / setup
  out.soc_mode              = v1.soc_mode;
  out.setup_mode            = v1.setup_mode;
  out.auto_from_bms_applied = v1.auto_from_bms_applied;

  // Maintenance charge
  out.maint_charge_enabled  = v1.maint_charge_enabled;
  out.maint_target_voltage  = v1.maint_target_voltage;

  // Auto-balancer
  out.auto_balance_enabled  = v1.auto_balance_enabled;
  out.auto_balance_last_ts  = v1.auto_balance_last_ts;

  // Network
  memcpy(out.wifi_ssid,  v1.wifi_ssid,  sizeof(out.wifi_ssid));
  memcpy(out.ntp_server, v1.ntp_server, sizeof(out.ntp_server));
  out.timezone_offset_h     = v1.timezone_offset_h;

  // MQTT
  out.mqtt_enabled          = v1.mqtt_enabled;
  memcpy(out.mqtt_host,       v1.mqtt_host,       sizeof(out.mqtt_host));
  out.mqtt_port             = v1.mqtt_port;
  memcpy(out.mqtt_user,       v1.mqtt_user,       sizeof(out.mqtt_user));
  memcpy(out.mqtt_pass_obf,   v1.mqtt_pass_obf,   sizeof(out.mqtt_pass_obf));
  memcpy(out.mqtt_base_topic, v1.mqtt_base_topic, sizeof(out.mqtt_base_topic));
  out.mqtt_level            = v1.mqtt_level;
  out.mqtt_diag_enabled     = v1.mqtt_diag_enabled;
  out.ha_discovery_enabled  = v1.ha_discovery_enabled;
  out.mqtt_full_publish     = v1.mqtt_full_publish;

  // Auth
  out.auth_enabled          = v1.auth_enabled;
  memcpy(out.auth_user, v1.auth_user, sizeof(out.auth_user));
  memcpy(out.auth_hash, v1.auth_hash, sizeof(out.auth_hash));

  // UI preferences
  out.theme_id              = v1.theme_id;
  out.chart_series_a        = v1.chart_series_a;
  out.chart_series_b        = v1.chart_series_b;

  // UI polling cadences
  out.ui_poll_live_ms       = v1.ui_poll_live_ms;
  out.ui_poll_diag_ms       = v1.ui_poll_diag_ms;
  out.ui_poll_alerts_ms     = v1.ui_poll_alerts_ms;

  // Counter reset
  out.last_reset_ts         = v1.last_reset_ts;

  // Debug
  out.serial_debug_enabled  = v1.serial_debug_enabled;
  out.spy_persist_default   = v1.spy_persist_default;

  out.schema_version = CURRENT_SCHEMA_VERSION;
  return true;
}

// ── Config_v2: schema version 2 layout (540 bytes) ───────────────────────────
// Identical to the old Config struct before notify fields were appended.
// Defined explicitly so we can safely memcpy a v2 NVS blob into it.
struct Config_v2 {
  uint16_t                 schema_version;
  Config::BoardPreset      board_preset;
  Config::PinMap           pins;
  bool                     rs485_enabled;
  uint8_t                  bms_count;
  uint8_t                  force_cell_count;
  Config::CanProtocol      can_protocol;
  bool                     can_enabled;
  float                    charge_amps_per_pack;
  float                    discharge_amps_per_pack;
  float                    cvl_voltage;
  float                    safe_pack_volt;
  float                    safe_cell_volt;
  float                    safe_cell_drift;
  float                    spike_volt_max;
  float                    spike_curr_max;
  uint8_t                  spike_soc_max;
  float                    charge_temp_min;
  float                    charge_temp_max;
  float                    discharge_temp_min;
  float                    discharge_temp_max;
  float                    temp_soft_zone;
  Config::TempMode         temp_mode;
  Config::SocMode          soc_mode;
  Config::SetupMode        setup_mode;
  bool                     auto_from_bms_applied;
  bool                     maint_charge_enabled;
  float                    maint_target_voltage;
  bool                     auto_balance_enabled;
  uint32_t                 auto_balance_last_ts;
  char                     wifi_ssid[33];
  char                     ntp_server[64];
  int8_t                   timezone_offset_h;
  bool                     mqtt_enabled;
  char                     mqtt_host[64];
  uint16_t                 mqtt_port;
  char                     mqtt_user[32];
  char                     mqtt_pass_obf[64];
  char                     mqtt_base_topic[64];
  Config::MqttLevel        mqtt_level;
  bool                     mqtt_diag_enabled;
  bool                     ha_discovery_enabled;
  bool                     mqtt_full_publish;
  bool                     auth_enabled;
  char                     auth_user[32];
  char                     auth_hash[65];
  uint8_t                  theme_id;
  uint8_t                  chart_series_a;
  uint8_t                  chart_series_b;
  uint16_t                 ui_poll_live_ms;
  uint16_t                 ui_poll_diag_ms;
  uint16_t                 ui_poll_alerts_ms;
  uint32_t                 last_reset_ts;
  bool                     serial_debug_enabled;
  bool                     spy_persist_default;
};

// Both Config_v1 and Config_v2 must be 540 bytes (same layout, different field
// for the rs485_enabled/padding slot but same total size).
static_assert(sizeof(Config_v2) == sizeof(Config_v1),
    "Config_v2 / Config_v1 size mismatch — padding assumption violated");

static bool migrate_v2_to_v3(const uint8_t* buf, size_t len, Config& out) {
  if (len < sizeof(Config_v2)) return false;

  Config_v2 v2;
  memcpy(&v2, buf, sizeof(Config_v2));

  // Start from DEFAULT_CONFIG so all v3-only fields get safe values.
  out = DEFAULT_CONFIG;

  // Overlay every v2 field.
  out.board_preset            = v2.board_preset;
  out.pins                    = v2.pins;
  out.rs485_enabled           = v2.rs485_enabled;
  out.bms_count               = v2.bms_count;
  out.force_cell_count        = v2.force_cell_count;
  out.can_protocol            = v2.can_protocol;
  out.can_enabled             = v2.can_enabled;
  out.charge_amps_per_pack    = v2.charge_amps_per_pack;
  out.discharge_amps_per_pack = v2.discharge_amps_per_pack;
  out.cvl_voltage             = v2.cvl_voltage;
  out.safe_pack_volt          = v2.safe_pack_volt;
  out.safe_cell_volt          = v2.safe_cell_volt;
  out.safe_cell_drift         = v2.safe_cell_drift;
  out.spike_volt_max          = v2.spike_volt_max;
  out.spike_curr_max          = v2.spike_curr_max;
  out.spike_soc_max           = v2.spike_soc_max;
  out.charge_temp_min         = v2.charge_temp_min;
  out.charge_temp_max         = v2.charge_temp_max;
  out.discharge_temp_min      = v2.discharge_temp_min;
  out.discharge_temp_max      = v2.discharge_temp_max;
  out.temp_soft_zone          = v2.temp_soft_zone;
  out.temp_mode               = v2.temp_mode;
  out.soc_mode                = v2.soc_mode;
  out.setup_mode              = v2.setup_mode;
  out.auto_from_bms_applied   = v2.auto_from_bms_applied;
  out.maint_charge_enabled    = v2.maint_charge_enabled;
  out.maint_target_voltage    = v2.maint_target_voltage;
  out.auto_balance_enabled    = v2.auto_balance_enabled;
  out.auto_balance_last_ts    = v2.auto_balance_last_ts;
  memcpy(out.wifi_ssid,       v2.wifi_ssid,       sizeof(out.wifi_ssid));
  memcpy(out.ntp_server,      v2.ntp_server,      sizeof(out.ntp_server));
  out.timezone_offset_h       = v2.timezone_offset_h;
  out.mqtt_enabled            = v2.mqtt_enabled;
  memcpy(out.mqtt_host,       v2.mqtt_host,       sizeof(out.mqtt_host));
  out.mqtt_port               = v2.mqtt_port;
  memcpy(out.mqtt_user,       v2.mqtt_user,       sizeof(out.mqtt_user));
  memcpy(out.mqtt_pass_obf,   v2.mqtt_pass_obf,   sizeof(out.mqtt_pass_obf));
  memcpy(out.mqtt_base_topic, v2.mqtt_base_topic, sizeof(out.mqtt_base_topic));
  out.mqtt_level              = v2.mqtt_level;
  out.mqtt_diag_enabled       = v2.mqtt_diag_enabled;
  out.ha_discovery_enabled    = v2.ha_discovery_enabled;
  out.mqtt_full_publish       = v2.mqtt_full_publish;
  out.auth_enabled            = v2.auth_enabled;
  memcpy(out.auth_user, v2.auth_user, sizeof(out.auth_user));
  memcpy(out.auth_hash, v2.auth_hash, sizeof(out.auth_hash));
  out.theme_id                = v2.theme_id;
  out.chart_series_a          = v2.chart_series_a;
  out.chart_series_b          = v2.chart_series_b;
  out.ui_poll_live_ms         = v2.ui_poll_live_ms;
  out.ui_poll_diag_ms         = v2.ui_poll_diag_ms;
  out.ui_poll_alerts_ms       = v2.ui_poll_alerts_ms;
  out.last_reset_ts           = v2.last_reset_ts;
  out.serial_debug_enabled    = v2.serial_debug_enabled;
  out.spy_persist_default     = v2.spy_persist_default;
  // notify fields stay at DEFAULT_CONFIG values: disabled, empty strings.

  out.schema_version = CURRENT_SCHEMA_VERSION;
  return true;
}

}  // namespace

// ── serialize / deserialize ───────────────────────────────────────────────────

namespace storage {

bool serialize(const Config& cfg, uint8_t* buf, size_t buf_size, size_t& out_len) {
  if (buf_size < sizeof(Config)) return false;
  memcpy(buf, &cfg, sizeof(Config));
  out_len = sizeof(Config);
  return true;
}

bool deserialize(const uint8_t* buf, size_t len, Config& out) {
  if (len < 2) return false;
  uint16_t ver = 0;
  memcpy(&ver, buf, sizeof(ver));   // schema_version is at offset 0

  if (ver == CURRENT_SCHEMA_VERSION) {
    if (len < sizeof(Config)) return false;
    memcpy(&out, buf, sizeof(Config));
    return true;
  }

  if (ver == 2) {
    // Field-preserving v2 → v3 migration. All existing settings survive;
    // notify fields are defaulted (disabled, empty strings).
    return migrate_v2_to_v3(buf, len, out);
  }

  if (ver == 1) {
    // v1 → v2 → v3: migrate through v2 first, then the notify defaults apply.
    if (!migrate_v1_to_v2(buf, len, out)) return false;
    // out is now at v2 layout semantics with schema_version=CURRENT (3).
    // No further transform needed since v2→v3 only adds new fields at defaults.
    return true;
  }

  return false;  // unrecognised schema version
}

uint32_t crc32(const uint8_t* data, size_t len) {
  uint32_t crc = 0xFFFFFFFFu;
  for (size_t i = 0; i < len; ++i) {
    crc = (crc >> 8) ^ k_crc_table[(crc ^ data[i]) & 0xFFu];
  }
  return ~crc;
}

// ── validate ──────────────────────────────────────────────────────────────────
// Rules per architecture §8.6. Writes the offending field name on failure.

static void set_field(char* out, size_t sz, const char* name) {
  strncpy(out, name, sz - 1);
  out[sz - 1] = '\0';
}

ValidationError validate(const Config& cfg, char* field_out, size_t field_buf) {
  if (field_out && field_buf > 0) field_out[0] = '\0';

  // bms_count in [1, 16]
  if (cfg.bms_count < 1 || cfg.bms_count > 16) {
    set_field(field_out, field_buf, "bms_count");
    return ValidationError::BmsCountOutOfRange;
  }

  // Cell voltage plausibility [2.0, 4.5] V
  if (cfg.safe_cell_volt < 2.0f || cfg.safe_cell_volt > 4.5f) {
    set_field(field_out, field_buf, "safe_cell_volt");
    return ValidationError::CellVoltageOutOfRange;
  }

  // Pack voltage plausibility [20, 65] V (applies to cvl_voltage and safe_pack_volt)
  if (cfg.safe_pack_volt < 20.0f || cfg.safe_pack_volt > 65.0f) {
    set_field(field_out, field_buf, "safe_pack_volt");
    return ValidationError::PackVoltageOutOfRange;
  }
  if (cfg.cvl_voltage < 20.0f || cfg.cvl_voltage > 65.0f) {
    set_field(field_out, field_buf, "cvl_voltage");
    return ValidationError::PackVoltageOutOfRange;
  }

  // Temperature plausibility [-40, 100] °C
  struct { float v; const char* name; } temps[] = {
    {cfg.charge_temp_min,    "charge_temp_min"},
    {cfg.charge_temp_max,    "charge_temp_max"},
    {cfg.discharge_temp_min, "discharge_temp_min"},
    {cfg.discharge_temp_max, "discharge_temp_max"},
  };
  for (auto& t : temps) {
    if (t.v < -40.0f || t.v > 100.0f) {
      set_field(field_out, field_buf, t.name);
      return ValidationError::TemperatureOutOfRange;
    }
  }

  // ESP32-S3 reserved GPIO blocklist (conservative).
  // 26-32: SPI0/1 flash pins (unavailable, internal).
  // 33-37: Octal PSRAM pins on 8MB PSRAM modules (unavailable, internal).
  // 19-20: USB D+/D- (reserved for USB stack; avoid even when CDC unused).
  // 43-44: UART0 TX/RX (debug serial by default; avoid repurposing).
  // 45-46: Strapping pins (boot mode); safe as GPIO post-boot but excluded
  //        to prevent accidental boot-mode interference on next reset.
  static const int8_t RESERVED_GPIOS[] = {
    19, 20,
    26, 27, 28, 29, 30, 31, 32,
    33, 34, 35, 36, 37,
    43, 44, 45, 46,
  };
  static constexpr int8_t GPIO_MAX = 48;

  // Collect all active pins (those that are not the auto/disabled sentinel -1).
  // RS485 pins are active only when rs485_enabled; CAN pins when can_enabled.
  const bool rs485_active = cfg.rs485_enabled;
  const bool dir_active   = rs485_active && (cfg.pins.rs485_dir >= 0);
  const bool can_active   = cfg.can_enabled;

  int8_t all_pins[6];
  size_t n_total = 0;
  if (rs485_active) {
    all_pins[n_total++] = cfg.pins.rs485_tx;
    all_pins[n_total++] = cfg.pins.rs485_rx;
    if (dir_active) all_pins[n_total++] = cfg.pins.rs485_dir;
  }
  if (can_active) {
    all_pins[n_total++] = cfg.pins.can_tx;
    all_pins[n_total++] = cfg.pins.can_rx;
  }
  // LED pin always validated if >= 0 (firmware supports LED even in partial configs).
  if (cfg.pins.led >= 0) all_pins[n_total++] = cfg.pins.led;

  // Range and reserved-pin check.
  for (size_t i = 0; i < n_total; ++i) {
    if (all_pins[i] < 0 || all_pins[i] > GPIO_MAX) {
      set_field(field_out, field_buf, "pins");
      return ValidationError::PinOutOfRange;
    }
    for (int8_t rg : RESERVED_GPIOS) {
      if (all_pins[i] == rg) {
        set_field(field_out, field_buf, "pins");
        return ValidationError::PinReserved;
      }
    }
  }

  // Duplicate check.
  for (size_t i = 0; i < n_total; ++i) {
    for (size_t j = i + 1; j < n_total; ++j) {
      if (all_pins[i] == all_pins[j]) {
        set_field(field_out, field_buf, "pins");
        return ValidationError::PinConflict;
      }
    }
  }

  // String null-termination within buffer bounds
  auto str_ok = [](const char* s, size_t n) -> bool {
    for (size_t i = 0; i < n; ++i) if (s[i] == '\0') return true;
    return false;
  };
  if (!str_ok(cfg.wifi_ssid,        sizeof(cfg.wifi_ssid)))        { set_field(field_out, field_buf, "wifi_ssid");        return ValidationError::StringNotTerminated; }
  if (!str_ok(cfg.ntp_server,       sizeof(cfg.ntp_server)))       { set_field(field_out, field_buf, "ntp_server");       return ValidationError::StringNotTerminated; }
  if (!str_ok(cfg.mqtt_host,        sizeof(cfg.mqtt_host)))        { set_field(field_out, field_buf, "mqtt_host");        return ValidationError::StringNotTerminated; }
  if (!str_ok(cfg.mqtt_base_topic,  sizeof(cfg.mqtt_base_topic)))  { set_field(field_out, field_buf, "mqtt_base_topic");  return ValidationError::StringNotTerminated; }
  if (!str_ok(cfg.auth_user,        sizeof(cfg.auth_user)))        { set_field(field_out, field_buf, "auth_user");        return ValidationError::StringNotTerminated; }
  if (!str_ok(cfg.auth_hash,        sizeof(cfg.auth_hash)))        { set_field(field_out, field_buf, "auth_hash");        return ValidationError::StringNotTerminated; }

  return ValidationError::None;
}

}  // namespace storage
