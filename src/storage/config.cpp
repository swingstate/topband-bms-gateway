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

  // v4 additions
  // sender_name: empty — notify module uses device hostname at runtime
  // notify_alert_flags: safety-critical events enabled by default.
  // Bit positions match SafetyState::SafetyEvent enum values (safety_state.h):
  // BmsWentOffline=1, PackOvervoltStart=3, PackUndervoltStart=7,
  // TempChargeStop=9, TempDischargeStop=11, CellImbalanceStart=13,
  // BmsReportedAlarm=15, NoPacksOnline=16
  c.notify_alert_flags = (1u<<1)|(1u<<3)|(1u<<7)|(1u<<9)|(1u<<11)|(1u<<13)|(1u<<15)|(1u<<16);
  c.notify_telegram_last_ok_ts = 0;
  c.notify_poll_interval_s = 60;
  c.notify_cooldown_s      = 120;
  c.notify_telegram_verified = false;
  c.notify_debounce_s      = 30;  // 30 s default: filters brief flaps

  // v6 addition: Auto is the default for fresh installs (follows live pack data).
  c.battery_config_mode    = Config::BatteryConfigMode::Auto;

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

// ── Config_v3: schema version 3 layout (644 bytes) ───────────────────────────
// Identical to Config before the v4 notify-wiring fields were appended.
struct Config_v3 {
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
  bool                     notify_telegram_enabled;
  char                     notify_telegram_token[80];
  char                     notify_telegram_chat_id[24];
};

static_assert(sizeof(Config_v3) == 644,
    "Config_v3 size drifted from expected 644 B — check alignment against v3 NVS blobs");

// ── Config_v4: schema version 4 layout (692 bytes) ───────────────────────────
// Identical to Config before notify_debounce_s was placed in the tail padding.
// The v4 NVS blob is 692 B; the former 3-byte tail pad occupies offsets 689-691.
// After v5 the same 692 B blob has: bool(1) + pad(1) + uint16(2) at those offsets.
struct Config_v4 {
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
  bool                     notify_telegram_enabled;
  char                     notify_telegram_token[80];
  char                     notify_telegram_chat_id[24];
  char                     notify_sender_name[32];
  uint32_t                 notify_alert_flags;
  uint32_t                 notify_telegram_last_ok_ts;
  uint16_t                 notify_poll_interval_s;
  uint16_t                 notify_cooldown_s;
  bool                     notify_telegram_verified;
  // 3 bytes tail padding (offset 689-691 in the blob are zero)
};

static_assert(sizeof(Config_v4) == 692,
    "Config_v4 size drifted from expected 692 B — check alignment against v4 NVS blobs");

// ── Config_v5: schema version 5 layout (692 bytes) ───────────────────────────
// Identical to the current Config minus battery_config_mode. Used as the source
// blob for the v5→v6 migration. The only difference from Config_v4 is that
// notify_debounce_s occupies the former tail padding (so no size change).
struct Config_v5 {
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
  bool                     notify_telegram_enabled;
  char                     notify_telegram_token[80];
  char                     notify_telegram_chat_id[24];
  char                     notify_sender_name[32];
  uint32_t                 notify_alert_flags;
  uint32_t                 notify_telegram_last_ok_ts;
  uint16_t                 notify_poll_interval_s;
  uint16_t                 notify_cooldown_s;
  bool                     notify_telegram_verified;
  // 1 byte implicit padding, then:
  uint16_t                 notify_debounce_s;
  // (no tail padding — 692 B total)
};

static_assert(sizeof(Config_v5) == 692,
    "Config_v5 size drifted from expected 692 B — check alignment against v5 NVS blobs");

}  // namespace

// Config layout check (updated each schema version).
// v3 size: 644 B.
// v4 additions: char[32]+uint32+uint32+uint16+uint16+bool = 45 B payload; bool fills
// existing 1-byte tail gap so net growth is 44 B → compiler pads to 692 B.
// v5 addition: uint16_t notify_debounce_s fits in former 3-byte tail padding →
// sizeof(Config_v5) = 692 B.
// v6 addition: BatteryConfigMode (uint8_t) at end → 693 B raw; compiler pads to
// 696 B (4-byte struct alignment, largest member is float).
// This assert catches field additions or reorderings that silently change the NVS blob.
static_assert(sizeof(Config) == 696,
    "Config size drifted from expected 696 B — if intentional, bump schema version, "
    "add a migration, and update this assert");

namespace {

// ── v5 → v6 migration ────────────────────────────────────────────────────────
// battery_config_mode is the only new field. Existing devices (already
// configured) get Manual so their configured charge_amps/discharge_amps/cvl
// values continue to drive limits unchanged. New installs use Auto (DEFAULT_CONFIG).
static bool migrate_v5_to_v6(const uint8_t* buf, size_t len, Config& out) {
  if (len < sizeof(Config_v5)) return false;

  Config_v5 v5;
  memcpy(&v5, buf, sizeof(Config_v5));

  // Start from DEFAULT_CONFIG (battery_config_mode = Auto as safe default
  // for new fields); then overlay all v5 fields below.
  out = DEFAULT_CONFIG;

  out.board_preset            = v5.board_preset;
  out.pins                    = v5.pins;
  out.rs485_enabled           = v5.rs485_enabled;
  out.bms_count               = v5.bms_count;
  out.force_cell_count        = v5.force_cell_count;
  out.can_protocol            = v5.can_protocol;
  out.can_enabled             = v5.can_enabled;
  out.charge_amps_per_pack    = v5.charge_amps_per_pack;
  out.discharge_amps_per_pack = v5.discharge_amps_per_pack;
  out.cvl_voltage             = v5.cvl_voltage;
  out.safe_pack_volt          = v5.safe_pack_volt;
  out.safe_cell_volt          = v5.safe_cell_volt;
  out.safe_cell_drift         = v5.safe_cell_drift;
  out.spike_volt_max          = v5.spike_volt_max;
  out.spike_curr_max          = v5.spike_curr_max;
  out.spike_soc_max           = v5.spike_soc_max;
  out.charge_temp_min         = v5.charge_temp_min;
  out.charge_temp_max         = v5.charge_temp_max;
  out.discharge_temp_min      = v5.discharge_temp_min;
  out.discharge_temp_max      = v5.discharge_temp_max;
  out.temp_soft_zone          = v5.temp_soft_zone;
  out.temp_mode               = v5.temp_mode;
  out.soc_mode                = v5.soc_mode;
  out.setup_mode              = v5.setup_mode;
  out.auto_from_bms_applied   = v5.auto_from_bms_applied;
  out.maint_charge_enabled    = v5.maint_charge_enabled;
  out.maint_target_voltage    = v5.maint_target_voltage;
  out.auto_balance_enabled    = v5.auto_balance_enabled;
  out.auto_balance_last_ts    = v5.auto_balance_last_ts;
  memcpy(out.wifi_ssid,       v5.wifi_ssid,       sizeof(out.wifi_ssid));
  memcpy(out.ntp_server,      v5.ntp_server,      sizeof(out.ntp_server));
  out.timezone_offset_h       = v5.timezone_offset_h;
  out.mqtt_enabled            = v5.mqtt_enabled;
  memcpy(out.mqtt_host,       v5.mqtt_host,       sizeof(out.mqtt_host));
  out.mqtt_port               = v5.mqtt_port;
  memcpy(out.mqtt_user,       v5.mqtt_user,       sizeof(out.mqtt_user));
  memcpy(out.mqtt_pass_obf,   v5.mqtt_pass_obf,   sizeof(out.mqtt_pass_obf));
  memcpy(out.mqtt_base_topic, v5.mqtt_base_topic, sizeof(out.mqtt_base_topic));
  out.mqtt_level              = v5.mqtt_level;
  out.mqtt_diag_enabled       = v5.mqtt_diag_enabled;
  out.ha_discovery_enabled    = v5.ha_discovery_enabled;
  out.mqtt_full_publish       = v5.mqtt_full_publish;
  out.auth_enabled            = v5.auth_enabled;
  memcpy(out.auth_user, v5.auth_user, sizeof(out.auth_user));
  memcpy(out.auth_hash, v5.auth_hash, sizeof(out.auth_hash));
  out.theme_id                = v5.theme_id;
  out.chart_series_a          = v5.chart_series_a;
  out.chart_series_b          = v5.chart_series_b;
  out.ui_poll_live_ms         = v5.ui_poll_live_ms;
  out.ui_poll_diag_ms         = v5.ui_poll_diag_ms;
  out.ui_poll_alerts_ms       = v5.ui_poll_alerts_ms;
  out.last_reset_ts           = v5.last_reset_ts;
  out.serial_debug_enabled    = v5.serial_debug_enabled;
  out.spy_persist_default     = v5.spy_persist_default;
  out.notify_telegram_enabled  = v5.notify_telegram_enabled;
  memcpy(out.notify_telegram_token,   v5.notify_telegram_token,   sizeof(out.notify_telegram_token));
  memcpy(out.notify_telegram_chat_id, v5.notify_telegram_chat_id, sizeof(out.notify_telegram_chat_id));
  memcpy(out.notify_sender_name,      v5.notify_sender_name,      sizeof(out.notify_sender_name));
  out.notify_alert_flags          = v5.notify_alert_flags;
  out.notify_telegram_last_ok_ts  = v5.notify_telegram_last_ok_ts;
  out.notify_poll_interval_s      = v5.notify_poll_interval_s;
  out.notify_cooldown_s           = v5.notify_cooldown_s;
  out.notify_telegram_verified    = v5.notify_telegram_verified;
  out.notify_debounce_s           = v5.notify_debounce_s;
  // battery_config_mode: existing devices that had configured limits → Manual.
  // This preserves their exact runtime behavior (config-based limits + sysparam
  // cap in the safe direction). They can switch to Auto in Settings → Battery.
  out.battery_config_mode = Config::BatteryConfigMode::Manual;

  out.schema_version = CURRENT_SCHEMA_VERSION;
  return true;
}

// ── v4 → v5 migration ────────────────────────────────────────────────────────
// notify_debounce_s occupies the former tail padding bytes (offset 690, size 2).
// A v4 blob has zeros there, which would give debounce=0 (immediate — no debounce).
// Migration sets the default (30 s) so existing users get the intended behavior.
static bool migrate_v4_to_v5(const uint8_t* buf, size_t len, Config& out) {
  if (len < sizeof(Config_v4)) return false;

  Config_v4 v4;
  memcpy(&v4, buf, sizeof(Config_v4));

  // Start from DEFAULT_CONFIG so notify_debounce_s gets its safe default (30).
  out = DEFAULT_CONFIG;

  out.board_preset            = v4.board_preset;
  out.pins                    = v4.pins;
  out.rs485_enabled           = v4.rs485_enabled;
  out.bms_count               = v4.bms_count;
  out.force_cell_count        = v4.force_cell_count;
  out.can_protocol            = v4.can_protocol;
  out.can_enabled             = v4.can_enabled;
  out.charge_amps_per_pack    = v4.charge_amps_per_pack;
  out.discharge_amps_per_pack = v4.discharge_amps_per_pack;
  out.cvl_voltage             = v4.cvl_voltage;
  out.safe_pack_volt          = v4.safe_pack_volt;
  out.safe_cell_volt          = v4.safe_cell_volt;
  out.safe_cell_drift         = v4.safe_cell_drift;
  out.spike_volt_max          = v4.spike_volt_max;
  out.spike_curr_max          = v4.spike_curr_max;
  out.spike_soc_max           = v4.spike_soc_max;
  out.charge_temp_min         = v4.charge_temp_min;
  out.charge_temp_max         = v4.charge_temp_max;
  out.discharge_temp_min      = v4.discharge_temp_min;
  out.discharge_temp_max      = v4.discharge_temp_max;
  out.temp_soft_zone          = v4.temp_soft_zone;
  out.temp_mode               = v4.temp_mode;
  out.soc_mode                = v4.soc_mode;
  out.setup_mode              = v4.setup_mode;
  out.auto_from_bms_applied   = v4.auto_from_bms_applied;
  out.maint_charge_enabled    = v4.maint_charge_enabled;
  out.maint_target_voltage    = v4.maint_target_voltage;
  out.auto_balance_enabled    = v4.auto_balance_enabled;
  out.auto_balance_last_ts    = v4.auto_balance_last_ts;
  memcpy(out.wifi_ssid,       v4.wifi_ssid,       sizeof(out.wifi_ssid));
  memcpy(out.ntp_server,      v4.ntp_server,      sizeof(out.ntp_server));
  out.timezone_offset_h       = v4.timezone_offset_h;
  out.mqtt_enabled            = v4.mqtt_enabled;
  memcpy(out.mqtt_host,       v4.mqtt_host,       sizeof(out.mqtt_host));
  out.mqtt_port               = v4.mqtt_port;
  memcpy(out.mqtt_user,       v4.mqtt_user,       sizeof(out.mqtt_user));
  memcpy(out.mqtt_pass_obf,   v4.mqtt_pass_obf,   sizeof(out.mqtt_pass_obf));
  memcpy(out.mqtt_base_topic, v4.mqtt_base_topic, sizeof(out.mqtt_base_topic));
  out.mqtt_level              = v4.mqtt_level;
  out.mqtt_diag_enabled       = v4.mqtt_diag_enabled;
  out.ha_discovery_enabled    = v4.ha_discovery_enabled;
  out.mqtt_full_publish       = v4.mqtt_full_publish;
  out.auth_enabled            = v4.auth_enabled;
  memcpy(out.auth_user, v4.auth_user, sizeof(out.auth_user));
  memcpy(out.auth_hash, v4.auth_hash, sizeof(out.auth_hash));
  out.theme_id                = v4.theme_id;
  out.chart_series_a          = v4.chart_series_a;
  out.chart_series_b          = v4.chart_series_b;
  out.ui_poll_live_ms         = v4.ui_poll_live_ms;
  out.ui_poll_diag_ms         = v4.ui_poll_diag_ms;
  out.ui_poll_alerts_ms       = v4.ui_poll_alerts_ms;
  out.last_reset_ts           = v4.last_reset_ts;
  out.serial_debug_enabled    = v4.serial_debug_enabled;
  out.spy_persist_default     = v4.spy_persist_default;
  out.notify_telegram_enabled  = v4.notify_telegram_enabled;
  memcpy(out.notify_telegram_token,   v4.notify_telegram_token,   sizeof(out.notify_telegram_token));
  memcpy(out.notify_telegram_chat_id, v4.notify_telegram_chat_id, sizeof(out.notify_telegram_chat_id));
  memcpy(out.notify_sender_name,      v4.notify_sender_name,      sizeof(out.notify_sender_name));
  out.notify_alert_flags          = v4.notify_alert_flags;
  out.notify_telegram_last_ok_ts  = v4.notify_telegram_last_ok_ts;
  out.notify_poll_interval_s      = v4.notify_poll_interval_s;
  out.notify_cooldown_s           = v4.notify_cooldown_s;
  out.notify_telegram_verified    = v4.notify_telegram_verified;
  // notify_debounce_s: stays at DEFAULT_CONFIG value (30 s).

  out.schema_version = CURRENT_SCHEMA_VERSION;
  return true;
}

static bool migrate_v3_to_v4(const uint8_t* buf, size_t len, Config& out) {
  if (len < sizeof(Config_v3)) return false;

  Config_v3 v3;
  memcpy(&v3, buf, sizeof(Config_v3));

  // Start from DEFAULT_CONFIG so all v4-only fields get safe default values.
  out = DEFAULT_CONFIG;

  out.board_preset            = v3.board_preset;
  out.pins                    = v3.pins;
  out.rs485_enabled           = v3.rs485_enabled;
  out.bms_count               = v3.bms_count;
  out.force_cell_count        = v3.force_cell_count;
  out.can_protocol            = v3.can_protocol;
  out.can_enabled             = v3.can_enabled;
  out.charge_amps_per_pack    = v3.charge_amps_per_pack;
  out.discharge_amps_per_pack = v3.discharge_amps_per_pack;
  out.cvl_voltage             = v3.cvl_voltage;
  out.safe_pack_volt          = v3.safe_pack_volt;
  out.safe_cell_volt          = v3.safe_cell_volt;
  out.safe_cell_drift         = v3.safe_cell_drift;
  out.spike_volt_max          = v3.spike_volt_max;
  out.spike_curr_max          = v3.spike_curr_max;
  out.spike_soc_max           = v3.spike_soc_max;
  out.charge_temp_min         = v3.charge_temp_min;
  out.charge_temp_max         = v3.charge_temp_max;
  out.discharge_temp_min      = v3.discharge_temp_min;
  out.discharge_temp_max      = v3.discharge_temp_max;
  out.temp_soft_zone          = v3.temp_soft_zone;
  out.temp_mode               = v3.temp_mode;
  out.soc_mode                = v3.soc_mode;
  out.setup_mode              = v3.setup_mode;
  out.auto_from_bms_applied   = v3.auto_from_bms_applied;
  out.maint_charge_enabled    = v3.maint_charge_enabled;
  out.maint_target_voltage    = v3.maint_target_voltage;
  out.auto_balance_enabled    = v3.auto_balance_enabled;
  out.auto_balance_last_ts    = v3.auto_balance_last_ts;
  memcpy(out.wifi_ssid,       v3.wifi_ssid,       sizeof(out.wifi_ssid));
  memcpy(out.ntp_server,      v3.ntp_server,      sizeof(out.ntp_server));
  out.timezone_offset_h       = v3.timezone_offset_h;
  out.mqtt_enabled            = v3.mqtt_enabled;
  memcpy(out.mqtt_host,       v3.mqtt_host,       sizeof(out.mqtt_host));
  out.mqtt_port               = v3.mqtt_port;
  memcpy(out.mqtt_user,       v3.mqtt_user,       sizeof(out.mqtt_user));
  memcpy(out.mqtt_pass_obf,   v3.mqtt_pass_obf,   sizeof(out.mqtt_pass_obf));
  memcpy(out.mqtt_base_topic, v3.mqtt_base_topic, sizeof(out.mqtt_base_topic));
  out.mqtt_level              = v3.mqtt_level;
  out.mqtt_diag_enabled       = v3.mqtt_diag_enabled;
  out.ha_discovery_enabled    = v3.ha_discovery_enabled;
  out.mqtt_full_publish       = v3.mqtt_full_publish;
  out.auth_enabled            = v3.auth_enabled;
  memcpy(out.auth_user, v3.auth_user, sizeof(out.auth_user));
  memcpy(out.auth_hash, v3.auth_hash, sizeof(out.auth_hash));
  out.theme_id                = v3.theme_id;
  out.chart_series_a          = v3.chart_series_a;
  out.chart_series_b          = v3.chart_series_b;
  out.ui_poll_live_ms         = v3.ui_poll_live_ms;
  out.ui_poll_diag_ms         = v3.ui_poll_diag_ms;
  out.ui_poll_alerts_ms       = v3.ui_poll_alerts_ms;
  out.last_reset_ts           = v3.last_reset_ts;
  out.serial_debug_enabled    = v3.serial_debug_enabled;
  out.spy_persist_default     = v3.spy_persist_default;
  out.notify_telegram_enabled  = v3.notify_telegram_enabled;
  memcpy(out.notify_telegram_token,   v3.notify_telegram_token,   sizeof(out.notify_telegram_token));
  memcpy(out.notify_telegram_chat_id, v3.notify_telegram_chat_id, sizeof(out.notify_telegram_chat_id));
  // v4 fields: sender_name, alert_flags, last_ok_ts, poll_interval, cooldown, verified
  // stay at DEFAULT_CONFIG values (safe defaults from make_default()).

  out.schema_version = CURRENT_SCHEMA_VERSION;
  return true;
}

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

  if (ver == 5) {
    // Field-preserving v5 → v6. All v5 settings survive;
    // battery_config_mode defaults to Manual (existing configured device).
    return migrate_v5_to_v6(buf, len, out);
  }

  if (ver == 4) {
    // Field-preserving v4 → v5. All v4 settings survive;
    // notify_debounce_s gets safe default (30 s) from DEFAULT_CONFIG.
    return migrate_v4_to_v5(buf, len, out);
  }

  if (ver == 3) {
    // Field-preserving v3 → v4 → v5. All v3 settings survive;
    // notify-wiring fields (sender_name, flags, intervals, verified) default safely.
    return migrate_v3_to_v4(buf, len, out);
  }

  if (ver == 2) {
    // migrate_v2_to_v3 starts from DEFAULT_CONFIG (which now carries v4 defaults),
    // overlays all v2 fields, and sets schema_version = CURRENT (4).
    // The v3 notify fields and v4 alert-wiring fields stay at safe defaults.
    return migrate_v2_to_v3(buf, len, out);
  }

  if (ver == 1) {
    // migrate_v1_to_v2 also starts from DEFAULT_CONFIG and produces a full v4 Config.
    return migrate_v1_to_v2(buf, len, out);
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

  // Cell voltage plausibility [2.0, 3.65] V — upper bound is the hard LiFePO4
  // sanity cap (LIFEPO4_CELL_MAX_V). The gateway enforces this at runtime in
  // all battery config modes; reject values above it at the API boundary too.
  if (cfg.safe_cell_volt < 2.0f || cfg.safe_cell_volt > 3.65f) {
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
