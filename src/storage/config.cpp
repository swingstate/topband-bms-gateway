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

  return c;
}

const Config DEFAULT_CONFIG = make_default();

// ── serialize / deserialize ───────────────────────────────────────────────────

namespace storage {

bool serialize(const Config& cfg, uint8_t* buf, size_t buf_size, size_t& out_len) {
  if (buf_size < sizeof(Config)) return false;
  memcpy(buf, &cfg, sizeof(Config));
  out_len = sizeof(Config);
  return true;
}

bool deserialize(const uint8_t* buf, size_t len, Config& out) {
  if (len < sizeof(Config)) return false;
  uint16_t ver = 0;
  memcpy(&ver, buf, sizeof(ver));   // schema_version is at offset 0
  if (ver != CURRENT_SCHEMA_VERSION) return false;
  memcpy(&out, buf, sizeof(Config));
  return true;
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
