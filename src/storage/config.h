#pragma once
#include <cstddef>
#include <cstdint>

// ── Schema version ────────────────────────────────────────────────────────────
// Increment when the Config layout changes. The migration runner (Phase J)
// uses this to upgrade older blobs stored in NVS.
constexpr uint16_t CURRENT_SCHEMA_VERSION = 1;

// ── Config ────────────────────────────────────────────────────────────────────
// Single struct replacing ~80 V2.67 globals. Serialized as one CRC-protected
// NVS blob ("cfg_v1"). See architecture §5.2 for the canonical definition.
struct Config {
  uint16_t schema_version;          // must be first — migration runner reads at offset 0

  // ── Hardware (board + pin map) ──────────────────────────────────────────────
  enum class BoardPreset : uint8_t { Waveshare = 0, LilyGo = 1, Custom = 2 };
  BoardPreset board_preset;
  struct PinMap {
    int8_t rs485_tx;
    int8_t rs485_rx;
    int8_t rs485_dir;    // -1 = hardware auto-direction (LilyGo)
    int8_t can_tx;
    int8_t can_rx;
    int8_t led;
  } pins;

  // ── Battery topology ────────────────────────────────────────────────────────
  uint8_t bms_count;              // 1..16
  uint8_t force_cell_count;       // 0 = auto-detect from BMS, else override

  // ── Inverter / CAN protocol ─────────────────────────────────────────────────
  enum class CanProtocol : uint8_t { Victron = 0, Pylontech = 1, SMA = 2 };
  CanProtocol can_protocol;
  bool can_enabled;

  // ── Charge / discharge limits ───────────────────────────────────────────────
  float charge_amps_per_pack;
  float discharge_amps_per_pack;
  float cvl_voltage;              // base CVL; may be lowered at runtime by sysparam

  // ── Safety cutoffs ──────────────────────────────────────────────────────────
  float safe_pack_volt;
  float safe_cell_volt;
  float safe_cell_drift;

  // ── Spike filter ────────────────────────────────────────────────────────────
  float spike_volt_max;
  float spike_curr_max;
  uint8_t spike_soc_max;

  // ── Temperature limits ──────────────────────────────────────────────────────
  float charge_temp_min;
  float charge_temp_max;
  float discharge_temp_min;
  float discharge_temp_max;
  float temp_soft_zone;
  enum class TempMode : uint8_t { Hottest = 0, Average = 1 };
  TempMode temp_mode;

  // ── SOC source selection ────────────────────────────────────────────────────
  enum class SocMode : uint8_t { Calculated = 0, RawBms = 1, Hybrid = 2 };
  SocMode soc_mode;

  // ── Setup mode ──────────────────────────────────────────────────────────────
  // Replaces V2.67 expert/easy/setupd/auto_applied flags (see architecture §5.2).
  enum class SetupMode : uint8_t {
    Wizard      = 0,   // first-run wizard until completed
    AutoFromBms = 1,   // one-shot apply BMS sysparam thresholds at startup
    Manual      = 2    // user has full control, no auto-apply
  };
  SetupMode setup_mode;
  bool auto_from_bms_applied;     // sticky: don't re-apply on next boot

  // ── Maintenance charge ──────────────────────────────────────────────────────
  bool maint_charge_enabled;
  float maint_target_voltage;

  // ── Auto-balancer ───────────────────────────────────────────────────────────
  bool auto_balance_enabled;
  uint32_t auto_balance_last_ts;

  // ── Network ─────────────────────────────────────────────────────────────────
  char wifi_ssid[33];
  // wifi_password is NOT stored here — managed by esp_wifi provisioning
  char ntp_server[64];
  int8_t timezone_offset_h;

  // ── MQTT ────────────────────────────────────────────────────────────────────
  bool mqtt_enabled;
  char mqtt_host[64];
  uint16_t mqtt_port;
  char mqtt_user[32];
  char mqtt_pass_obf[64];          // obfuscated, see architecture §8.7
  char mqtt_base_topic[64];
  // Off=0, StatusOnly=1, DataSystem=2, PerPack=3 (NEW), PerCell=4 (shifted from 3).
  // NVS migration: stored value 3 (old PerCell) is upgraded to 4 on first load.
  enum class MqttLevel : uint8_t {
    Off        = 0,
    StatusOnly = 1,
    DataSystem = 2,
    PerPack    = 3,
    PerCell    = 4,
  };
  MqttLevel mqtt_level;
  bool mqtt_diag_enabled;
  bool ha_discovery_enabled;
  bool mqtt_full_publish;

  // ── Auth ────────────────────────────────────────────────────────────────────
  bool auth_enabled;
  char auth_user[32];
  char auth_hash[65];              // SHA-256 hex (64 chars + null), never plaintext

  // ── UI preferences ──────────────────────────────────────────────────────────
  uint8_t theme_id;                // 0=light, 1=dark, 2=auto
  uint8_t chart_series_a;          // 0..3 = power/voltage/soc/temp
  uint8_t chart_series_b;

  // ── UI polling cadences (ms) ────────────────────────────────────────────────
  uint16_t ui_poll_live_ms;        // default 1500, range 500-60000
  uint16_t ui_poll_diag_ms;        // default 5000
  uint16_t ui_poll_alerts_ms;      // default 30000

  // ── Counter reset window ────────────────────────────────────────────────────
  uint32_t last_reset_ts;          // 0 = unset; stamped on first NTP sync

  // ── Debug ───────────────────────────────────────────────────────────────────
  bool serial_debug_enabled;
  bool spy_persist_default;
};

// ── Default config ─────────────────────────────────────────────────────────────
// Waveshare pin map, V2.67-equivalent thresholds (see architecture §8.6 mapping).
extern const Config DEFAULT_CONFIG;

// ── Validation ────────────────────────────────────────────────────────────────
enum class ValidationError : uint8_t {
  None = 0,
  BmsCountOutOfRange,
  CellVoltageOutOfRange,
  PackVoltageOutOfRange,
  TemperatureOutOfRange,
  PinConflict,
  StringNotTerminated,
};

namespace storage {
  // Serializes cfg into buf (deterministic byte layout; schema_version at offset 0).
  // Returns false if buf_size < sizeof(Config).
  bool serialize(const Config& cfg, uint8_t* buf, size_t buf_size, size_t& out_len);

  // Reverses serialize. Validates schema_version == CURRENT_SCHEMA_VERSION.
  // Returns false on version mismatch or truncation.
  bool deserialize(const uint8_t* buf, size_t len, Config& out);

  // CRC-32/IEEE (poly 0xEDB88320). Standard test vector: crc32("123456789") == 0xCBF43926.
  uint32_t crc32(const uint8_t* data, size_t len);

  // Applies validation rules from architecture §8.6. On failure, writes the
  // offending field name into field_out (null-terminated, truncated to field_buf).
  ValidationError validate(const Config& cfg, char* field_out, size_t field_buf);
}
