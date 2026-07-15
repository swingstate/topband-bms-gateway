#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>
#include <cstring>
#include "storage/config.h"

// ── Helpers ───────────────────────────────────────────────────────────────────

static bool round_trip(const Config& src, Config& dst) {
  uint8_t buf[sizeof(Config) + 64];
  size_t  len = 0;
  if (!storage::serialize(src, buf, sizeof(buf), len)) return false;
  return storage::deserialize(buf, len, dst);
}

// ── Test cases ────────────────────────────────────────────────────────────────

TEST_CASE("Round-trip: DEFAULT_CONFIG serializes and deserializes identically", "[config]") {
  Config out{};
  bool ok = round_trip(DEFAULT_CONFIG, out);

  REQUIRE(ok);
  // Byte-for-byte identity (memcmp covers every field including padding)
  REQUIRE(memcmp(&DEFAULT_CONFIG, &out, sizeof(Config)) == 0);
  REQUIRE(out.schema_version == CURRENT_SCHEMA_VERSION);
}

TEST_CASE("Schema version mismatch: corrupted version byte rejected", "[config]") {
  uint8_t buf[sizeof(Config)];
  size_t len = 0;
  REQUIRE(storage::serialize(DEFAULT_CONFIG, buf, sizeof(buf), len));

  // Corrupt schema_version at offset 0
  buf[0] = 0xFF;
  buf[1] = 0xFF;

  Config out{};
  bool ok = storage::deserialize(buf, len, out);
  REQUIRE_FALSE(ok);
}

TEST_CASE("Truncated buffer: deserialize rejects short inputs", "[config]") {
  uint8_t buf[sizeof(Config)];
  size_t len = 0;
  REQUIRE(storage::serialize(DEFAULT_CONFIG, buf, sizeof(buf), len));

  Config out{};
  // Pass half the actual length
  bool ok = storage::deserialize(buf, len / 2, out);
  REQUIRE_FALSE(ok);
}

TEST_CASE("Validation: DEFAULT_CONFIG passes all rules", "[config]") {
  char field[64] = {};
  auto err = storage::validate(DEFAULT_CONFIG, field, sizeof(field));
  REQUIRE(err == ValidationError::None);
  REQUIRE(field[0] == '\0');   // field name should not be written on success
}

TEST_CASE("Validation fail — bms_count out of range", "[config]") {
  Config bad = DEFAULT_CONFIG;
  bad.bms_count = 17;   // max is 16

  char field[64] = {};
  auto err = storage::validate(bad, field, sizeof(field));
  REQUIRE(err == ValidationError::BmsCountOutOfRange);
  REQUIRE(std::string(field) == "bms_count");
}

TEST_CASE("Validation fail — cell voltage out of range", "[config]") {
  Config bad = DEFAULT_CONFIG;
  bad.safe_cell_volt = 5.0f;   // above 4.5 V plausibility ceiling

  char field[64] = {};
  auto err = storage::validate(bad, field, sizeof(field));
  REQUIRE(err == ValidationError::CellVoltageOutOfRange);
  REQUIRE(std::string(field) == "safe_cell_volt");
}

TEST_CASE("Validation fail — temperature out of range", "[config]") {
  Config bad = DEFAULT_CONFIG;
  bad.charge_temp_max = 200.0f;   // above 100 °C plausibility ceiling

  char field[64] = {};
  auto err = storage::validate(bad, field, sizeof(field));
  REQUIRE(err == ValidationError::TemperatureOutOfRange);
  REQUIRE(std::string(field) == "charge_temp_max");
}

TEST_CASE("Validation fail — pin conflict", "[config]") {
  Config bad = DEFAULT_CONFIG;
  // Force rs485_tx to the same GPIO as can_tx
  bad.pins.rs485_tx = bad.pins.can_tx;

  char field[64] = {};
  auto err = storage::validate(bad, field, sizeof(field));
  REQUIRE(err == ValidationError::PinConflict);
  REQUIRE(std::string(field) == "pins");
}

TEST_CASE("CRC-32 known-answer test: '123456789' -> 0xCBF43926", "[crc]") {
  const uint8_t input[] = {'1','2','3','4','5','6','7','8','9'};
  uint32_t crc = storage::crc32(input, sizeof(input));
  REQUIRE(crc == 0xCBF43926u);
}

// ── P2 notification-field round-trip tests ────────────────────────────────────

TEST_CASE("Round-trip: notify_telegram_token survives serialize/deserialize", "[config][notify]") {
  Config src = DEFAULT_CONFIG;
  src.notify_telegram_enabled = true;
  // Typical Telegram bot token format: bot<id>:<hash>, usually ~46 chars.
  strncpy(src.notify_telegram_token,
          "bot1234567890:ABCDEFGHIJKLMNOPQRSTUVWXYZabc",
          sizeof(src.notify_telegram_token) - 1);
  strncpy(src.notify_telegram_chat_id, "987654321",
          sizeof(src.notify_telegram_chat_id) - 1);

  Config dst{};
  bool ok = round_trip(src, dst);

  REQUIRE(ok);
  REQUIRE(dst.notify_telegram_enabled == true);
  REQUIRE(std::string(dst.notify_telegram_token) == "bot1234567890:ABCDEFGHIJKLMNOPQRSTUVWXYZabc");
  REQUIRE(std::string(dst.notify_telegram_chat_id) == "987654321");
}

TEST_CASE("Round-trip: empty notify fields survive serialize/deserialize", "[config][notify]") {
  // DEFAULT_CONFIG has notify disabled and empty token/chat_id.
  Config dst{};
  bool ok = round_trip(DEFAULT_CONFIG, dst);

  REQUIRE(ok);
  REQUIRE(dst.notify_telegram_enabled == false);
  REQUIRE(dst.notify_telegram_token[0] == '\0');
  REQUIRE(dst.notify_telegram_chat_id[0] == '\0');
}

TEST_CASE("Config struct is 880 bytes (v11 layout)", "[config]") {
  // Stale expectation from the v5 era (692 B / schema 5) — the struct has
  // grown through v6..v11 since (see config.h schema-version comment and the
  // static_assert in config.cpp for the full per-version derivation).
  // This test mirrors that static_assert and catches ABI drift.
  REQUIRE(sizeof(Config) == 880);
  REQUIRE(CURRENT_SCHEMA_VERSION == 11);
}

// ── v3 → v4 migration tests ────────────────────────────────────────────────────

// Config_v3 layout mirror (must stay in sync with the frozen struct in config.cpp).
// Used only for building synthetic v3 blobs in tests.
// TestSocMode mirrors the pre-v11 Config::SocMode (removed; see config.cpp
// LegacySocMode) purely for byte-layout parity — same uint8_t single-byte enum.
namespace {
enum class TestSocMode : uint8_t { Calculated = 0, RawBms = 1, Hybrid = 2 };
struct TestConfig_v3 {
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
  TestSocMode              soc_mode;
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
static_assert(sizeof(TestConfig_v3) == 644, "TestConfig_v3 size mismatch");
}  // namespace

TEST_CASE("Migration v3->v4: all v3 settings survive, v4 fields get safe defaults", "[config][migrate]") {
  TestConfig_v3 v3{};
  v3.schema_version = 3;
  v3.board_preset   = Config::BoardPreset::Waveshare;
  v3.bms_count      = 5;
  v3.rs485_enabled  = true;
  v3.can_enabled    = true;
  v3.can_protocol   = Config::CanProtocol::Pylontech;
  v3.charge_amps_per_pack    = 25.0f;
  v3.discharge_amps_per_pack = 35.0f;
  v3.cvl_voltage    = 51.0f;
  v3.safe_pack_volt = 52.0f;
  v3.safe_cell_volt = 3.50f;
  v3.safe_cell_drift = 0.15f;
  v3.charge_temp_min = 5.0f; v3.charge_temp_max = 45.0f;
  v3.discharge_temp_min = -10.0f; v3.discharge_temp_max = 55.0f;
  v3.temp_soft_zone = 4.0f;
  v3.temp_mode      = Config::TempMode::Average;
  v3.soc_mode       = TestSocMode::Hybrid;
  v3.setup_mode     = Config::SetupMode::Manual;
  strncpy(v3.wifi_ssid,       "MyNet",   sizeof(v3.wifi_ssid) - 1);
  strncpy(v3.ntp_server,      "time.cloudflare.com", sizeof(v3.ntp_server) - 1);
  v3.timezone_offset_h = 2;
  v3.mqtt_enabled   = true;
  strncpy(v3.mqtt_host,       "broker.local", sizeof(v3.mqtt_host) - 1);
  v3.mqtt_port      = 8883;
  strncpy(v3.mqtt_base_topic, "home/bms", sizeof(v3.mqtt_base_topic) - 1);
  v3.mqtt_level     = Config::MqttLevel::PerCell;
  v3.notify_telegram_enabled = true;
  strncpy(v3.notify_telegram_token,   "bot999:TOKENXYZ", sizeof(v3.notify_telegram_token) - 1);
  strncpy(v3.notify_telegram_chat_id, "55667788",        sizeof(v3.notify_telegram_chat_id) - 1);
  v3.auth_enabled   = false;
  strncpy(v3.auth_user, "admin", sizeof(v3.auth_user) - 1);
  v3.theme_id       = 1;
  v3.ui_poll_live_ms = 2000; v3.ui_poll_diag_ms = 8000; v3.ui_poll_alerts_ms = 15000;

  Config out{};
  bool ok = storage::deserialize(reinterpret_cast<const uint8_t*>(&v3), sizeof(v3), out);
  REQUIRE(ok);

  // Schema bumped to current.
  REQUIRE(out.schema_version == CURRENT_SCHEMA_VERSION);

  // All v3 fields preserved.
  REQUIRE(out.bms_count == 5);
  REQUIRE(out.rs485_enabled == true);
  REQUIRE(out.can_protocol == Config::CanProtocol::Pylontech);
  REQUIRE(out.charge_amps_per_pack == Catch::Approx(25.0f));
  REQUIRE(out.discharge_amps_per_pack == Catch::Approx(35.0f));
  REQUIRE(out.cvl_voltage == Catch::Approx(51.0f));
  REQUIRE(out.timezone_offset_h == 2);
  REQUIRE(out.mqtt_enabled == true);
  REQUIRE(out.mqtt_port == 8883);
  REQUIRE(std::string(out.mqtt_base_topic) == "home/bms");
  REQUIRE(out.mqtt_level == Config::MqttLevel::PerCell);
  REQUIRE(out.notify_telegram_enabled == true);
  REQUIRE(std::string(out.notify_telegram_token)   == "bot999:TOKENXYZ");
  REQUIRE(std::string(out.notify_telegram_chat_id) == "55667788");
  REQUIRE(out.theme_id == 1);
  REQUIRE(out.ui_poll_live_ms == 2000);

  // v4 additions: safe defaults.
  REQUIRE(out.notify_sender_name[0] == '\0');
  REQUIRE(out.notify_poll_interval_s == 60u);
  REQUIRE(out.notify_cooldown_s      == 120u);
  REQUIRE(out.notify_telegram_verified == false);
  REQUIRE(out.notify_telegram_last_ok_ts == 0u);
  // Default alert flags should enable the safety-critical set.
  REQUIRE(out.notify_alert_flags != 0u);
  REQUIRE((out.notify_alert_flags & (1u << 7)) != 0u);  // PackUndervoltStart
  REQUIRE((out.notify_alert_flags & (1u << 3)) != 0u);  // PackOvervoltStart
  REQUIRE((out.notify_alert_flags & (1u << 16)) != 0u); // NoPacksOnline
}

TEST_CASE("Migration v3->v4: round-trip preserves v4 fields after initial set", "[config][migrate]") {
  // Verify that v4 fields set in a v4 Config survive a full round-trip.
  Config src = DEFAULT_CONFIG;
  src.notify_poll_interval_s = 180;
  src.notify_cooldown_s      = 300;
  src.notify_telegram_verified = true;
  src.notify_telegram_last_ok_ts = 1700000000u;
  src.notify_alert_flags = (1u << 3) | (1u << 7);
  strncpy(src.notify_sender_name, "GW-kitchen", sizeof(src.notify_sender_name) - 1);

  Config dst{};
  bool ok = round_trip(src, dst);
  REQUIRE(ok);
  REQUIRE(dst.notify_poll_interval_s == 180u);
  REQUIRE(dst.notify_cooldown_s      == 300u);
  REQUIRE(dst.notify_telegram_verified == true);
  REQUIRE(dst.notify_telegram_last_ok_ts == 1700000000u);
  REQUIRE(dst.notify_alert_flags == ((1u << 3) | (1u << 7)));
  REQUIRE(std::string(dst.notify_sender_name) == "GW-kitchen");
}


TEST_CASE("Round-trip: v4 notify_alert_flags all-bits survives", "[config][notify]") {
  Config src = DEFAULT_CONFIG;
  src.notify_alert_flags = 0xFFFFFFFFu;

  Config dst{};
  bool ok = round_trip(src, dst);
  REQUIRE(ok);
  REQUIRE(dst.notify_alert_flags == 0xFFFFFFFFu);
}

// ── v4 → v5 migration tests ────────────────────────────────────────────────────
// Config_v4 layout mirror (must stay in sync with the frozen struct in
// config.cpp) — same rationale as TestConfig_v3 above: the live Config's
// prefix is NOT guaranteed to match any historical schema's layout (fields
// can be removed/reordered mid-struct, not just appended), so synthetic
// blobs must be built from a real byte-layout mirror, never from a
// live-Config instance with a relabelled schema_version.
namespace {
struct TestConfig_v4 {
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
  TestSocMode              soc_mode;
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
static_assert(sizeof(TestConfig_v4) == 692, "TestConfig_v4 size mismatch");
}  // namespace

TEST_CASE("Migration v4->v5: debounce_s gets default 30 from v4 blob", "[config][migrate]") {
  // Build a genuine v4-shaped blob (not a live Config with a relabelled
  // schema_version — see TestConfig_v4 comment above for why that's unsafe).
  TestConfig_v4 v4{};
  v4.schema_version         = 4;
  v4.board_preset           = Config::BoardPreset::Waveshare;
  v4.can_protocol           = Config::CanProtocol::Victron;
  v4.bms_count              = 7;
  v4.notify_poll_interval_s = 90;
  v4.notify_cooldown_s      = 180;
  v4.notify_telegram_enabled = true;
  strncpy(v4.notify_sender_name, "GW-test", sizeof(v4.notify_sender_name) - 1);
  // notify_debounce_s doesn't exist in v4 — its bytes are the 3-byte tail
  // padding, already zeroed by the `TestConfig_v4 v4{};` value-init above.

  Config out{};
  bool ok = storage::deserialize(reinterpret_cast<const uint8_t*>(&v4), sizeof(v4), out);
  REQUIRE(ok);

  // Schema bumped to v5.
  REQUIRE(out.schema_version == CURRENT_SCHEMA_VERSION);

  // All v4 fields preserved.
  REQUIRE(out.bms_count == 7);
  REQUIRE(out.notify_poll_interval_s == 90u);
  REQUIRE(out.notify_cooldown_s      == 180u);
  REQUIRE(out.notify_telegram_enabled == true);
  REQUIRE(std::string(out.notify_sender_name) == "GW-test");

  // notify_debounce_s should be the safe default (30), NOT 0 from the v4 zeros.
  REQUIRE(out.notify_debounce_s == 30u);
}

TEST_CASE("Round-trip: v5 notify_debounce_s survives serialize/deserialize", "[config][migrate]") {
  Config src = DEFAULT_CONFIG;
  src.notify_debounce_s = 45u;

  Config dst{};
  bool ok = round_trip(src, dst);
  REQUIRE(ok);
  REQUIRE(dst.notify_debounce_s == 45u);
}

TEST_CASE("Round-trip: notify_debounce_s zero (disabled) survives", "[config][migrate]") {
  Config src = DEFAULT_CONFIG;
  src.notify_debounce_s = 0u;

  Config dst{};
  bool ok = round_trip(src, dst);
  REQUIRE(ok);
  REQUIRE(dst.notify_debounce_s == 0u);
}

// ── v10 → v11 migration tests ──────────────────────────────────────────────────
// v11 removes Config::SocMode/ShuntCurrentMode (soc_mode/shunt_current_mode)
// in favour of BatterySourcePolicy + per-metric MetricSource fields. These
// tests build genuine v10-shaped blobs (see TestConfig_v10 comment) to verify
// existing settings survive and the new fields are derived correctly — this
// is the "config schema changes must preserve existing settings on upgrade"
// guarantee for the V3.2 Battery Value Sources consolidation.

namespace {
// Mirrors LegacyShuntCurrentMode in config.cpp — same rationale as TestSocMode.
enum class TestShuntCurrentMode : uint8_t { Auto = 0, ShuntLeads = 1, BmsLeads = 2 };

// Config_v10 layout mirror (must stay in sync with the frozen struct in config.cpp).
struct TestConfig_v10 {
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
  TestSocMode              soc_mode;
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
  uint16_t                 notify_debounce_s;
  Config::BatteryConfigMode battery_config_mode;
  bool                     ble_shunt_enabled;
  bool                     ble_mppt_enabled;
  char                     ble_shunt_mac[18];
  char                     ble_mppt_mac[18];
  char                     ble_shunt_key[33];
  char                     ble_mppt_key[33];
  char                     wifi_bssid[18];
  int8_t                   wifi_rssi_threshold;
  char                     mqtt_solar_passthrough_topic[64];
  TestShuntCurrentMode     shunt_current_mode;
};
static_assert(sizeof(TestConfig_v10) == 884, "TestConfig_v10 size mismatch");

// Builds a valid, fully-populated v10 blob with sensible non-default field
// values in a few spots so the "existing settings survive" assertions below
// are meaningful (not just checking untouched zero-init memory).
//
// MQTT and BLE/MPPT fields are populated here (not just the SocMode/
// ShuntCurrentMode-adjacent fields the original version of this helper set)
// because a real V3.2 field regression showed MQTT and MPPT settings being
// lost across a v10->v11 upgrade on real hardware, and the original
// synthetic helper left both blank/default — a config.cpp-level test built
// on it could never have caught that class of bug. See
// "Migration v10->v11: realistic full config" below and
// test/host/test_nvs_store.cpp (which reproduces the actual root cause: a
// read-buffer sizing bug in nvs_store.cpp, not a config.cpp migration bug).
TestConfig_v10 make_v10_blob() {
  TestConfig_v10 v10{};
  v10.schema_version = 10;
  v10.board_preset    = Config::BoardPreset::Waveshare;
  v10.can_protocol    = Config::CanProtocol::Victron;
  v10.can_enabled     = true;
  v10.bms_count       = 6;
  v10.cvl_voltage     = 52.5f;
  v10.setup_mode      = Config::SetupMode::Manual;
  strncpy(v10.wifi_ssid, "HomeNet", sizeof(v10.wifi_ssid) - 1);
  strncpy(v10.wifi_bssid, "aa:11:bb:22:cc:33", sizeof(v10.wifi_bssid) - 1);
  v10.wifi_rssi_threshold = -75;

  v10.mqtt_enabled = true;
  strncpy(v10.mqtt_host, "192.168.1.50", sizeof(v10.mqtt_host) - 1);
  v10.mqtt_port = 1883;
  strncpy(v10.mqtt_user, "gateway_user", sizeof(v10.mqtt_user) - 1);
  strncpy(v10.mqtt_pass_obf, "obf:s3cr3t-pass", sizeof(v10.mqtt_pass_obf) - 1);
  strncpy(v10.mqtt_base_topic, "topband/gw1", sizeof(v10.mqtt_base_topic) - 1);
  v10.mqtt_level           = Config::MqttLevel::PerCell;
  v10.mqtt_diag_enabled    = true;
  v10.ha_discovery_enabled = true;
  v10.mqtt_full_publish    = true;
  strncpy(v10.mqtt_solar_passthrough_topic, "opendtu/solar/state",
          sizeof(v10.mqtt_solar_passthrough_topic) - 1);

  v10.ble_shunt_enabled = true;
  v10.ble_mppt_enabled  = true;
  strncpy(v10.ble_shunt_mac, "aa:bb:cc:dd:ee:ff", sizeof(v10.ble_shunt_mac) - 1);
  strncpy(v10.ble_mppt_mac,  "11:22:33:44:55:66", sizeof(v10.ble_mppt_mac) - 1);
  strncpy(v10.ble_shunt_key, "0123456789abcdef0123456789abcdef", sizeof(v10.ble_shunt_key) - 1);
  strncpy(v10.ble_mppt_key,  "fedcba9876543210fedcba9876543210", sizeof(v10.ble_mppt_key) - 1);

  v10.notify_telegram_enabled = true;
  strncpy(v10.notify_telegram_token, "123456:ABC-DEF-token", sizeof(v10.notify_telegram_token) - 1);
  strncpy(v10.notify_telegram_chat_id, "-100987654", sizeof(v10.notify_telegram_chat_id) - 1);

  v10.auth_enabled = true;
  strncpy(v10.auth_user, "admin", sizeof(v10.auth_user) - 1);

  return v10;
}
}  // namespace

TEST_CASE("Migration v10->v11: defaults (Calculated+Auto) -> Auto policy", "[config][migrate][v11]") {
  TestConfig_v10 v10 = make_v10_blob();
  v10.soc_mode            = TestSocMode::Calculated;
  v10.shunt_current_mode  = TestShuntCurrentMode::Auto;

  Config out{};
  bool ok = storage::deserialize(reinterpret_cast<const uint8_t*>(&v10), sizeof(v10), out);
  REQUIRE(ok);
  REQUIRE(out.schema_version == CURRENT_SCHEMA_VERSION);

  // Pre-existing v10 settings preserved.
  REQUIRE(out.bms_count == 6);
  REQUIRE(out.cvl_voltage == Catch::Approx(52.5f));
  REQUIRE(std::string(out.wifi_ssid) == "HomeNet");
  REQUIRE(out.ble_shunt_enabled == true);
  REQUIRE(std::string(out.ble_shunt_mac) == "aa:bb:cc:dd:ee:ff");

  // Both old fields at their defaults -> new Auto policy (uniform shunt-leads-
  // when-fresh is a superset of the old Calculated+Auto default behaviour).
  REQUIRE(out.battery_source_policy == Config::BatterySourcePolicy::Auto);
}

TEST_CASE("Migration v10->v11: RawBms + BmsLeads -> Manual, all metrics on BMS",
          "[config][migrate][v11]") {
  TestConfig_v10 v10 = make_v10_blob();
  v10.soc_mode           = TestSocMode::RawBms;
  v10.shunt_current_mode = TestShuntCurrentMode::BmsLeads;

  Config out{};
  bool ok = storage::deserialize(reinterpret_cast<const uint8_t*>(&v10), sizeof(v10), out);
  REQUIRE(ok);

  REQUIRE(out.battery_source_policy == Config::BatterySourcePolicy::Manual);
  REQUIRE(out.voltage_source == Config::MetricSource::Bms);
  REQUIRE(out.current_source == Config::MetricSource::Bms);
  REQUIRE(out.soc_source      == Config::MetricSource::Bms);
}

TEST_CASE("Migration v10->v11: Calculated + ShuntLeads -> Manual, current+soc on Shunt",
          "[config][migrate][v11]") {
  TestConfig_v10 v10 = make_v10_blob();
  v10.soc_mode           = TestSocMode::Calculated;  // not RawBms -> soc "was default"
  v10.shunt_current_mode = TestShuntCurrentMode::ShuntLeads;  // explicit non-default choice

  Config out{};
  bool ok = storage::deserialize(reinterpret_cast<const uint8_t*>(&v10), sizeof(v10), out);
  REQUIRE(ok);

  // Any explicit non-default choice on EITHER old field flips the whole
  // policy to Manual (Auto is uniform across all three metrics or not at all).
  REQUIRE(out.battery_source_policy == Config::BatterySourcePolicy::Manual);
  REQUIRE(out.voltage_source == Config::MetricSource::Bms);  // no historical equivalent
  REQUIRE(out.current_source == Config::MetricSource::Shunt);
  REQUIRE(out.soc_source      == Config::MetricSource::Shunt);
}

// Regression test for a real V3.2 field report: MQTT and MPPT settings were
// lost upgrading 3.1.0-preview.4 (schema v10) -> 3.2.0-dev.8 (schema v11) on
// real hardware. migrate_v10_to_v11() itself turned out to copy every v10
// field correctly (verified by this test); the actual defect was in
// nvs_store.cpp's read-buffer sizing (fixed separately, see
// test/host/test_nvs_store.cpp), which never reached migrate_v10_to_v11() at
// all. This test still earns its place: the pre-existing v10->v11 tests
// above only ever populated/asserted the SocMode/ShuntCurrentMode-adjacent
// fields, so a REAL migrate_v10_to_v11() regression dropping e.g. mqtt_host
// would have passed them silently. Assert every non-trivial v10 field
// (not just the ones the SocMode/ShuntCurrentMode change touched) survives.
TEST_CASE("Migration v10->v11: realistic full config (MQTT+MPPT populated) - "
          "all fields round-trip, no data loss", "[config][migrate][v11]") {
  TestConfig_v10 v10 = make_v10_blob();
  v10.soc_mode           = TestSocMode::Calculated;
  v10.shunt_current_mode = TestShuntCurrentMode::Auto;

  Config out{};
  bool ok = storage::deserialize(reinterpret_cast<const uint8_t*>(&v10), sizeof(v10), out);
  REQUIRE(ok);
  REQUIRE(out.schema_version == CURRENT_SCHEMA_VERSION);

  // Network / WiFi
  REQUIRE(std::string(out.wifi_ssid) == "HomeNet");
  REQUIRE(std::string(out.wifi_bssid) == "aa:11:bb:22:cc:33");
  REQUIRE(out.wifi_rssi_threshold == -75);

  // MQTT — the field the owner reported losing.
  REQUIRE(out.mqtt_enabled == true);
  REQUIRE(std::string(out.mqtt_host) == "192.168.1.50");
  REQUIRE(out.mqtt_port == 1883);
  REQUIRE(std::string(out.mqtt_user) == "gateway_user");
  REQUIRE(std::string(out.mqtt_pass_obf) == "obf:s3cr3t-pass");
  REQUIRE(std::string(out.mqtt_base_topic) == "topband/gw1");
  REQUIRE(out.mqtt_level == Config::MqttLevel::PerCell);
  REQUIRE(out.mqtt_diag_enabled == true);
  REQUIRE(out.ha_discovery_enabled == true);
  REQUIRE(out.mqtt_full_publish == true);
  REQUIRE(std::string(out.mqtt_solar_passthrough_topic) == "opendtu/solar/state");

  // BLE / MPPT — the other field the owner reported losing.
  REQUIRE(out.ble_shunt_enabled == true);
  REQUIRE(out.ble_mppt_enabled == true);
  REQUIRE(std::string(out.ble_shunt_mac) == "aa:bb:cc:dd:ee:ff");
  REQUIRE(std::string(out.ble_mppt_mac) == "11:22:33:44:55:66");
  REQUIRE(std::string(out.ble_shunt_key) == "0123456789abcdef0123456789abcdef");
  REQUIRE(std::string(out.ble_mppt_key) == "fedcba9876543210fedcba9876543210");

  // Telegram notifications and web auth — also at risk, per the same class
  // of bug; assert they survive too.
  REQUIRE(out.notify_telegram_enabled == true);
  REQUIRE(std::string(out.notify_telegram_token) == "123456:ABC-DEF-token");
  REQUIRE(std::string(out.notify_telegram_chat_id) == "-100987654");
  REQUIRE(out.auth_enabled == true);
  REQUIRE(std::string(out.auth_user) == "admin");
}
