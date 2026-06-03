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

TEST_CASE("Config struct is 692 bytes (v5 layout unchanged from v4)", "[config]") {
  // v3: 644 B.  v4 adds char[32]+uint32+uint32+uint16+uint16+bool → 692 B.
  // v5 adds uint16_t notify_debounce_s into former 3-byte tail padding → still 692 B.
  // This test mirrors the static_assert in config.cpp and catches ABI drift.
  REQUIRE(sizeof(Config) == 692);
  REQUIRE(CURRENT_SCHEMA_VERSION == 5);
}

// ── v3 → v4 migration tests ────────────────────────────────────────────────────

// Config_v3 layout mirror (must stay in sync with the frozen struct in config.cpp).
// Used only for building synthetic v3 blobs in tests.
namespace {
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
  v3.soc_mode       = Config::SocMode::Hybrid;
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
// Config_v4 is identical to Config_v5 in size (692 B) — the v5 field uses former
// tail padding. We build a synthetic v4 blob by constructing a v5 Config, zeroing
// the debounce field bytes (tail padding in v4), and tagging schema_version = 4.

TEST_CASE("Migration v4->v5: debounce_s gets default 30 from v4 blob", "[config][migrate]") {
  // Build a v5 Config with known v4 fields; schema_version set to 4.
  Config src = DEFAULT_CONFIG;
  src.schema_version         = 4;   // pretend to be a v4 blob
  src.notify_debounce_s      = 0;   // simulates v4 tail padding (zero)
  src.bms_count              = 7;
  src.notify_poll_interval_s = 90;
  src.notify_cooldown_s      = 180;
  src.notify_telegram_enabled = true;
  strncpy(src.notify_sender_name, "GW-test", sizeof(src.notify_sender_name) - 1);

  // Serialize the blob with schema_version=4.
  uint8_t buf[sizeof(Config) + 64];
  size_t len = 0;
  REQUIRE(storage::serialize(src, buf, sizeof(buf), len));

  // Deserialize: should trigger v4->v5 migration.
  Config out{};
  bool ok = storage::deserialize(buf, len, out);
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
