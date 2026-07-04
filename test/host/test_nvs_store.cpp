// Regression coverage for the V3.2 config-migration data-loss bug: upgrading
// 3.1.0-preview.4 (schema v10, Config_v10 = 884 B) -> 3.2.0-dev.8 (schema
// v11, Config = 880 B) silently reset MQTT and MPPT/BLE settings to
// defaults on real hardware, despite the dedicated migrate_v10_to_v11()
// tests in test_config_serde.cpp passing.
//
// Root cause: storage::loadConfig() (src/storage/nvs_store.cpp) read the
// stored blob into a buffer sized off sizeof(Config) — the CURRENT,
// running-firmware struct size (880 B) — instead of the size the blob was
// actually stored at. v11 is the first schema change to ever SHRINK the
// struct (a mid-struct field removal closed an alignment gap), so an
// upgrading device's still-v10 blob (884 B) no longer fit the read buffer.
// nvs_get_blob() rejected the undersized buffer, loadConfig() treated that
// as "not found", and the whole config — not just MQTT/MPPT, everything —
// silently fell back to DEFAULT_CONFIG.
//
// This was invisible to test_config_serde.cpp because config.cpp's
// deserialize()/migrate_v10_to_v11() are pure functions the test calls
// directly with a correctly-sized buffer it constructs itself — the bug
// lived entirely in the NVS read path those tests never touch. This file
// compiles the REAL nvs_store.cpp against a fake in-memory NVS backing
// store (stubs/esp_idf/fake_nvs.cpp) so the actual buggy code path is
// exercised end-to-end.
#include "storage/nvs_store.h"
#include "storage/config.h"
#include "fake_nvs_test_api.h"

#include <catch2/catch_test_macros.hpp>
#include <cstring>
#include <string>

namespace {

// Mirrors LegacySocMode / LegacyShuntCurrentMode in config.cpp (same
// rationale as the equivalent mirror in test_config_serde.cpp — the real
// enums are private to config.cpp's anonymous namespace).
enum class TestSocMode : uint8_t { Calculated = 0, RawBms = 1, Hybrid = 2 };
enum class TestShuntCurrentMode : uint8_t { Auto = 0, ShuntLeads = 1, BmsLeads = 2 };

// Config_v10 layout mirror (must stay in sync with the frozen struct in
// config.cpp / test_config_serde.cpp).
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
static_assert(sizeof(TestConfig_v10) > sizeof(Config),
    "This regression only reproduces while the historical v10 blob is "
    "LARGER than the current Config — if that stops being true the test "
    "still passes but no longer exercises the original bug");

// Builds a realistic pre-upgrade v10 blob representative of an actual
// 3.1.0-preview.4 install: MQTT and BLE/MPPT fully configured, matching
// what the owner had on real hardware when the regression was reported.
TestConfig_v10 make_realistic_v10_blob() {
  TestConfig_v10 v10{};
  v10.schema_version = 10;
  v10.board_preset    = Config::BoardPreset::Waveshare;
  v10.can_protocol    = Config::CanProtocol::Victron;
  v10.can_enabled     = true;
  v10.bms_count       = 6;
  v10.cvl_voltage     = 52.5f;
  v10.setup_mode      = Config::SetupMode::Manual;
  v10.soc_mode           = TestSocMode::Calculated;
  v10.shunt_current_mode = TestShuntCurrentMode::Auto;

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

  v10.ble_shunt_enabled = true;
  v10.ble_mppt_enabled  = true;
  strncpy(v10.ble_shunt_mac, "aa:bb:cc:dd:ee:ff", sizeof(v10.ble_shunt_mac) - 1);
  strncpy(v10.ble_mppt_mac,  "11:22:33:44:55:66", sizeof(v10.ble_mppt_mac) - 1);
  strncpy(v10.ble_shunt_key, "0123456789abcdef0123456789abcdef", sizeof(v10.ble_shunt_key) - 1);
  strncpy(v10.ble_mppt_key,  "fedcba9876543210fedcba9876543210", sizeof(v10.ble_mppt_key) - 1);

  return v10;
}

// Seeds the fake NVS with a v10 blob + matching CRC, exactly as saveConfig()
// would have written it back on the device's last boot under firmware
// 3.1.0-preview.4.
void seed_v10_blob(const TestConfig_v10& v10) {
  fake_nvs_reset();
  const auto* bytes = reinterpret_cast<const uint8_t*>(&v10);
  uint32_t crc = storage::crc32(bytes, sizeof(v10));
  fake_nvs_seed_blob("cfg_v1", &v10, sizeof(v10));
  fake_nvs_seed_u32("cfg_v1_crc", crc);
}

}  // namespace

TEST_CASE("loadConfig migrates a realistic oversized v10 blob without falling back to defaults "
          "(regression: nvs_store.cpp read-buffer sizing bug lost MQTT+BLE config)",
          "[storage][nvs][migrate][regression]") {
  TestConfig_v10 v10 = make_realistic_v10_blob();
  REQUIRE(sizeof(v10) > sizeof(Config));  // the exact condition that broke loadConfig()
  seed_v10_blob(v10);

  Config out{};
  bool ok = storage::loadConfig(out);

  REQUIRE(ok);
  REQUIRE(out.schema_version == CURRENT_SCHEMA_VERSION);

  // MQTT — the field the owner reported losing.
  REQUIRE(out.mqtt_enabled == true);
  REQUIRE(std::string(out.mqtt_host) == "192.168.1.50");
  REQUIRE(out.mqtt_port == 1883);
  REQUIRE(std::string(out.mqtt_user) == "gateway_user");
  REQUIRE(std::string(out.mqtt_pass_obf) == "obf:s3cr3t-pass");
  REQUIRE(std::string(out.mqtt_base_topic) == "topband/gw1");
  REQUIRE(out.mqtt_level == Config::MqttLevel::PerCell);
  REQUIRE(out.ha_discovery_enabled == true);

  // BLE / MPPT — the other field the owner reported losing.
  REQUIRE(out.ble_shunt_enabled == true);
  REQUIRE(out.ble_mppt_enabled == true);
  REQUIRE(std::string(out.ble_shunt_mac) == "aa:bb:cc:dd:ee:ff");
  REQUIRE(std::string(out.ble_mppt_mac) == "11:22:33:44:55:66");
  REQUIRE(std::string(out.ble_shunt_key) == "0123456789abcdef0123456789abcdef");
  REQUIRE(std::string(out.ble_mppt_key) == "fedcba9876543210fedcba9876543210");

  // Other settings, to catch any other field at risk from the same bug class.
  REQUIRE(out.bms_count == 6);
  REQUIRE(out.cvl_voltage == 52.5f);
  REQUIRE(std::string(out.wifi_ssid) == "HomeNet");
  REQUIRE(std::string(out.wifi_bssid) == "aa:11:bb:22:cc:33");
  REQUIRE(out.wifi_rssi_threshold == -75);
}

TEST_CASE("loadConfig persists the migrated blob so the next boot loads v11 directly",
          "[storage][nvs][migrate]") {
  TestConfig_v10 v10 = make_realistic_v10_blob();
  seed_v10_blob(v10);

  Config first{};
  REQUIRE(storage::loadConfig(first));

  // Second load should now see a v11-shaped blob already in NVS (loadConfig
  // re-persists on migration) and must still preserve the same settings.
  Config second{};
  REQUIRE(storage::loadConfig(second));
  REQUIRE(second.schema_version == CURRENT_SCHEMA_VERSION);
  REQUIRE(std::string(second.mqtt_host) == "192.168.1.50");
  REQUIRE(std::string(second.ble_mppt_mac) == "11:22:33:44:55:66");
}

TEST_CASE("loadConfig on empty NVS returns defaults", "[storage][nvs]") {
  fake_nvs_reset();
  Config out{};
  bool ok = storage::loadConfig(out);
  REQUIRE_FALSE(ok);
  REQUIRE(out.schema_version == CURRENT_SCHEMA_VERSION);
}

TEST_CASE("saveConfig then loadConfig round-trips DEFAULT_CONFIG exactly", "[storage][nvs]") {
  fake_nvs_reset();
  REQUIRE(storage::saveConfig(DEFAULT_CONFIG));

  Config out{};
  REQUIRE(storage::loadConfig(out));
  REQUIRE(std::memcmp(&out, &DEFAULT_CONFIG, sizeof(Config)) == 0);
}
