#include <catch2/catch_test_macros.hpp>
#include <ArduinoJson.h>
#include <cstring>
#include "storage/config.h"
#include "mqtt/config_backup.h"
#include "web/config_json.h"

// ── mqtt::config_backup::build_json ────────────────────────────────────────
//
// Reuses web::config_to_json() (verified separately in test_config_json.cpp)
// but must omit — not blank — WiFi SSID and MQTT broker host/port/user/pass,
// since those are needed just to reach this backup in the first place.
// Everything else must survive intact for the restore path to work.

TEST_CASE("config_backup::build_json excludes WiFi SSID and MQTT broker fields", "[mqtt][config_backup]") {
  Config cfg = DEFAULT_CONFIG;
  strncpy(cfg.wifi_ssid, "MyHomeWifi", sizeof(cfg.wifi_ssid) - 1);
  strncpy(cfg.mqtt_host, "192.168.1.50", sizeof(cfg.mqtt_host) - 1);
  cfg.mqtt_port = 8883;
  strncpy(cfg.mqtt_user, "mqttuser", sizeof(cfg.mqtt_user) - 1);
  strncpy(cfg.mqtt_pass_obf, "obfuscated-secret", sizeof(cfg.mqtt_pass_obf) - 1);
  strncpy(cfg.auth_user, "admin", sizeof(cfg.auth_user) - 1);

  JsonDocument doc;
  mqtt::config_backup::build_json(cfg, doc);

  REQUIRE_FALSE(doc["wifi_ssid"].is<const char*>());
  REQUIRE_FALSE(doc["mqtt_host"].is<const char*>());
  REQUIRE_FALSE(doc["mqtt_port"].is<int>());
  REQUIRE_FALSE(doc["mqtt_user"].is<const char*>());
  REQUIRE_FALSE(doc["mqtt_pass_obf"].is<const char*>());
  REQUIRE_FALSE(doc["auth_user"].is<const char*>());
}

TEST_CASE("config_backup::build_json keeps everything else, including MQTT detail settings", "[mqtt][config_backup]") {
  Config cfg = DEFAULT_CONFIG;
  cfg.mqtt_enabled          = true;
  strncpy(cfg.mqtt_base_topic, "topband-bms", sizeof(cfg.mqtt_base_topic) - 1);
  cfg.mqtt_diag_enabled     = true;
  cfg.ha_discovery_enabled  = true;
  cfg.wifi_rssi_threshold   = -70;

  JsonDocument doc;
  mqtt::config_backup::build_json(cfg, doc);

  REQUIRE(doc["mqtt_enabled"].as<bool>() == true);
  REQUIRE(std::string(doc["mqtt_base_topic"].as<const char*>()) == "topband-bms");
  REQUIRE(doc["mqtt_diag_enabled"].as<bool>() == true);
  REQUIRE(doc["ha_discovery_enabled"].as<bool>() == true);
  REQUIRE(doc["wifi_rssi_threshold"].as<int>() == -70);
  REQUIRE(doc["schema_version"].as<int>() == (int)CURRENT_SCHEMA_VERSION);
}

TEST_CASE("config_backup::build_json payload round-trips through json_to_config sans excluded fields", "[mqtt][config_backup]") {
  Config source = DEFAULT_CONFIG;
  strncpy(source.wifi_ssid, "SourceWifi", sizeof(source.wifi_ssid) - 1);
  strncpy(source.mqtt_host, "broker.example", sizeof(source.mqtt_host) - 1);
  source.mqtt_port = 1883;
  strncpy(source.auth_user, "sourceadmin", sizeof(source.auth_user) - 1);
  source.bms_count = 7;

  JsonDocument doc;
  mqtt::config_backup::build_json(source, doc);

  // Simulate the restore path: overlay onto a "live" config that has its own
  // WiFi/MQTT-broker/auth-username settings, which must survive untouched.
  Config live = DEFAULT_CONFIG;
  strncpy(live.wifi_ssid, "LiveWifi", sizeof(live.wifi_ssid) - 1);
  strncpy(live.mqtt_host, "live-broker.local", sizeof(live.mqtt_host) - 1);
  live.mqtt_port = 8883;
  strncpy(live.auth_user, "liveadmin", sizeof(live.auth_user) - 1);

  web::json_to_config(doc, live);

  REQUIRE(std::string(live.wifi_ssid) == "LiveWifi");
  REQUIRE(std::string(live.mqtt_host) == "live-broker.local");
  REQUIRE(live.mqtt_port == 8883);
  REQUIRE(std::string(live.auth_user) == "liveadmin");
  REQUIRE(live.bms_count == 7);
}
