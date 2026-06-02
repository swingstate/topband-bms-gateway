#include <catch2/catch_test_macros.hpp>
#include <ArduinoJson.h>
#include <cstring>
#include "storage/config.h"
#include "web/config_json.h"

// ── json_to_config: leave-blank-to-keep for notification credentials ──────────
//
// The token is always redacted to "" in GET /api/config responses.  When the UI
// saves an unrelated setting it sends back "" for the token; that must NOT erase
// the saved token.  The chat_id is echoed (not redacted) but we apply the same
// guard so that an accidental blank never silently wipes a configured value.

TEST_CASE("json_to_config: blank token preserves existing token", "[config_json][notify]") {
  Config cfg = DEFAULT_CONFIG;
  strncpy(cfg.notify_telegram_token, "bot12345:SAVED_TOKEN",
          sizeof(cfg.notify_telegram_token) - 1);
  strncpy(cfg.notify_telegram_chat_id, "123456789",
          sizeof(cfg.notify_telegram_chat_id) - 1);
  cfg.notify_telegram_enabled = true;

  // Simulate what the browser sends after GET /api/config: token is "" (redacted),
  // chat_id and enabled are echoed back.
  JsonDocument doc;
  doc["notify_telegram_enabled"] = true;
  doc["notify_telegram_token"]   = "";
  doc["notify_telegram_chat_id"] = "123456789";

  web::json_to_config(doc, cfg);

  REQUIRE(std::string(cfg.notify_telegram_token) == "bot12345:SAVED_TOKEN");
  REQUIRE(std::string(cfg.notify_telegram_chat_id) == "123456789");
  REQUIRE(cfg.notify_telegram_enabled == true);
}

TEST_CASE("json_to_config: blank chat_id preserves existing chat_id", "[config_json][notify]") {
  Config cfg = DEFAULT_CONFIG;
  strncpy(cfg.notify_telegram_chat_id, "987654321",
          sizeof(cfg.notify_telegram_chat_id) - 1);

  JsonDocument doc;
  doc["notify_telegram_enabled"] = true;
  doc["notify_telegram_token"]   = "";
  doc["notify_telegram_chat_id"] = "";   // blank must NOT overwrite

  web::json_to_config(doc, cfg);

  REQUIRE(std::string(cfg.notify_telegram_chat_id) == "987654321");
}

TEST_CASE("json_to_config: non-blank token updates token", "[config_json][notify]") {
  Config cfg = DEFAULT_CONFIG;

  JsonDocument doc;
  doc["notify_telegram_token"]   = "bot99999:NEW_TOKEN_VALUE";
  doc["notify_telegram_chat_id"] = "555555";
  doc["notify_telegram_enabled"] = true;

  web::json_to_config(doc, cfg);

  REQUIRE(std::string(cfg.notify_telegram_token) == "bot99999:NEW_TOKEN_VALUE");
  REQUIRE(std::string(cfg.notify_telegram_chat_id) == "555555");
  REQUIRE(cfg.notify_telegram_enabled == true);
}

TEST_CASE("json_to_config: unrelated save does not touch notification fields", "[config_json][notify]") {
  // Populate a config with Telegram credentials.
  Config cfg = DEFAULT_CONFIG;
  cfg.notify_telegram_enabled = true;
  strncpy(cfg.notify_telegram_token, "bot11111:KEEP_THIS",
          sizeof(cfg.notify_telegram_token) - 1);
  strncpy(cfg.notify_telegram_chat_id, "111222333",
          sizeof(cfg.notify_telegram_chat_id) - 1);

  // Simulate a POST /api/config that only changes an unrelated field (mqtt_level).
  // The UI echoes back: token="" (redacted), chat_id="111222333" (echoed), enabled=true.
  JsonDocument doc;
  doc["mqtt_level"]              = 2;
  doc["notify_telegram_enabled"] = true;
  doc["notify_telegram_token"]   = "";
  doc["notify_telegram_chat_id"] = "111222333";

  web::json_to_config(doc, cfg);

  // Token must be preserved.
  REQUIRE(std::string(cfg.notify_telegram_token) == "bot11111:KEEP_THIS");
  // Chat ID: echoed value written back (same content).
  REQUIRE(std::string(cfg.notify_telegram_chat_id) == "111222333");
  // Unrelated field updated.
  REQUIRE(cfg.mqtt_level == Config::MqttLevel::DataSystem);
}

// ── config_to_json: token is always redacted ──────────────────────────────────

TEST_CASE("config_to_json: token is always redacted (empty string)", "[config_json][notify]") {
  Config cfg = DEFAULT_CONFIG;
  strncpy(cfg.notify_telegram_token, "bot12345:SECRET_SHOULD_NOT_LEAK",
          sizeof(cfg.notify_telegram_token) - 1);

  JsonDocument doc;
  web::config_to_json(cfg, doc);

  const char* tok = doc["notify_telegram_token"] | "MISSING";
  REQUIRE(std::string(tok) == "");
}

TEST_CASE("config_to_json: chat_id is echoed (not redacted)", "[config_json][notify]") {
  Config cfg = DEFAULT_CONFIG;
  strncpy(cfg.notify_telegram_chat_id, "42424242",
          sizeof(cfg.notify_telegram_chat_id) - 1);

  JsonDocument doc;
  web::config_to_json(cfg, doc);

  const char* cid = doc["notify_telegram_chat_id"] | "";
  REQUIRE(std::string(cid) == "42424242");
}

// ── v4 notify fields: json_to_config / config_to_json ─────────────────────────

TEST_CASE("json_to_config: v4 fields notify_sender_name round-trips", "[config_json][notify][v4]") {
  Config cfg = DEFAULT_CONFIG;
  JsonDocument doc;
  doc["notify_sender_name"] = "GW-main";
  web::json_to_config(doc, cfg);
  REQUIRE(std::string(cfg.notify_sender_name) == "GW-main");
}

TEST_CASE("json_to_config: notify_alert_flags updated from JSON", "[config_json][notify][v4]") {
  Config cfg = DEFAULT_CONFIG;
  cfg.notify_alert_flags = 0u;  // clear
  JsonDocument doc;
  doc["notify_alert_flags"] = (uint32_t)0x0000000Au;
  web::json_to_config(doc, cfg);
  REQUIRE(cfg.notify_alert_flags == 0x0000000Au);
}

TEST_CASE("json_to_config: notify_poll_interval_s floor clamped to 60", "[config_json][notify][v4]") {
  Config cfg = DEFAULT_CONFIG;
  JsonDocument doc;
  doc["notify_poll_interval_s"] = (uint16_t)10u;  // below floor
  doc["notify_cooldown_s"]      = (uint16_t)5u;   // below floor
  web::json_to_config(doc, cfg);
  REQUIRE(cfg.notify_poll_interval_s >= 60u);
  REQUIRE(cfg.notify_cooldown_s      >= 60u);
}

TEST_CASE("json_to_config: notify_poll_interval_s above floor kept as-is", "[config_json][notify][v4]") {
  Config cfg = DEFAULT_CONFIG;
  JsonDocument doc;
  doc["notify_poll_interval_s"] = (uint16_t)300u;
  doc["notify_cooldown_s"]      = (uint16_t)180u;
  web::json_to_config(doc, cfg);
  REQUIRE(cfg.notify_poll_interval_s == 300u);
  REQUIRE(cfg.notify_cooldown_s      == 180u);
}

TEST_CASE("config_to_json: v4 notify fields are included in output", "[config_json][notify][v4]") {
  Config cfg = DEFAULT_CONFIG;
  cfg.notify_poll_interval_s = 120;
  cfg.notify_cooldown_s      = 240;
  cfg.notify_alert_flags     = 0xA5A5A5A5u;
  strncpy(cfg.notify_sender_name, "Workshop", sizeof(cfg.notify_sender_name) - 1);
  cfg.notify_telegram_verified = true;
  cfg.notify_telegram_last_ok_ts = 1700001000u;

  JsonDocument doc;
  web::config_to_json(cfg, doc);

  REQUIRE((uint32_t)(doc["notify_poll_interval_s"] | 0u)    == 120u);
  REQUIRE((uint32_t)(doc["notify_cooldown_s"]      | 0u)    == 240u);
  REQUIRE((uint32_t)(doc["notify_alert_flags"]     | 0u)    == 0xA5A5A5A5u);
  REQUIRE(std::string(doc["notify_sender_name"] | "")       == "Workshop");
  REQUIRE((bool)(doc["notify_telegram_verified"] | false)   == true);
  REQUIRE((uint32_t)(doc["notify_telegram_last_ok_ts"] | 0u) == 1700001000u);
}
