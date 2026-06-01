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
