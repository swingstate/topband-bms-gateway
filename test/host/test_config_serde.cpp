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

TEST_CASE("Config struct is 644 bytes (v3 layout: 540 + 104 net)", "[config]") {
  // v2 baseline: 540 B (verified by static_assert in config.cpp).
  // v3 adds: bool(1) + char[80] + char[24] = 105 B payload; one bool fills an
  // existing tail gap so net struct growth is 104 B → 644 B.  644 is 4-byte
  // aligned; no trailing padding.  This test mirrors the static_assert in
  // config.cpp and catches ABI drift between the host build and the target.
  REQUIRE(sizeof(Config) == 644);
  REQUIRE(sizeof(Config) > sizeof(uint16_t));   // at least schema_version
}
