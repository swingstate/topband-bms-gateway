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
