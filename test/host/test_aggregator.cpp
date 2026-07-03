#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>
#include "sources/aggregator.h"

using sources::Aggregator;
using sources::Metric;
using sources::ReadingStatus;
using sources::SourceReading;

namespace {
SourceReading reading(float v, ReadingStatus st, const char* unit = "A") {
  return { v, unit, 1000, st };
}
}  // namespace

// ── select_bank_value: Auto policy ──────────────────────────────────────────

TEST_CASE("select_bank_value: Auto, shunt fresh -> shunt leads regardless of bms_valid",
          "[aggregator][auto]") {
  SourceReading shunt_r = reading(12.5f, ReadingStatus::Valid);
  auto r = Aggregator::select_bank_value(0.0f, /*bms_valid=*/false, shunt_r,
                                         Config::BatterySourcePolicy::Auto,
                                         Config::MetricSource::Bms);
  REQUIRE(r.value == Catch::Approx(12.5f));
  REQUIRE(r.from_shunt == true);
  REQUIRE(r.valid == true);
}

TEST_CASE("select_bank_value: Auto, shunt stale -> falls back to BMS", "[aggregator][auto]") {
  SourceReading shunt_r = reading(12.5f, ReadingStatus::Stale);
  auto r = Aggregator::select_bank_value(3.0f, /*bms_valid=*/true, shunt_r,
                                         Config::BatterySourcePolicy::Auto,
                                         Config::MetricSource::Bms);
  REQUIRE(r.value == Catch::Approx(3.0f));
  REQUIRE(r.from_shunt == false);
  REQUIRE(r.valid == true);
}

TEST_CASE("select_bank_value: Auto, shunt unavailable -> falls back to BMS", "[aggregator][auto]") {
  SourceReading shunt_r = sources::unavailable_reading("A");
  auto r = Aggregator::select_bank_value(3.0f, /*bms_valid=*/true, shunt_r,
                                         Config::BatterySourcePolicy::Auto,
                                         Config::MetricSource::Bms);
  REQUIRE(r.from_shunt == false);
  REQUIRE(r.valid == true);
}

TEST_CASE("select_bank_value: Auto, shunt stale AND bms invalid -> honest no-data",
          "[aggregator][auto]") {
  SourceReading shunt_r = reading(12.5f, ReadingStatus::Stale);
  auto r = Aggregator::select_bank_value(0.0f, /*bms_valid=*/false, shunt_r,
                                         Config::BatterySourcePolicy::Auto,
                                         Config::MetricSource::Bms);
  REQUIRE(r.from_shunt == false);
  REQUIRE(r.valid == false);  // neither source usable right now -> UI shows "--"
}

// ── select_bank_value: Manual policy ────────────────────────────────────────

TEST_CASE("select_bank_value: Manual+Bms -> always BMS, never falls back to shunt",
          "[aggregator][manual]") {
  SourceReading shunt_r = reading(99.0f, ReadingStatus::Valid);
  auto r = Aggregator::select_bank_value(3.0f, /*bms_valid=*/true, shunt_r,
                                         Config::BatterySourcePolicy::Manual,
                                         Config::MetricSource::Bms);
  REQUIRE(r.value == Catch::Approx(3.0f));
  REQUIRE(r.from_shunt == false);
  REQUIRE(r.valid == true);
}

TEST_CASE("select_bank_value: Manual+Shunt, shunt fresh -> shunt value, valid",
          "[aggregator][manual]") {
  SourceReading shunt_r = reading(48.5f, ReadingStatus::Valid, "V");
  auto r = Aggregator::select_bank_value(50.0f, /*bms_valid=*/true, shunt_r,
                                         Config::BatterySourcePolicy::Manual,
                                         Config::MetricSource::Shunt);
  REQUIRE(r.value == Catch::Approx(48.5f));
  REQUIRE(r.from_shunt == true);
  REQUIRE(r.valid == true);
}

TEST_CASE("select_bank_value: Manual+Shunt, shunt stale -> no silent BMS fallback",
          "[aggregator][manual]") {
  // An explicit Manual+Shunt choice that turns out stale/unavailable reads as
  // "no data" (valid=false) -- it must NOT silently substitute the BMS value,
  // even though BMS is valid and available.
  SourceReading shunt_r = reading(48.5f, ReadingStatus::Stale, "V");
  auto r = Aggregator::select_bank_value(50.0f, /*bms_valid=*/true, shunt_r,
                                         Config::BatterySourcePolicy::Manual,
                                         Config::MetricSource::Shunt);
  REQUIRE(r.from_shunt == true);
  REQUIRE(r.valid == false);
}

TEST_CASE("select_bank_value: Manual+Shunt, shunt unavailable -> no data",
          "[aggregator][manual]") {
  SourceReading shunt_r = sources::unavailable_reading("V");
  auto r = Aggregator::select_bank_value(50.0f, /*bms_valid=*/true, shunt_r,
                                         Config::BatterySourcePolicy::Manual,
                                         Config::MetricSource::Shunt);
  REQUIRE(r.from_shunt == true);
  REQUIRE(r.valid == false);
}

// ── select: raw passthrough (PV_*, SHUNT_SOC, TOTAL_CURRENT/VOLTAGE) ────────

TEST_CASE("select: TOTAL_CURRENT/TOTAL_VOLTAGE are raw BMS passthrough, not fused",
          "[aggregator][select]") {
  SourceReading bms_r   = reading(1.0f, ReadingStatus::Valid);
  SourceReading shunt_r = reading(2.0f, ReadingStatus::Valid);
  SourceReading mppt_r  = sources::unavailable_reading("A");
  auto cur = Aggregator::select(Metric::TOTAL_CURRENT, bms_r, shunt_r, mppt_r);
  REQUIRE(cur.value == Catch::Approx(1.0f));
  auto volt = Aggregator::select(Metric::TOTAL_VOLTAGE, bms_r, shunt_r, mppt_r);
  REQUIRE(volt.value == Catch::Approx(1.0f));
}

TEST_CASE("select: PV_POWER is MPPT-only", "[aggregator][select]") {
  SourceReading bms_r   = sources::unavailable_reading("W");
  SourceReading shunt_r = sources::unavailable_reading("W");
  SourceReading mppt_r  = reading(400.0f, ReadingStatus::Valid, "W");
  auto r = Aggregator::select(Metric::PV_POWER, bms_r, shunt_r, mppt_r);
  REQUIRE(r.value == Catch::Approx(400.0f));
}

TEST_CASE("select: SHUNT_SOC is shunt-only raw passthrough", "[aggregator][select]") {
  SourceReading bms_r   = reading(80.0f, ReadingStatus::Valid, "%");
  SourceReading shunt_r = reading(94.0f, ReadingStatus::Valid, "%");
  SourceReading mppt_r  = sources::unavailable_reading("%");
  auto r = Aggregator::select(Metric::SHUNT_SOC, bms_r, shunt_r, mppt_r);
  REQUIRE(r.value == Catch::Approx(94.0f));
}
