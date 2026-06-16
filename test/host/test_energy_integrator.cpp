#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include "bms/energy_integrator.h"

using Catch::Matchers::WithinRel;

// ── Stubs for storage::energy_store ──────────────────────────────────────────
// energy_integrator calls two energy_store functions. We stub them here to
// capture their arguments without linking the real LittleFS implementation.

namespace storage::energy_store {

static float   s_last_delta   = 0.0f;
static int     s_accum_calls  = 0;
static uint32_t s_last_now_s  = 0;
static int8_t  s_last_tz      = 0;
static int     s_rollover_calls = 0;

void accumulate(float delta_kwh) {
  s_last_delta = delta_kwh;
  s_accum_calls++;
}

void check_daily_rollover(uint32_t now_unix_s, int8_t tz_offset_h) {
  s_last_now_s  = now_unix_s;
  s_last_tz     = tz_offset_h;
  s_rollover_calls++;
}

// Provide minimal stubs so the linker is satisfied.
bool init()                { return true; }
bool persist()             { return true; }
float today_in_kwh()       { return 0.0f; }
float today_out_kwh()      { return 0.0f; }
float total_in_kwh()       { return 0.0f; }
float total_out_kwh()      { return 0.0f; }
float week_in_kwh()        { return 0.0f; }
float week_out_kwh()       { return 0.0f; }
float month_in_kwh()       { return 0.0f; }
float month_out_kwh()      { return 0.0f; }

static void reset_stubs() {
  s_last_delta    = 0.0f;
  s_accum_calls   = 0;
  s_last_now_s    = 0;
  s_last_tz       = 0;
  s_rollover_calls = 0;
}

}  // namespace storage::energy_store

// ── Tests ─────────────────────────────────────────────────────────────────────

TEST_CASE("energy_integrator: correct kWh for 1000 W over 36 s", "[energy_integrator]") {
  storage::energy_store::reset_stubs();
  // dt_s = 36 s (within guard limit of 60 s).
  // 1000 W × 36 s / 3600 / 1000 = 0.01 kWh
  bms::energy_integrator::integrate(1000.0f, 36.0f, 0, 0);
  REQUIRE_THAT(storage::energy_store::s_last_delta, WithinRel(0.01f, 0.001f));
  REQUIRE(storage::energy_store::s_accum_calls == 1);
  REQUIRE(storage::energy_store::s_rollover_calls == 1);
}

TEST_CASE("energy_integrator: correct kWh for 500 W over 10 s", "[energy_integrator]") {
  storage::energy_store::reset_stubs();
  bms::energy_integrator::integrate(500.0f, 10.0f, 0, 0);
  // 500 W × 10 s / 3600 / 1000 = ~0.001388 kWh
  const float expected = 500.0f * 10.0f / 3600.0f / 1000.0f;
  REQUIRE_THAT(storage::energy_store::s_last_delta, WithinRel(expected, 0.001f));
}

TEST_CASE("energy_integrator: negative power → negative delta (discharge)", "[energy_integrator]") {
  storage::energy_store::reset_stubs();
  bms::energy_integrator::integrate(-800.0f, 10.0f, 0, 0);
  REQUIRE(storage::energy_store::s_last_delta < 0.0f);
  REQUIRE_THAT(storage::energy_store::s_last_delta,
               WithinRel(-800.0f * 10.0f / 3600.0f / 1000.0f, 0.001f));
}

TEST_CASE("energy_integrator: guard rejects dt_s <= 0", "[energy_integrator]") {
  storage::energy_store::reset_stubs();
  bms::energy_integrator::integrate(1000.0f, 0.0f, 0, 0);
  REQUIRE(storage::energy_store::s_accum_calls == 0);

  bms::energy_integrator::integrate(1000.0f, -5.0f, 0, 0);
  REQUIRE(storage::energy_store::s_accum_calls == 0);
}

TEST_CASE("energy_integrator: guard rejects dt_s > 60", "[energy_integrator]") {
  storage::energy_store::reset_stubs();
  bms::energy_integrator::integrate(1000.0f, 61.0f, 0, 0);
  REQUIRE(storage::energy_store::s_accum_calls == 0);
}

TEST_CASE("energy_integrator: passes now_unix_s and tz_offset_h to rollover", "[energy_integrator]") {
  storage::energy_store::reset_stubs();
  bms::energy_integrator::integrate(100.0f, 10.0f, 1700000000u, 2);
  REQUIRE(storage::energy_store::s_last_now_s == 1700000000u);
  REQUIRE(storage::energy_store::s_last_tz    == 2);
}
