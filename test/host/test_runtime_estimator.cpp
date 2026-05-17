#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include "bms/runtime_estimator.h"
#include "safety_state.h"

using namespace bms::runtime_estimator;
using Catch::Matchers::WithinAbs;

static SafetyState make_safety(float current, float soc, float cap_ah) {
  SafetyState s = {};
  s.pack_current_total = current;
  s.soc_avg            = soc;
  s.capacity_total_ah  = cap_ah;
  return s;
}

TEST_CASE("runtime_estimator: idle when |current| <= 0.5 A", "[runtime_estimator]") {
  RuntimeStateEst state;
  SafetyState s = make_safety(0.0f, 50.0f, 200.0f);
  int32_t r = estimate_min(s, state);
  REQUIRE(r == -1);
  REQUIRE(state == RuntimeStateEst::Idle);
}

TEST_CASE("runtime_estimator: idle at threshold boundary (0.5 A)", "[runtime_estimator]") {
  RuntimeStateEst state;
  int32_t r;

  r = estimate_min(make_safety(0.5f, 50.0f, 200.0f), state);
  REQUIRE(r == -1);
  REQUIRE(state == RuntimeStateEst::Idle);

  r = estimate_min(make_safety(-0.5f, 50.0f, 200.0f), state);
  REQUIRE(r == -1);
  REQUIRE(state == RuntimeStateEst::Idle);
}

TEST_CASE("runtime_estimator: charging → UntilFull", "[runtime_estimator]") {
  RuntimeStateEst state;
  // 100 Ah cap, 50% SOC → 50 Ah needed; at 50 A → 1 h = 60 min.
  SafetyState s = make_safety(50.0f, 50.0f, 100.0f);
  int32_t r = estimate_min(s, state);
  REQUIRE(state == RuntimeStateEst::UntilFull);
  REQUIRE(r == 60);
}

TEST_CASE("runtime_estimator: discharging → UntilEmpty", "[runtime_estimator]") {
  RuntimeStateEst state;
  // 200 Ah cap, 75% SOC → 150 Ah remain; at -50 A → 3 h = 180 min.
  SafetyState s = make_safety(-50.0f, 75.0f, 200.0f);
  int32_t r = estimate_min(s, state);
  REQUIRE(state == RuntimeStateEst::UntilEmpty);
  REQUIRE(r == 180);
}

TEST_CASE("runtime_estimator: zero capacity → idle", "[runtime_estimator]") {
  RuntimeStateEst state;
  SafetyState s = make_safety(10.0f, 50.0f, 0.0f);
  int32_t r = estimate_min(s, state);
  REQUIRE(r == -1);
  REQUIRE(state == RuntimeStateEst::Idle);
}

TEST_CASE("runtime_estimator: SOC out of range → idle", "[runtime_estimator]") {
  RuntimeStateEst state;
  int32_t r;

  r = estimate_min(make_safety(10.0f, -1.0f, 100.0f), state);
  REQUIRE(r == -1);
  REQUIRE(state == RuntimeStateEst::Idle);

  r = estimate_min(make_safety(10.0f, 101.0f, 100.0f), state);
  REQUIRE(r == -1);
  REQUIRE(state == RuntimeStateEst::Idle);
}

TEST_CASE("runtime_estimator: charging at 100% SOC → 0 min", "[runtime_estimator]") {
  RuntimeStateEst state;
  // 0 Ah needed → 0 h → rounds to 0 min, not -1.
  SafetyState s = make_safety(10.0f, 100.0f, 100.0f);
  int32_t r = estimate_min(s, state);
  REQUIRE(state == RuntimeStateEst::UntilFull);
  REQUIRE(r == 0);
}
