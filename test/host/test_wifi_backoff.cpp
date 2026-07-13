#include <catch2/catch_test_macros.hpp>

#include "net/wifi_backoff.h"

using namespace net::wifi::backoff;

TEST_CASE("wifi_backoff: attempt 1 backs off by BASE_MS", "[wifi_backoff]") {
  REQUIRE(next_delay_ms(1) == BASE_MS);
}

TEST_CASE("wifi_backoff: doubles each subsequent attempt until the cap", "[wifi_backoff]") {
  REQUIRE(next_delay_ms(1) == 3000u);
  REQUIRE(next_delay_ms(2) == 6000u);
  REQUIRE(next_delay_ms(3) == 12000u);
  REQUIRE(next_delay_ms(4) == 24000u);
  REQUIRE(next_delay_ms(5) == 48000u);
  REQUIRE(next_delay_ms(6) == 96000u);
}

TEST_CASE("wifi_backoff: clamps at MAX_MS once the doubling would exceed it", "[wifi_backoff]") {
  REQUIRE(next_delay_ms(7) == MAX_MS);
  REQUIRE(next_delay_ms(8) == MAX_MS);
}

TEST_CASE("wifi_backoff: stays capped indefinitely — never gives up, never overflows",
          "[wifi_backoff]") {
  // Sweep a wide range of attempt counts, including values that would
  // overflow a naive uint32_t doubling loop (2^32 requires ~32 doublings).
  // Every single one must return a finite, bounded, positive delay: there is
  // no attempt count for which this function reports "stop trying". This is
  // the property that makes the permanent-StaFailed regression structurally
  // impossible to reintroduce via this code path.
  for (uint32_t attempt : {10u, 20u, 32u, 50u, 100u, 1000u, 100000u, 4000000000u}) {
    uint32_t d = next_delay_ms(attempt);
    REQUIRE(d == MAX_MS);
    REQUIRE(d > 0u);
  }
}

TEST_CASE("wifi_backoff: monotonic non-decreasing over the growth phase", "[wifi_backoff]") {
  uint32_t prev = 0;
  for (uint32_t attempt = 1; attempt <= 20; ++attempt) {
    uint32_t d = next_delay_ms(attempt);
    REQUIRE(d >= prev);
    REQUIRE(d <= MAX_MS);
    prev = d;
  }
}

TEST_CASE("wifi_backoff: attempt 0 is a no-wait sentinel (immediate first try)",
          "[wifi_backoff]") {
  REQUIRE(next_delay_ms(0) == 0u);
}

TEST_CASE("wifi_backoff: still-down alert interval is in the 10-15 minute range",
          "[wifi_backoff]") {
  REQUIRE(STILL_DOWN_ALERT_INTERVAL_MS >= 10u * 60u * 1000u);
  REQUIRE(STILL_DOWN_ALERT_INTERVAL_MS <= 15u * 60u * 1000u);
}

TEST_CASE("wifi_backoff: cumulative wait before hitting the cap is a few minutes, "
          "not instant hammering and not an unbounded stall", "[wifi_backoff]") {
  uint64_t cumulative = 0;
  uint32_t attempt = 1;
  while (next_delay_ms(attempt) < MAX_MS) {
    cumulative += next_delay_ms(attempt);
    ++attempt;
    REQUIRE(attempt < 100u);  // sanity bound so a broken policy can't spin forever here
  }
  // Reaches the cap within a handful of attempts (not attempt 1: that would be
  // hammering) and within a handful of minutes (not hours: that would be too
  // slow to recover from a brief-ish outage).
  REQUIRE(attempt > 1u);
  REQUIRE(cumulative > 30u * 1000u);          // more than 30 s of backoff growth
  REQUIRE(cumulative < 5u * 60u * 1000u);     // less than 5 minutes to reach the cap
}
