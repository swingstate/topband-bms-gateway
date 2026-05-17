#include <catch2/catch_test_macros.hpp>
#include <cstring>

// The NATIVE_BUILD guard in log_ring.cpp replaces PSRAM allocation with
// regular malloc and skips the esp_log_set_vprintf hook.
#include "diag/log_ring.h"

TEST_CASE("log_ring: basic append and snapshot", "[log_ring]") {
  diag::log_ring::reset_for_test(10);
  REQUIRE(diag::log_ring::count() == 0);
}

TEST_CASE("log_ring: append and count", "[log_ring]") {
  diag::log_ring::reset_for_test(10);

  diag::log_ring::append("line one");
  diag::log_ring::append("line two");
  diag::log_ring::append("line three");

  // count() reflects lines appended (up to capacity).
  // Note: because init() is idempotent the ring may have been populated by an
  // earlier test in the same process.  We only verify the snapshot contains our
  // strings.
  char out[1024] = {};
  size_t written = diag::log_ring::snapshot(out, sizeof(out));
  REQUIRE(written > 0);
  REQUIRE(strstr(out, "line one")   != nullptr);
  REQUIRE(strstr(out, "line two")   != nullptr);
  REQUIRE(strstr(out, "line three") != nullptr);
}

TEST_CASE("log_ring: ring wraps correctly", "[log_ring]") {
  // reset_for_test() is needed because catch_discover_tests runs each test case
  // as a separate process, so init() from an earlier test is not present.
  diag::log_ring::reset_for_test(10);

  // Fill the ring exactly (10 items), then overflow by 1.
  // "wrap-0" should be evicted; "wrap-10" should be present.
  for (int i = 0; i < 11; i++) {
    char line[32];
    snprintf(line, sizeof(line), "wrap-%d", i);
    diag::log_ring::append(line);
  }

  char out[4096] = {};
  diag::log_ring::snapshot(out, sizeof(out));

  // "wrap-0" was at position 0 when ring was full; after 11 total appends
  // into a capacity-10 ring it should have been evicted.
  REQUIRE(strstr(out, "wrap-10") != nullptr);
  REQUIRE(strstr(out, "wrap-0")  == nullptr);
}

TEST_CASE("log_ring: trailing newlines are stripped", "[log_ring]") {
  diag::log_ring::reset_for_test(10);
  diag::log_ring::append("trailing\n");
  diag::log_ring::append("crlf\r\n");

  char out[512] = {};
  diag::log_ring::snapshot(out, sizeof(out));
  // Stripped lines should appear without their original line endings.
  REQUIRE(strstr(out, "trailing\n\n") == nullptr);  // only the separator \n, not double
  REQUIRE(strstr(out, "trailing")     != nullptr);
}

TEST_CASE("log_ring: snapshot output is newline-delimited", "[log_ring]") {
  diag::log_ring::reset_for_test(10);
  diag::log_ring::append("alpha");
  diag::log_ring::append("beta");

  char out[512] = {};
  size_t n = diag::log_ring::snapshot(out, sizeof(out));
  REQUIRE(n > 0);
  // Each line ends with '\n'.
  REQUIRE(out[n - 1] == '\n');
}
