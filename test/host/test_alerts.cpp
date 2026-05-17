#include <catch2/catch_test_macros.hpp>
#include <cstring>

// NATIVE_BUILD: emit() stores into a test ring; no FreeRTOS queues.
#include "diag/alerts.h"

// Minimal stub for bus/queues.h inclusion required by alerts.cpp.
// alerts.cpp includes bus/queues.h for AlertEntry. In NATIVE_BUILD mode
// q_alert is not used (the queue send path is ifdef'd out).
// bus/queues.h declares q_alert as extern; this TU provides it.
#include "bus/queues.h"
QueueHandle_t q_log           = nullptr;
QueueHandle_t q_alert         = nullptr;
QueueHandle_t q_mqtt_publish  = nullptr;
QueueHandle_t q_history_sample = nullptr;
QueueHandle_t q_safety_event  = nullptr;
QueueHandle_t q_ota           = nullptr;

using namespace diag::alerts;

TEST_CASE("alerts: source name round-trip", "[alerts]") {
  REQUIRE(strcmp(source_name(Source::Poller), "poller") == 0);
  REQUIRE(strcmp(source_name(Source::Safety), "safety") == 0);
  REQUIRE(strcmp(source_name(Source::Can),    "can")    == 0);
  REQUIRE(strcmp(source_name(Source::Mqtt),   "mqtt")   == 0);
  REQUIRE(strcmp(source_name(Source::Wifi),   "wifi")   == 0);
  REQUIRE(strcmp(source_name(Source::Boot),   "boot")   == 0);
  REQUIRE(strcmp(source_name(Source::Ota),    "ota")    == 0);

  REQUIRE(source_from_name("poller") == Source::Poller);
  REQUIRE(source_from_name("wifi")   == Source::Wifi);
  REQUIRE(source_from_name("boot")   == Source::Boot);
  REQUIRE(source_from_name("xyz")    == Source::Unknown);
  REQUIRE(source_from_name(nullptr)  == Source::Unknown);
}

TEST_CASE("alerts: basic emit stores alert", "[alerts]") {
  test_reset();

  emit(Severity::Info,  "boot",   "started, version=3.0.0");
  emit(Severity::Warn,  "wifi",   "disconnected");
  emit(Severity::Error, "can",    "bus-off entered");

  TestAlert out[16];
  size_t n = test_get_all(out, 16);
  REQUIRE(n == 3);

  REQUIRE(out[0].severity == 0);  // Info
  REQUIRE(out[0].source_id == (uint8_t)Source::Boot);
  REQUIRE(strstr(out[0].message, "started") != nullptr);

  REQUIRE(out[1].severity == 1);  // Warn
  REQUIRE(out[1].source_id == (uint8_t)Source::Wifi);
  REQUIRE(strcmp(out[1].message, "disconnected") == 0);

  REQUIRE(out[2].severity == 2);  // Error
  REQUIRE(out[2].source_id == (uint8_t)Source::Can);
}

TEST_CASE("alerts: throttle suppresses duplicates within window", "[alerts]") {
  test_reset();

  // In NATIVE_BUILD, mono_ms() always returns 0, so all emits happen at t=0.
  // The second and subsequent identical emit should be suppressed.
  emit(Severity::Warn, "wifi", "disconnected");
  emit(Severity::Warn, "wifi", "disconnected");
  emit(Severity::Warn, "wifi", "disconnected");

  TestAlert out[16];
  size_t n = test_get_all(out, 16);
  // Only one alert should be stored (the first one; subsequent are suppressed).
  REQUIRE(n == 1);
}

TEST_CASE("alerts: different sources are not throttled together", "[alerts]") {
  test_reset();

  emit(Severity::Warn, "wifi", "disconnected");
  emit(Severity::Warn, "mqtt", "disconnected");  // same message, different source
  emit(Severity::Warn, "can",  "disconnected");  // same message, different source

  TestAlert out[16];
  size_t n = test_get_all(out, 16);
  REQUIRE(n == 3);
}

TEST_CASE("alerts: different messages from same source are not throttled", "[alerts]") {
  test_reset();

  emit(Severity::Info, "boot", "started, version=3.0.0");
  emit(Severity::Info, "boot", "other message");

  TestAlert out[16];
  size_t n = test_get_all(out, 16);
  REQUIRE(n == 2);
}

TEST_CASE("alerts: printf formatting works", "[alerts]") {
  test_reset();

  emit(Severity::Warn, "poller", "pack %u offline", 3u);
  emit(Severity::Info, "poller", "pack %u online",  5u);

  TestAlert out[16];
  size_t n = test_get_all(out, 16);
  REQUIRE(n == 2);
  REQUIRE(strstr(out[0].message, "pack 3") != nullptr);
  REQUIRE(strstr(out[1].message, "pack 5") != nullptr);
}

TEST_CASE("alerts: critical severity stored correctly", "[alerts]") {
  test_reset();

  emit(Severity::Critical, "boot", "5x power-cycle reset triggered");

  TestAlert out[4];
  size_t n = test_get_all(out, 4);
  REQUIRE(n == 1);
  REQUIRE(out[0].severity == 3);  // Critical
}
