#pragma once
#include <cstdint>

// ── WiFi reconnect backoff policy (V3.2) ────────────────────────────────────
// Pure, host-testable policy extracted from the event-driven STA reconnect
// state machine in wifi.cpp (which cannot be built on host — it pulls in
// esp_wifi/esp_netif/esp_event). Isolating the algorithm here lets the growth
// and cap behaviour be unit-tested without any ESP-IDF dependency.
//
// Root cause this replaces: wifi.cpp used to give up permanently after
// MAX_RETRY=5 failed reconnect attempts (~15-30 s) and never tried again at
// runtime — an unattended gateway must never reach a state where it stops
// trying to reach its configured AP. next_delay_ms() always returns a finite,
// bounded, positive delay for attempt >= 1: there is no input that yields "no
// further attempt" (see test/host/test_wifi_backoff.cpp for a proof-by-sweep
// over attempt counts, including very large ones, that this holds).
namespace net::wifi::backoff {

// First backoff wait after the initial (immediate) reconnect attempt fails.
constexpr uint32_t BASE_MS = 3000;         // 3 s
// Cap: never wait longer than this between attempts, however long the outage.
// Long enough to be neighborly to the AP/RF environment during an extended
// outage; short enough that the gateway is back within one interval of the
// AP actually returning.
constexpr uint32_t MAX_MS = 120000;        // 2 min
constexpr uint32_t MULTIPLIER = 2;

// attempt: 1-based count of connect attempts already made during the current
// outage (the attempt whose failure we are backing off from). attempt 1 is
// the immediate post-disconnect retry that wifi.cpp fires with no delay —
// next_delay_ms is first consulted for attempt=1, to size the wait before
// attempt 2.
//
// Returns the delay in ms before the next attempt, doubling each step and
// clamped to MAX_MS. Never overflows (clamped before the multiply can grow
// unbounded) and never returns 0 for attempt >= 1, so the caller can always
// schedule a timer rather than busy-loop.
constexpr uint32_t next_delay_ms(uint32_t attempt) {
  if (attempt == 0) return 0;
  uint64_t d = static_cast<uint64_t>(BASE_MS);
  for (uint32_t i = 1; i < attempt; ++i) {
    d *= MULTIPLIER;
    if (d >= MAX_MS) return MAX_MS;
  }
  return (d > MAX_MS) ? MAX_MS : static_cast<uint32_t>(d);
}

// Minimum spacing between periodic "still down" alert log entries during an
// extended outage. Keeps the Alerts log informative without spamming one
// entry per retry attempt (which would happen every few seconds once the
// backoff cap is reached, over a multi-hour outage).
constexpr uint32_t STILL_DOWN_ALERT_INTERVAL_MS = 15u * 60u * 1000u;  // 15 min

}  // namespace net::wifi::backoff
