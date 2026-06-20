#pragma once
#include <cstdint>
#include <cstdbool>

// ── Staged network self-test ──────────────────────────────────────────────────
// Runs five sequential stages in a background task and stores the results.
// Triggered via POST /api/net/self-test; polled via GET /api/net/self-test.
// All stages are read-only on the network stack — no credentials required.
// Bot tokens never appear in this path; the final end-to-end check is the
// existing "Send test" in the Notify settings.

namespace net::diag {

struct Stage {
  bool run;        // was this stage executed?
  bool pass;
  char detail[160];
};

struct Report {
  bool     running;
  int8_t   current_stage;  // -1=idle/done, 0=wifi, 1=dns, 2=tcp, 3=tls, 4=ntp
  uint32_t started_at;     // esp_timer_get_time() / 1e6, seconds since boot
  Stage    wifi;
  Stage    dns;
  Stage    tcp;
  Stage    tls;
  Stage    ntp;
};

// Start the self-test. Returns false if already running.
bool start();

// True while the background task is executing.
bool is_running();

// Snapshot the current (possibly in-progress) report.
Report get_report();

}  // namespace net::diag
