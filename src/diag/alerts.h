#pragma once
#include <cstdint>
#include <cstddef>

// Alert emission API (architecture §4.9, §5.8).
//
// emit() posts an AlertEntry to q_alert. HousekeepingTask drains the queue
// and hands entries to storage::alerts_store for persistence.
//
// Throttle: duplicate (same source+message) within 60 s is suppressed.
// The next non-suppressed emit from that source appends "(N suppressed)".

namespace diag::alerts {

// Source IDs embedded in AlertEntry.flags low byte.
// Keep in sync with source_name() in alerts.cpp.
enum class Source : uint8_t {
  Unknown = 0,
  Poller  = 1,
  Safety  = 2,
  Can     = 3,
  Mqtt    = 4,
  Wifi    = 5,
  Boot    = 6,
  Ota     = 7,
};

// Severity levels matching AlertEntry.severity byte.
enum class Severity : uint8_t {
  Info     = 0,
  Warn     = 1,
  Error    = 2,
  Critical = 3,
};

// Emit an alert. Thread-safe. Non-blocking: if q_alert is full, drops oldest
// entry from the queue before inserting (counter maintained in flags).
// source: one of the Source enum values as a const string ("poller", "wifi"…).
void emit(Severity sev, const char* source, const char* fmt, ...)
    __attribute__((format(printf, 3, 4)));

// Convert source name string → Source enum (used internally and by tests).
Source source_from_name(const char* name);

// Convert Source enum → human-readable name string.
const char* source_name(Source s);

#ifdef NATIVE_BUILD
// Test helpers: inspect the alerts accumulated during a test run.
struct TestAlert {
  uint32_t ts_epoch;
  uint8_t  severity;    // 0=info, 1=warn, 2=err, 3=crit
  uint8_t  source_id;   // Source enum value
  char     message[120];
};

size_t test_get_all(TestAlert* out, size_t max_count);
void   test_reset();
#endif

}  // namespace diag::alerts
