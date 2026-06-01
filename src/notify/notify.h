#pragma once
#include "provider.h"
#include "storage/config.h"
#include <cstddef>

// ── Notification dispatcher ───────────────────────────────────────────────────
//
// Public API.  Event-wiring (alerts → notify::send) is deferred to a later
// iteration; for now only the Test button calls into this module.
//
// Thread safety: notify::send() and notify::test() are safe to call from any
// task.  Both schedule work off-thread so they return immediately.

namespace notify {

// Fan out a message to all enabled providers.  Non-blocking — work runs in a
// short-lived FreeRTOS task.  Failures are logged; callers are not notified.
void send(Severity severity, const char* title, const char* body);

// Result of a test send (polled by GET /api/notify/<id>/test).
enum class TestStatus : uint8_t { Idle, Running, Ok, Failed };

struct TestResult {
  TestStatus status  = TestStatus::Idle;
  char message[128]  = {};
};

// Kick off a test send for the provider identified by provider_id.
// form_cfg: a Config built from the unsaved UI form values — token may be
//           empty if the user left the field blank (use saved token then).
// saved_cfg: the live saved config, used as fallback when form fields are empty.
// Non-blocking; result is polled via test_result().
// Returns false if a test is already running.
bool test(const char* provider_id,
          const Config& form_cfg,
          const Config& saved_cfg);

// Read the current test result (snapshot, safe to call from HTTP handler).
TestResult test_result();

}  // namespace notify
