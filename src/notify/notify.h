#pragma once
#include "provider.h"
#include "storage/config.h"
#include "safety_state.h"
#include <cstddef>
#include <cstdint>

// ── Notification dispatcher ───────────────────────────────────────────────────
//
// Thread safety: all public functions are safe to call from any task.
// send() and on_safety_event() serialize TLS operations via a binary semaphore
// so at most one TLS handshake runs at a time, keeping DRAM pressure bounded.

namespace notify {

// Initialize the notify module (create TLS serialization semaphore, load
// persisted verified state).  Call once from app::run_boot() before tasks start.
void init();

// Fan out a message to all enabled providers.  Non-blocking — work runs in a
// short-lived FreeRTOS task.  Acquires the TLS semaphore; drops silently if
// another send is already in flight.  Failures are logged; callers are not notified.
void send(Severity severity, const char* title, const char* body);

// Route a SafetyEvent through the debounce layer, then to alert log and
// notifications.  Called per-event from the BMS poller after runSafety().
// Debounce (A1): begin events are held until notify_debounce_s elapses; if the
// matching clear arrives first both are suppressed.  The safety CONTROL LOOP is
// unaffected — only the human-facing log/notify layer is debounced.
// A2: alert log text and Telegram body derive from the same format_event_body().
// Non-blocking; drops silently when rate-limited or TLS is busy.
void on_safety_event(const SafetyState::EventEntry& entry, uint32_t now_ms);

// Flush debounced events whose window has expired.  Call once per poller cycle
// AFTER processing all events for that cycle (same task, same thread).
void flush_pending_alerts(uint32_t now_ms);

// ── Test send ─────────────────────────────────────────────────────────────────

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

// ── Verified state ────────────────────────────────────────────────────────────

// Call after a successful test send to persist the verified state to NVS.
void mark_telegram_verified();

// Call when Telegram credentials change to reset verified state.
// Persists the reset to NVS if the old state was verified.
void reset_telegram_verified();

// Polled status for the UI (GET /api/notify/status).
struct TelegramStatus {
  bool     token_stored;     // true if notify_telegram_token is non-empty
  bool     chat_id_stored;   // true if notify_telegram_chat_id is non-empty
  bool     verified;         // true if last test/send succeeded
  uint32_t last_ok_ts;       // epoch of last successful send; 0 if never
};
TelegramStatus telegram_status();

}  // namespace notify
