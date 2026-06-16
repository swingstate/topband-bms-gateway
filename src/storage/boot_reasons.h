#pragma once
#include <cstdint>

// Boot-reasons NVS ring — 5x rapid SW-reset detection for factory-reset gesture.
//
// Design intent: a deliberate 5x quick-reset gesture (e.g. 5 rapid button presses
// that trigger esp_restart or 5 quick UI restart actions) clears auth + WiFi,
// providing a no-screen recovery path.
//
// IMPORTANT — only ESP_RST_SW counts toward the tally.  All other reset reasons
// (POWERON, BROWNOUT, PANIC, WDT variants, etc.) are excluded.  See boot_reasons.cpp.
//
// Ring clearing — the other half of the protection:
//   Any device that has been running stably for >= 30 s calls mark_healthy(), which
//   clears the ring.  This prevents legitimate SW restarts (OTA apply, settings save)
//   from accumulating across separate boot cycles.  mark_healthy() must be called
//   from a task that runs on every normal boot; the housekeeping task does this.
//
// ── Panic-loop guard ──────────────────────────────────────────────────────────
// Separate NVS counter tracks consecutive panic/WDT reboots (ESP_RST_PANIC,
// ESP_RST_TASK_WDT, ESP_RST_INT_WDT).  If >= CRASH_LOOP_THRESHOLD consecutive
// panic boots are detected, is_crash_loop_suspected() returns true and the caller
// (notify::init()) disables outbound TLS for the current boot.  This prevents the
// notify subsystem from contributing to a crash loop even if a provider-side bug
// re-emerges.  Safety/control and CAN broadcast are never gated behind this.

namespace storage::boot_reasons {

// ── Factory-reset gesture (SW-reset ring) ─────────────────────────────────────

// Record this boot's entry in the ring.  Call early in boot, after NVS init.
// Only ESP_RST_SW entries are counted; crashes/power-events are silently ignored.
void record_this_boot();

// Must be called once the device has been running stably for >= 30 s (housekeeping
// task calls this).  Clears both the SW-reset ring and the panic counter so that
// legitimate long-running boots never accumulate toward either threshold.
void mark_healthy();

// Returns true if the last 5 recorded boots all had uptime < 5 000 ms,
// indicating the user rapid-reset the device 5 times.
bool is_5x_reset_detected();

// Clear the SW-reset ring (call after acting on a 5x reset so subsequent normal
// boots don't keep re-triggering the detection).
void clear();

// ── Panic-loop guard ──────────────────────────────────────────────────────────

// Call early in boot (after NVS init).  Increments the panic counter if the reset
// reason was ESP_RST_PANIC / TASK_WDT / INT_WDT; clears it on any clean boot.
void record_panic_boot();

// Returns true if the panic counter has reached the crash-loop threshold (3).
// Does NOT clear the counter — call mark_healthy() after stable operation.
bool is_crash_loop_suspected();

}  // namespace storage::boot_reasons
