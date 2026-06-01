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
//   Any device that has been running stably for ≥ 30 s calls mark_healthy(), which
//   clears the ring.  This prevents legitimate SW restarts (OTA apply, settings save)
//   from accumulating across separate boot cycles.  mark_healthy() must be called
//   from a task that runs on every normal boot; the housekeeping task does this.

namespace storage::boot_reasons {

// Record this boot's entry in the ring.  Call early in boot, after NVS init.
// Only ESP_RST_SW entries are counted; crashes/power-events are silently ignored.
void record_this_boot();

// Must be called once the device has been running stably for ≥ 30 s (housekeeping
// task calls this).  Clears the ring so that legitimate long-running boots never
// accumulate toward the wipe threshold across separate restart cycles.
void mark_healthy();

// Returns true if the last 5 recorded boots all had uptime < 5 000 ms,
// indicating the user rapid-reset the device 5 times.
bool is_5x_reset_detected();

// Clear the ring (call after acting on a 5x reset so subsequent normal boots
// don't keep re-triggering the detection).
void clear();

}  // namespace storage::boot_reasons
