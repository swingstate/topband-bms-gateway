#pragma once
#include <cstdint>

// Boot-reasons NVS ring — 5x rapid SW-reset detection for factory-reset gesture.
//
// Design intent: a deliberate 5x quick-reset gesture (e.g. 5 rapid button presses
// that trigger esp_restart or 5 quick UI restart actions) clears auth + WiFi,
// providing a no-screen recovery path.
//
// IMPORTANT — POWERON/BROWNOUT are explicitly excluded:
//   On ESP32-S3, both a genuine power outage and pressing the hardware RESET/EN
//   pin produce ESP_RST_POWERON — they are indistinguishable. A mains-powered BMS
//   gateway must survive repeated power outages without losing credentials. Only
//   ESP_RST_SW (software restart) counts toward the tally.
//
// Normal-operation restarts (OTA apply, settings save) are excluded by the
// NORMAL_BOOT_MS threshold: a SW restart that follows a long-running boot clears
// the ring rather than accumulating it.

namespace storage::boot_reasons {

// Record this boot's uptime. Call as early as possible in the boot sequence,
// after NVS is initialised. Only counts POWERON and BROWNOUT reset causes.
void record_this_boot();

// Returns true if the last 5 recorded boots all had uptime < 5 000 ms,
// indicating the user rapid-power-cycled the device 5 times.
bool is_5x_reset_detected();

// Clear the ring (call after acting on a 5x reset so subsequent normal boots
// don't keep re-triggering the detection).
void clear();

}  // namespace storage::boot_reasons
