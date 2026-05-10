#pragma once
#include <cstdint>

// Boot-reasons NVS ring — 5x rapid power-cycle reset detection.
// Architecture §1.7 D3.4: "5x power cycle within 20 seconds" resets auth + WiFi.
//
// Implementation: NTP is not running at boot, so there is no wall clock.
// Instead, each boot records its uptime-at-record (ms from esp_timer_get_time()).
// After a hard power-on the device has been running only a few hundred ms before
// this code executes. Storing the last 5 uptimes and checking that ALL are below
// RAPID_THRESHOLD_MS (5 000 ms) is a reliable proxy for "user yanked power 5
// times before the device fully started". Only ESP_RST_POWERON and
// ESP_RST_BROWNOUT causes are counted; software reboots (esp_restart) are
// excluded to avoid false triggers on OTA or settings saves.

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
