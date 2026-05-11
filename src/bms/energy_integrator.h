#pragma once
#include <cstdint>

// ── energy_integrator — Power × dt → kWh accumulation ────────────────────────
// Called from ControlTask (Core 0) after each poll cycle.
// Delegates persistence to storage::energy_store.

namespace bms::energy_integrator {

// Integrate signed power over dt.
// pack_power_w: sum of all packs (positive = charging = energy in).
// dt_s: time since last call in seconds.
// now_unix_s: current UTC unix timestamp (0 if NTP not synced).
// tz_offset_h: from Config::timezone_offset_h, for daily rollover detection.
void integrate(float pack_power_w, float dt_s,
               uint32_t now_unix_s, int8_t tz_offset_h);

}  // namespace bms::energy_integrator
