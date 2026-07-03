#pragma once
#include "bus/types.h"
#include <cstddef>

// ── solar_day_ring — one-day PSRAM ring for PV power (MPPT) ──────────────────
//
// Stores SOLAR_DAY_CAPACITY (288) points at 5-min cadence = 24 h window.
// All ring storage is in PSRAM via EXT_RAM_BSS_ATTR — zero net internal DRAM.
//
// Write path (HistoryTask only, every 5 min, same cadence as coarse ring):
//   append(t_epoch, pv_power_w, valid)
//
// Read path (HttpTask only, on /api/solar-day request):
//   read() fills caller-provided SolarDayPoint array, oldest-first.
//
// Persistence: save()/load() persist the ring to /lfs/solar_day.bin so data
// survives firmware restarts (PSRAM content is lost on power-cycle / OTA).
// load() is called once from HistoryTask before the main loop. save() is called
// at every coarse boundary (5 min). The existing day-rollover logic in append()
// handles the case where a restored ring belongs to a previous day.
//
// Day rollover: if the UTC-day of a new append() differs from stored data,
// the ring is cleared and the new day begins cleanly.
// Invariant: all stored points belong to the same UTC calendar day.

namespace app::solar_day_ring {

// Append one PV-power sample.
// valid=false → pv_power_w sentinel (0xFFFF) stored; client renders as null.
// Silently no-ops when t_epoch==0 (NTP not yet synced).
void append(uint32_t t_epoch, float pv_power_w, bool valid);

// Epoch of the oldest stored point (0 = ring empty / no NTP).
uint32_t t0_epoch();

// UTC midnight epoch for the current stored day (0 = ring empty).
uint32_t midnight_epoch();

// Number of valid points currently in the ring.
uint32_t count();

// Fill out[0..n-1] oldest-first. Returns count actually written (≤ max_count).
size_t read(SolarDayPoint* out, size_t max_count);

// Persist ring state to /lfs/solar_day.bin (atomic write). Called every 5 min
// from HistoryTask so data survives restarts. No-op on LittleFS write failure.
void save();

// Restore ring state from /lfs/solar_day.bin. Call once from HistoryTask before
// the main loop. No-op if file is missing, corrupt, or from a different day.
void load();

}  // namespace app::solar_day_ring
