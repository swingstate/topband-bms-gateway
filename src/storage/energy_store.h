#pragma once
#include <cstdint>

// Forward-declare to avoid circular includes.
struct Config;

// ── energy_store — persistent kWh counters (architecture §6.7) ──────────────
//
// Persistent file: /lfs/energy/counters.bin
// Written every hour if dirty (HousekeepingTask cadence).
// Atomic write via tmp+rename.
//
// Thread-safety: single writer (energy_integrator called from ControlTask).
// Readers (HTTP handler) read the values under no lock since they are floats
// and we accept slightly stale values (acceptable for a dashboard display).

namespace storage::energy_store {

// Load from /lfs/energy/counters.bin; create fresh file if missing or corrupt.
bool init();

// Accumulate signed energy. Positive = charging (in); negative = discharging (out).
// Called from energy_integrator (ControlTask context), which calls this every cycle.
void accumulate(float delta_kwh);

// Called every integration cycle: if calendar day has rolled over (local time),
// rotate today → yesterday, reset today counters.
// tz_offset_h: signed hours from UTC (same as cfg.timezone_offset_h).
void check_daily_rollover(uint32_t now_unix_s, int8_t tz_offset_h);

// Accessors — snapshot of current counters. Values may be read from any task.
float today_in_kwh();
float today_out_kwh();
float total_in_kwh();
float total_out_kwh();
// Weekly and monthly sums (sum of daily/monthly arrays).
float week_in_kwh();
float week_out_kwh();
float month_in_kwh();
float month_out_kwh();

// Flush to LittleFS. Returns false on I/O error.
// Called hourly from HousekeepingTask.
bool persist();

}  // namespace storage::energy_store
