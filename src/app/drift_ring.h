#pragma once
#include "bms_snapshot.h"
#include <cstdint>

// ── drift_ring — 5-day per-cell voltage band ring (PSRAM) ─────────────────────
//
// Tracks per-cell voltage min/max per pack over up to 5 completed UTC days plus
// the current day's running accumulator. All bulk storage is EXT_RAM_BSS_ATTR
// (PSRAM) — zero net internal-DRAM increase.
//
// Storage: ~7.5 KB PSRAM total (6 DayBuckets × ~1076 B + all-time 1024 B).
// Granularity: updated every 5-min coarse boundary from HistoryTask.
// Persistence: in-RAM only (matches solar_day_ring behaviour). History rebuilds
//              over time after a reboot; "building history" shown in UI until
//              first day bucket accrues.
//
// Concurrency: single-writer (HistoryTask, Core 1) / single-reader (HttpTask,
// Core 0). No lock held; benign torn-read acceptable for display-only data,
// same model as solar_day_ring. Never DMA or ISR.

namespace app::drift_ring {

constexpr uint8_t DRIFT_MAX_PACKS    = 16;
constexpr uint8_t DRIFT_MAX_CELLS    = 16;  // BmsPackSnapshot::cell_v[] width
constexpr uint8_t DRIFT_HISTORY_DAYS = 5;   // completed days kept in ring

// Reader-facing result for one pack, filled by read_pack().
struct PackDrift {
  bool    has_history;   // true if any accumulated band data exists for this pack
  uint8_t cell_count;   // cells tracked (0 = pack never seen)
  uint8_t n_days;       // valid entries in trend_spread

  struct Cell {
    uint16_t d5min;  // 5-day rolling min (mV); 0 = no data
    uint16_t d5max;  // 5-day rolling max (mV); 0 = no data
    uint16_t ev_min; // all-time min (mV); 0 = no data
    uint16_t ev_max; // all-time max (mV); 0 = no data
  } cells[DRIFT_MAX_CELLS];

  // Max pack spread per day, oldest first. Length = n_days.
  // Indices 0..n_days-2 are completed days; index n_days-1 is today (partial).
  uint16_t trend_spread[DRIFT_HISTORY_DAYS + 1];
};

// Called from HistoryTask every 5 min at the coarse boundary.
// Silently no-ops when t_epoch == 0 (NTP not synced).
void update(const BmsSystemSnapshot& snap, uint32_t t_epoch);

// Fill PackDrift for pack at index pack_idx. Returns false when the pack has
// never produced any cell data (never came online since boot).
// Safe to call from HttpTask without a lock — display-only, benign torn-read.
bool read_pack(uint8_t pack_idx, PackDrift& out);

// Number of complete UTC days committed to the ring (0..DRIFT_HISTORY_DAYS).
uint8_t complete_days();

}  // namespace app::drift_ring
