#pragma once
#include "bms_snapshot.h"
#include <cstdint>

// ── drift_ring — 5-day per-cell voltage band ring (PSRAM) ─────────────────────
//
// Tracks per-cell voltage min/max per pack over up to 5 completed UTC days plus
// the current day's running accumulator.  Sampling is gated by SoC region:
//   High (top-of-charge)  : SOC >= SOC_HIGH_THRESHOLD (≥ 90 %)
//   Low  (bottom-of-discharge): SOC <= SOC_LOW_THRESHOLD  (≤ 15 %)
// The flat LiFePO4 mid-SoC range is deliberately ignored — spread values there
// are ~0–10 mV and carry no diagnostic information.
//
// All bulk storage is EXT_RAM_BSS_ATTR (PSRAM) — zero net internal-DRAM increase.
//
// Storage: ~20 KB PSRAM total.
//   DayBucket: 4 + 3×(16×16×4) + 2×(16×2) + 16 = 3156 B
//   5 completed-day ring + 1 today accumulator = 6 × 3156 = 18936 B
//   All-time bands: 1024 B
//
// Granularity: updated every 5-min coarse boundary from HistoryTask.
// Persistence: completed-day ring + all-time bands are saved to LittleFS on
// every day commit and restored at boot by load(). Today's partial accumulator
// is NOT persisted; it rebuilds within the current day. Without persistence a
// reboot (e.g. an OTA flash) would reset the trend to "building" for days
// (review Part 2.1).
//
// Concurrency: single-writer (HistoryTask, Core 1) / single-reader (HttpTask,
// Core 0). No lock held; benign torn-read acceptable for display-only data.

namespace app::drift_ring {

constexpr uint8_t DRIFT_MAX_PACKS    = 16;
constexpr uint8_t DRIFT_MAX_CELLS    = 16;  // BmsPackSnapshot::cell_v[] width
constexpr uint8_t DRIFT_HISTORY_DAYS = 5;   // completed days kept in ring

// SoC gate thresholds (integer %, matching BmsPackSnapshot::soc).
constexpr uint8_t SOC_HIGH_THRESHOLD = 90;  // top-of-charge  (≥ 90 %)
constexpr uint8_t SOC_LOW_THRESHOLD  = 15;  // bottom-of-discharge (≤ 15 %)

// Reader-facing result for one pack, filled by read_pack().
struct PackDrift {
  bool    has_history;   // true if any accumulated band data exists for this pack
  bool    has_toc;       // true if any top-of-charge samples exist in window
  bool    has_bod;       // true if any bottom-of-discharge samples exist in window
  uint8_t cell_count;   // cells tracked (0 = pack never seen)
  uint8_t n_days;       // valid entries in trend_spread

  uint16_t toc_spread;   // max (cell_high_max − cell_high_min) across 5-day window (mV)
  uint16_t bod_spread;   // max (cell_low_max  − cell_low_min)  across 5-day window (mV)

  // Cell that hits full-charge ceiling first (highest toc_max).
  uint8_t  first_full_idx;  // 0-based cell index
  uint16_t first_full_mv;   // its 5-day toc_max (mV); 0 if !has_toc

  // Cell that empties first at bottom-of-discharge (lowest bod_min).
  uint8_t  first_empty_idx;
  uint16_t first_empty_mv;  // its 5-day bod_min (mV); 0 if !has_bod

  // Repetition detection (review Part 2.3): a first-full cell is only a
  // pattern when the SAME cell wins the per-day argmax repeatedly. Computed
  // at read time from the per-day ToC/BoD bands; no extra storage.
  // *_days_total = days in window (incl. today) with region data;
  // *_mode_idx   = most frequent winner; *_days_won = its win count.
  uint8_t ff_mode_idx;
  uint8_t ff_days_won;
  uint8_t ff_days_total;
  uint8_t fe_mode_idx;
  uint8_t fe_days_won;
  uint8_t fe_days_total;

  struct Cell {
    uint16_t d5min;   // 5-day union of high+low region min (mV); 0 = no data
    uint16_t d5max;   // 5-day union of high+low region max (mV); 0 = no data
    uint16_t ev_min;  // all-time min (mV, unconditional); 0 = no data
    uint16_t ev_max;  // all-time max (mV, unconditional); 0 = no data
    uint16_t toc_min; // 5-day min while at top-of-charge (mV); 0 = no data
    uint16_t toc_max; // 5-day max while at top-of-charge (mV); 0 = no data
    uint16_t bod_min; // 5-day min while at bottom-of-discharge (mV); 0 = no data
    uint16_t bod_max; // 5-day max while at bottom-of-discharge (mV); 0 = no data
  } cells[DRIFT_MAX_CELLS];

  // Max ToC spread per day, oldest first; length = n_days.
  // Indices 0..n_days-2 are completed days; index n_days-1 is today (partial).
  // trend_day carries the UTC day number (epoch/86400) of each entry so the
  // rate can honor real day spacing (gaps from cloudy days or downtime).
  uint16_t trend_spread[DRIFT_HISTORY_DAYS + 1];
  uint32_t trend_day[DRIFT_HISTORY_DAYS + 1];
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

// Restore the completed-day ring + all-time bands from LittleFS.
// Call once at boot after lfs::init(); no-op when no file exists.
void load();

// Persist the completed-day ring + all-time bands to LittleFS (atomic
// tmp+rename). Called internally on every day commit; exposed for tests.
void save();

}  // namespace app::drift_ring
