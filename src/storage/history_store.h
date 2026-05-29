#pragma once
#include "bus/types.h"
#include <cstddef>

// ── history_store — dual-rate ring files in LittleFS ──────────────────────────
//
// Fine ring  : /lfs/history/fine.bin   — 10s samples, 2h window, 720 pts
// Coarse ring: /lfs/history/coarse.bin — 5-min samples, 7d window, 2016 pts
//
// Write path (called from HistoryTask only — single writer):
//   Every 10s : append_fine()
//   Every 5min: append_coarse() + flush()  (flush updates the on-disk header)
//
// Read path (called from HTTP handler — single reader from HttpTask):
//   read_fine() / read_coarse() — fills caller-allocated output arrays.

namespace storage::history_store {

// Mount check + create directories/files if missing.
// Must be called once from boot after LittleFS is mounted.
bool init();

// Append one fine sample. Wraps the ring when full (oldest entry overwritten).
// thread-safe within single-writer invariant (HistoryTask only).
bool append_fine(const HistoryFinePoint& pt);

// Append one coarse sample (downsampled from 30 fine samples).
bool append_coarse(const HistoryCoarsePoint& pt);

// Flush in-memory ring state to LittleFS (atomic via tmp+rename).
// Called every 5 min from HistoryTask after each coarse-downsample cycle.
void flush();

// Read the N most recent entries from the fine ring, oldest-first.
// Returns count actually copied (≤ max_count, ≤ ring capacity).
size_t read_fine(HistoryFinePoint* out, size_t max_count);

// Read the N most recent entries from the coarse ring, oldest-first.
size_t read_coarse(HistoryCoarsePoint* out, size_t max_count);

// Set the epoch_base for the fine ring (called once on first NTP-synced sample).
void set_fine_epoch_base(uint32_t epoch_s);

// Epoch of the oldest fine entry currently in the ring (0 = ring empty/no NTP).
uint32_t fine_epoch_base();

// Epoch of the oldest coarse entry currently in the ring (0 = ring empty).
uint32_t coarse_epoch_base();

// Number of valid entries currently in each ring.
uint32_t fine_count();
uint32_t coarse_count();

// ── Cell-drift companion rings (PSRAM-backed, zero DRAM BSS cost) ─────────────
// Drift is stored in separate PSRAM-heap arrays parallel to the main rings.
// Not persisted to LittleFS — lost on reboot (acceptable for a monitoring chart).
// Call record_*_drift_mv() immediately after the corresponding append_*() call.
void     record_fine_drift_mv(uint16_t mv);
void     record_coarse_drift_mv(uint16_t mv);
// Returns the drift value (in mV) for the sample at read-order index `idx`
// (0 = oldest in the window, same ordering as read_fine/read_coarse output).
// Returns 0 if drift data is unavailable or the index is out of range.
uint16_t read_fine_drift_at(size_t idx);
uint16_t read_coarse_drift_at(size_t idx);

}  // namespace storage::history_store
