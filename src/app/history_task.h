#pragma once

// ── HistoryTask — 10-second sample accumulator (Core 1, priority 1) ───────────
// Every 10 s : reads latest snapshot, appends one HistoryFinePoint.
// Every 5 min: aggregates 30 fine points → one HistoryCoarsePoint, flushes to LFS.

namespace app::history_task {

// Create and start the FreeRTOS task. Must be called after:
//   - storage::history_store::init()
//   - storage::energy_store::init()
//   - bus::snapshot_bus::init()
// Returns false on task-creation failure.
bool start();

}  // namespace app::history_task
