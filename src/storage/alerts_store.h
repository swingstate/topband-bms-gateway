#pragma once
#include <cstdint>
#include <cstddef>
#include "bus/queues.h"  // AlertEntry

// LittleFS-backed ring for persistent alert storage (architecture §4.8, §5.6).
//
// Ring file: /lfs/alerts/ring.bin
// Format: header (16 B) + 200 × AlertEntry (132 B) = 26 416 B
// In-RAM mirror is the live source; LittleFS is the persistence layer.
// Persist cadence: every 5 min if dirty (HousekeepingTask), plus on shutdown.
// Atomic write via tmp+rename to guard against partial-write corruption.

namespace storage::alerts_store {

static constexpr size_t RING_CAPACITY = 200;

// Load from LittleFS ring file into RAM. Creates fresh file if missing/corrupt.
// Call once during boot after lfs::init().
bool init();

// Append alert to in-RAM ring and mark dirty. O(1).
bool append(const AlertEntry& a);

// Read up to max_count alerts from the ring into out[], starting from the
// newest and working backwards. skip_from_newest allows pagination.
// Returns count of entries written.
size_t read(AlertEntry* out, size_t max_count, size_t skip_from_newest = 0);

// Delete all alerts in RAM and flush an empty ring to LittleFS.
void clear_all();

// Flush dirty in-RAM ring to LittleFS. Returns false on I/O error.
// No-op if ring is not dirty.
bool persist();

// Number of alerts currently stored in RAM.
size_t stored_count();

}  // namespace storage::alerts_store
