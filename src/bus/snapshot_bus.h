#pragma once
#include "bms_snapshot.h"

// ── Snapshot bus — lock-free seqlock double-buffer ──────────────────────────
// Single producer (ControlTask on Core 0), multiple readers (any Core 1 task).
// Two BmsSystemSnapshot slots live in PSRAM; an atomic index selects which slot
// readers observe. Readers retry if they catch the producer mid-publish.
//
// Producer protocol:
//   sys = begin_publish();    // get write pointer to inactive slot
//   // ... fill sys->pack[i], update fields ...
//   publish();                // fence, swap active slot, mark stable
//
// Consumer protocol:
//   BmsSystemSnapshot snap;
//   if (read(snap)) { /* use snap */ }
//
// INVARIANT: begin_publish / publish must only be called from a single task.
// Calling begin_publish concurrently with itself is a design bug.

namespace bus::snapshot_bus {

// One-time init. Allocates two BmsSystemSnapshot slots in PSRAM (device) or
// heap (NATIVE_BUILD). Returns false on allocation failure; caller must halt.
bool init();

// Producer — called only from ControlTask on Core 0.
// Returns a writable pointer to the inactive slot. The caller fills it, then
// calls publish(). The slot's seq field is set to an odd value (write marker).
BmsSystemSnapshot* begin_publish();

// Producer — completes the publish. Issues memory fence, swaps active_slot,
// then sets the slot's seq to an even value (stable marker). Fast: no I/O,
// just atomic ops and a fence.
void publish();

// Consumer — copies the current active snapshot into out.
// Returns false if no publish has occurred yet (total_publishes == 0) or if
// the seqlock retry limit (10 attempts) is exceeded.
bool read(BmsSystemSnapshot& out);

// Diagnostic counters (RAM-only, approximate — no lock on read path).
uint64_t total_publishes();
uint64_t total_reads();
uint64_t total_read_retries();

}  // namespace bus::snapshot_bus
