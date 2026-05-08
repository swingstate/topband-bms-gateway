#pragma once
#include "bms_snapshot.h"
#include "bms/protocol.h"

// ── Snapshot fill helpers ───────────────────────────────────────────────────
// Converts parsed protocol structs into BmsPackSnapshot / BmsSystemSnapshot
// fields. Pure C++17; no IDF or Arduino dependencies.

namespace bms {

// Zero-initialise a BmsPackSnapshot and mark it offline. Call at boot for all
// pack slots, and whenever a pack transitions to the offline state.
void init_pack_snapshot_offline(BmsPackSnapshot& pack, uint8_t bms_id);

// Fill analog fields from a parsed 0x42 response. Computes derived fields
// (cell_min/max/avg, cell_drift, temp_max/avg). Applies the 120 s
// hold-last-value workaround for pack_current (architecture §5.3, V2.66 H1).
// Sets online = true, updates last_seen_ms.
void fill_from_analog(const bms::protocol::tb_analog_values_fixed_point& parsed,
                      uint32_t now_ms,
                      BmsPackSnapshot& out);

// Merge alarm data from a parsed 0x44 response. Only alarm fields are updated;
// analog data is left untouched. Updates last_alarm_ms.
void fill_from_alarm(const bms::protocol::tb_alarm_info& parsed,
                     uint32_t now_ms,
                     BmsPackSnapshot& inout);

// Merge system parameter data from a parsed 0x47 response. Only sysparam_*
// fields are updated; analog data is left untouched. Sets sysparam_valid = true,
// updates last_sysparam_ms.
void fill_from_sysparam(const bms::protocol::tb_system_parameter& parsed,
                        uint32_t now_ms,
                        BmsPackSnapshot& inout);

// Transition a pack to offline if last_seen_ms has aged beyond the threshold.
// Returns true if the pack just changed from online to offline (rising edge).
bool decay_online_status(BmsPackSnapshot& inout, uint32_t now_ms,
                         uint32_t offline_threshold_ms);

// Recount online packs across all 16 slots and update
// sys.pack_count_online. Call once per cycle after filling all packs.
void update_system_aggregates(BmsSystemSnapshot& sys);

}  // namespace bms
