#pragma once
#include <cstddef>
#include <cstdint>
#include "bms_snapshot.h"
#include "safety_state.h"
#include "bms/poller.h"
#include "can/tx.h"

// JSON serializers for MQTT publish payloads (architecture §4.7).
// All functions are host-portable (no IDF dependencies).
// Callers provide ts_ms / uptime_s from esp_timer so this file stays pure.

namespace mqtt::payloads {

// {base}/data — aggregated live state, published every 5 s.
// Returns bytes written into out (0 on error or overflow).
size_t build_data(const BmsSystemSnapshot& snap, const SafetyState& safety,
                  uint64_t ts_ms, uint32_t uptime_s,
                  char* out, size_t out_size);

// {base}/diag — diagnostic counters, published every 30 s when enabled.
size_t build_diag(const BmsSystemSnapshot& snap, const SafetyState& safety,
                  const bms::poller::PollerStats& ps, const can::tx::CanStats& cs,
                  uint64_t ts_ms, uint32_t uptime_s,
                  char* out, size_t out_size);

// {base}/cells/bms{n} — per-pack cell detail, published every 20 s at PerCell level.
size_t build_cells(const BmsPackSnapshot& pack, uint64_t ts_ms,
                   char* out, size_t out_size);

// {base}/alarm — one event per safety state transition.
size_t build_alarm_event(const SafetyState::EventEntry& evt,
                          uint64_t ts_ms, char* out, size_t out_size);

}  // namespace mqtt::payloads
