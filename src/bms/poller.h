#pragma once
#include <cstdint>
#include "storage/config.h"
#include "safety_state.h"

// ── BMS Poller / ControlTask ────────────────────────────────────────────────
// ControlTask runs pinned to Core 0 at priority 5. Each iteration:
//   Phase A (every 3 s) — poll all configured packs for analog data (0x42)
//   Phase B (every 3 s, round-robin one pack) — poll alarm (0x44) or sysparam (0x47)
//   Phase D — safety::runSafety() + safety state commit
//   CAN TX stub (Phase E will replace)
//   Snapshot publish via bus::snapshot_bus

namespace bms::poller {

constexpr uint32_t POLL_TICK_MS              = 50;        // ControlTask base tick
constexpr uint32_t ANALOG_POLL_PERIOD_MS     = 3000;      // full analog cycle period
constexpr uint32_t ALARM_SYSPARAM_PERIOD_MS  = 3000;      // round-robin cadence
constexpr uint32_t OFFLINE_THRESHOLD_MS      = 10000;     // 3+ missed cycles → offline
constexpr uint32_t SYSPARAM_FRESHNESS_MS     = 5 * 60 * 1000; // sysparam re-poll interval
constexpr uint32_t RS485_FRAME_TIMEOUT_MS    = 500;       // total RX wait per request
constexpr uint32_t RS485_INTER_BYTE_TIMEOUT_MS = 50;      // inactivity = frame complete

// Spawn ControlTask. Must be called after bus::snapshot_bus::init().
// Returns false on FreeRTOS task creation failure.
bool start(const Config& cfg);

// Runtime statistics — updated by ControlTask, read by heartbeat / HTTP.
struct PollerStats {
  uint32_t cycles_completed;
  uint32_t analog_polls_attempted;
  uint32_t analog_polls_ok;
  uint32_t analog_polls_timeout;
  uint32_t analog_polls_parse_err;
  uint32_t alarm_polls_ok;
  uint32_t alarm_polls_err;
  uint32_t sysparam_polls_ok;
  uint32_t sysparam_polls_err;
  // Frames with a valid checksum but a responder address that does not match
  // the polled pack (late reply from a previous poll, bus collision). Such
  // frames are discarded, never attributed to the wrong pack (review F1).
  uint32_t wrong_addr;
  uint32_t cycle_max_ms;
  uint32_t cycle_avg_ms;

  // Per-pack RS485 communication counters (indexed by BMS address 0..15).
  struct PackCommStats {
    uint32_t polls;
    uint32_t ok;
    uint32_t timeouts;
    uint32_t errors;
  } pack[16];
};

// Thread-safe stats snapshot via portENTER_CRITICAL (< 1 µs, no I/O).
void get_stats(PollerStats& out);

// Thread-safe safety state snapshot. Returns false if safety has not yet run
// (i.e. before the first ControlTask cycle completes). Phase E consumers
// (CAN TX, MQTT) call this to read the latest SafetyState.
bool read_safety_state(SafetyState& out);

}  // namespace bms::poller
