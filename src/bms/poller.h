#pragma once
#include <cstdint>
#include "storage/config.h"

// ── BMS Poller / ControlTask ────────────────────────────────────────────────
// ControlTask runs pinned to Core 0 at priority 5. Each iteration:
//   Phase A (every 3 s) — poll all configured packs for analog data (0x42)
//   Phase B (every 3 s, round-robin one pack) — poll alarm (0x44) or sysparam (0x47)
//   Safety stub (Phase D will replace)
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
  uint32_t cycle_max_ms;
  uint32_t cycle_avg_ms;
};

// Thread-safe stats snapshot via portENTER_CRITICAL (< 1 µs, no I/O).
void get_stats(PollerStats& out);

}  // namespace bms::poller
