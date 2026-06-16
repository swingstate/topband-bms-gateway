#pragma once
#include <cstdint>
#include "storage/config.h"
#include "safety_state.h"

namespace can::tx {

// Initialise the TWAI driver using pin map from cfg.
// Must be called once before can_tx_if_due(). Returns false on driver failure.
bool init(const Config& cfg);

// Called from ControlTask every 50 ms tick (architecture §3.1 Phase C).
// Internally checks whether 1000 ms heartbeat is due, or whether alarm_flags
// has transitioned since the last send (express send on rising edge).
// protocol is read live by the caller (poller.cpp via app::get_config()) so
// switching protocol takes effect on the next CAN cycle without reboot.
// Returns true if frames were transmitted this tick.
bool can_tx_if_due(const SafetyState& current, uint32_t now_ms,
                   Config::CanProtocol protocol = Config::CanProtocol::Victron);

// Low-level single-frame enqueue. Called by victron::send_all_victron.
// In IDF builds: calls twai_transmit with 10 ms timeout.
// In NATIVE_BUILD: stub that counts sends for test verification.
// Returns true on success.
bool enqueue(uint32_t id, const uint8_t data[8]);

// Diagnostic counters — updated by enqueue() and can_tx_if_due().
struct CanStats {
  uint64_t tx_ok;
  uint64_t tx_fail;
  uint32_t tx_fail_streak_max;
  uint32_t bus_off_count;
  uint32_t driver_restart_count;
  uint32_t express_sends;    // out-of-cadence sends triggered by alarm_flags transition
  uint32_t heartbeats;       // regular 1 s sends
};
void get_stats(CanStats& out);

#ifdef NATIVE_BUILD
// Test helpers available only in host builds.
// reset() clears all state (last_tx_ms sentinel, alarm tracking, counters).
void test_reset();
#endif

}  // namespace can::tx
