#pragma once
#include "storage/config.h"
#include "shunt_source.h"
#include "mppt_source.h"
#include <cstdint>

// ── BLE Scanner (V3.1 Victron BLE spike) ─────────────────────────────────────
//
// Starts a NimBLE passive scan, watches for Victron GXVE manufacturer-specific
// advertisements, decrypts them (AES-128-CTR via mbedTLS), and pushes decoded
// values to ShuntSource / MpptSource.
//
// Stack choice: NimBLE (not Bluedroid). RAM cost when active:
//   NimBLE   ~80 KB internal heap (controller + host)
//   Bluedroid ~170 KB internal heap
// This difference is load-bearing for the coexistence gate.
//
// SAFETY: start() must be called AFTER the ControlTask (BMS poller) is running.
// A BLE init failure must NEVER prevent safety/CAN path from functioning.
// The caller (boot.cpp) gates the call on ble_shunt_enabled || ble_mppt_enabled.
//
// BUILD NOTE: all implementation code is guarded by CONFIG_BT_NIMBLE_ENABLED.
// Without a clean build after adding CONFIG_BT_NIMBLE_ENABLED=y to
// sdkconfig.defaults, all public functions are safe no-ops.

namespace sources::ble_scanner {

// Initialise and start the NimBLE scan task.
// shunt and mppt must remain valid for the device lifetime.
// Returns true on success, false if NimBLE init fails (BLE stays off; caller logs).
bool start(const Config& cfg, ShuntSource* shunt, MpptSource* mppt);

// True if the NimBLE stack is currently initialised and scanning.
bool is_active();

// BLE host stack identifier for Diag page.
// Returns "nimble", "bluedroid", or "none".
const char* stack_name();

// Pause the BLE passive scan for the duration of a TLS handshake so NimBLE
// buffers and TLS buffers never need large contiguous DRAM simultaneously.
// No-op if BLE was never started (ble_shunt_enabled and ble_mppt_enabled both
// false). Always call resume_scan() after, even on error paths — never let
// a TLS failure leave the scan permanently stopped.
void pause_scan();

// Restart the passive scan after a TLS handshake completes.
// No-op if BLE was never started, or if NimBLE reset during the pause
// (the on_sync callback will restart scanning automatically in that case).
void resume_scan();

// Total BLE_GAP_EVENT_DISC events received (all devices, not only Victron).
uint32_t gap_event_count();

// Total advertisements with Victron company ID (0x02E1) seen since boot.
// Non-zero means at least one Victron device is in range and advertising.
uint32_t victron_adv_count();

// Total advertisements with Victron record type 0x01 (MPPT/Solar Charger).
// If > 0 but MPPT values stay at 0, the MAC address in config does not match.
uint32_t mppt_adv_count();

// ── Per-advertisement debug state ─────────────────────────────────────────────
// Exposes the filter funnel so a non-recognising MPPT can be diagnosed without
// serial access.  Returned via get_adv_debug(); consumed by /api/diag ble_debug.

struct AdvDebugEntry {
  char    mac_str[18];     // canonical "aa:bb:cc:dd:ee:ff" (BLE addr byte-reversed)
  int8_t  rssi;
  uint8_t record_type;     // md[2]: 0x01=MPPT, 0x02=SmartShunt, other=unknown
  bool    mac_match;       // matched configured MPPT MAC after byte reversal
  bool    record_type_ok;  // record_type == 0x01
  bool    decrypt_ok;      // AES-CTR succeeded (n/a unless mac_match && record_type_ok)
  bool    valid;           // slot is populated
};

struct AdvDebugState {
  AdvDebugEntry entries[8];
  uint8_t       count;             // number of valid entries in entries[] (0-8)
  uint32_t      victron_total;     // Victron company ID ads received
  uint32_t      mppt_type_match;   // Victron ads with record_type == 0x01
  uint32_t      mppt_mac_match;    // type 0x01 ads where MAC matched configured target
  uint32_t      mppt_decrypt_ok;   // type 0x01 + MAC match + AES decrypt ok
  char          configured_mac[18]; // MPPT MAC as stored internally for comparison
  bool          mppt_mac_valid;    // configured MAC parsed successfully at startup
};

// Fill out with a diagnostic snapshot of the last ≤8 Victron advertisements and
// per-stage filter counters.  Thread-safe for diagnostic purposes (torn reads ok).
void get_adv_debug(AdvDebugState& out);

}  // namespace sources::ble_scanner
