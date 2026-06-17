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

}  // namespace sources::ble_scanner
