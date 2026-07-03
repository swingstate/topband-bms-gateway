#pragma once
#include "aggregator.h"
#include "bms_source.h"
#include "shunt_source.h"
#include "mppt_source.h"

// ── Source registry ────────────────────────────────────────────────────────────
//
// Singletons owned by boot.cpp; accessible read-only to any module that needs
// live source readings or diagnostic snapshots.
//
// Call order: sources::init_registry() during boot (after Config is loaded),
// then sources::ble_scanner::start() if BLE is enabled.
// All getters return nullptr before init_registry() is called.

namespace sources {

// Initialise registry singletons. Call once from boot.cpp before tasks start.
void init_registry(const Config& cfg);

// Non-owning accessors (lifetime: device lifetime).
BmsSource*   bms_source();
ShuntSource* shunt_source();
MpptSource*  mppt_source();
Aggregator*  aggregator();

}  // namespace sources
