#pragma once
#include "storage/config.h"

namespace app {
  // Runs the full boot sequence: NVS, config, queues, snapshot bus, WiFi,
  // LittleFS, HTTP server, tasks, then heartbeat loop. Never returns.
  void run_boot();

  // Read-only access to the runtime config. Safe to call from any task after boot.
  const Config& get_config();

  // Apply new_cfg: validate, save to NVS, update g_config.
  // Returns false if validation or NVS save fails (g_config is not changed).
  bool update_and_save_config(const Config& new_cfg);
}
