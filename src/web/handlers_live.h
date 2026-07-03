#pragma once
#include "esp_http_server.h"
#include <cstdint>

namespace web {

// GET /api/live — full live snapshot, safety state, and stats.
esp_err_t handle_live(httpd_req_t* req);

// GET /api/bms/:id — per-pack detail (Phase H; returns 501 for now).
esp_err_t handle_bms_id(httpd_req_t* req);

// Coexistence diagnostic: /api/live handler wall-clock latency.
// Readable by handlers_diag for the ble_spike{} section.
uint32_t live_handler_last_ms();
uint32_t live_handler_max_ms();

}  // namespace web
