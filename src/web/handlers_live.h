#pragma once
#include "esp_http_server.h"

namespace web {

// GET /api/live — full live snapshot, safety state, and stats.
esp_err_t handle_live(httpd_req_t* req);

// GET /api/bms/:id — per-pack detail (Phase H; returns 501 for now).
esp_err_t handle_bms_id(httpd_req_t* req);

}  // namespace web
