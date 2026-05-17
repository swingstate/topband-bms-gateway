#pragma once
#include "esp_http_server.h"

namespace web {

// GET /api/history?series=<name>[,<name2>]&tier=fine|coarse|both
// Returns fine or coarse ring data as JSON per architecture §6.5.
esp_err_t handle_history(httpd_req_t* req);

// GET /api/history/export.csv?series=all&tier=both
// CSV export of full history for analytics.
esp_err_t handle_history_export(httpd_req_t* req);

}  // namespace web
