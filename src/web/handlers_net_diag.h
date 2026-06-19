#pragma once
#include "esp_http_server.h"

namespace web {

// POST /api/net/self-test — trigger the 5-stage network diagnostic.
// Returns 409 if already running.
esp_err_t handle_net_diag_post(httpd_req_t* req);

// GET /api/net/self-test — return current (or last) diagnostic results as JSON.
esp_err_t handle_net_diag_get(httpd_req_t* req);

}  // namespace web
