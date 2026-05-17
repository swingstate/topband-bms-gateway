#pragma once
#include "esp_http_server.h"

namespace web {

// GET  /api/alerts?limit=50&skip=0&min_severity=0
esp_err_t handle_alerts_get(httpd_req_t* req);

// DELETE /api/alerts — clear all (CSRF required)
esp_err_t handle_alerts_delete(httpd_req_t* req);

}  // namespace web
