#pragma once
#include "esp_http_server.h"

namespace web {

// GET /api/config — current Config as JSON (sensitive fields redacted).
esp_err_t handle_config_get(httpd_req_t* req);

// POST /api/config — receive full Config JSON, validate, persist, return 200.
esp_err_t handle_config_post(httpd_req_t* req);

// POST /api/wifi — receive {ssid, pass} JSON, persist WiFi creds, schedule restart.
esp_err_t handle_wifi_post(httpd_req_t* req);

}  // namespace web
