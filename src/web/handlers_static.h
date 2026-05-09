#pragma once
#include "esp_http_server.h"

namespace web {

// Catch-all: serves /lfs/ui/* files from LittleFS. Maps URI → filesystem path.
// "/" and "/index.html" → /lfs/ui/index.html. 404 for unknown paths.
esp_err_t handle_static(httpd_req_t* req);

// GET /wifi-setup — simple WiFi credential entry form (AP mode only).
esp_err_t handle_wifi_setup_page(httpd_req_t* req);

}  // namespace web
