#pragma once
#include "esp_http_server.h"

namespace web {

// GET /api/diag — streaming JSON with runtime visibility data.
esp_err_t handle_diag(httpd_req_t* req);

// GET /api/diag/coredump.bin — returns raw coredump binary if present.
// 404 if no valid coredump exists. Per Finding 9.
esp_err_t handle_diag_coredump(httpd_req_t* req);

}  // namespace web
