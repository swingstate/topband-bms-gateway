#pragma once
#include "esp_http_server.h"

namespace web {

// GET /api/diag — streaming JSON with runtime visibility data.
esp_err_t handle_diag(httpd_req_t* req);

}  // namespace web
