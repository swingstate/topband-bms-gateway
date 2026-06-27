#pragma once
#include "esp_http_server.h"

namespace web {

// GET /api/drift
// Returns per-cell voltage bands (5-day min/max, all-time min/max, live now)
// for every online pack. Streaming JSON, zero heap allocation.
esp_err_t handle_drift(httpd_req_t* req);

}  // namespace web
