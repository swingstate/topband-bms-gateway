#pragma once
#include "esp_http_server.h"

namespace web {

// GET /api/health — liveness probe, no auth, always 200.
esp_err_t handle_health(httpd_req_t* req);

// POST /api/restart — return 200 immediately, then esp_restart() after 2 s.
esp_err_t handle_restart(httpd_req_t* req);

// GET /api/backup — download Config as topband-config.json attachment.
esp_err_t handle_backup(httpd_req_t* req);

}  // namespace web
