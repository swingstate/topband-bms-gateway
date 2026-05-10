#pragma once
#include "esp_http_server.h"

namespace web {

// GET /api/health — liveness probe, no auth, always 200.
esp_err_t handle_health(httpd_req_t* req);

// POST /api/restart — return 200 immediately, then esp_restart() after 2 s.
esp_err_t handle_restart(httpd_req_t* req);

// GET /api/backup — download Config as topband-config.json attachment.
esp_err_t handle_backup(httpd_req_t* req);

// POST /api/factory_reset — body: {"confirm":true}
// Wipes WiFi credentials and admin password from NVS, then reboots.
// Next boot enters captive portal mode. Authenticated endpoint.
esp_err_t handle_factory_reset(httpd_req_t* req);

// POST /api/svc/ha/discovery/send — force-publishes all HA discovery payloads now.
// Returns 409 if MQTT is not connected.
esp_err_t handle_ha_discovery_send(httpd_req_t* req);

// POST /api/svc/ha/discovery/clear — resets HA discovery NVS marker so
// cleanup_stale() re-runs on next connect. Also triggers full discovery if connected.
esp_err_t handle_ha_discovery_clear(httpd_req_t* req);

}  // namespace web
