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

// POST /api/restore — body: {"backup":{...},"include_hardware":bool}
// Validates backup JSON, merges onto current config per chosen scope, saves, reboots.
esp_err_t handle_restore(httpd_req_t* req);

// POST /api/mqtt/test — body: {"host","port","user","pass","base_topic"}
// Starts a throwaway MQTT client against the unsaved form values (does NOT
// touch the live MQTT connection). Returns immediately; poll GET for result.
esp_err_t handle_mqtt_test_post(httpd_req_t* req);

// GET /api/mqtt/test — returns {"status":"idle|running|ok|failed","stage":"...","message":"..."}.
esp_err_t handle_mqtt_test_get(httpd_req_t* req);

// GET /api/wifi/status — returns ssid, rssi, ip, gw, netmask, dns, hostname, connected_for_s.
esp_err_t handle_wifi_status_get(httpd_req_t* req);

// GET /api/wifi/scan — triggers a WiFi scan and returns the list of nearby networks.
esp_err_t handle_wifi_scan_get(httpd_req_t* req);

// POST /api/wifi/configure — body: {"ssid":"...","password":"..."}
// Saves credentials and starts async connect; on success the gateway reboots.
esp_err_t handle_wifi_configure_post(httpd_req_t* req);

}  // namespace web
