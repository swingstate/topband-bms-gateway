#pragma once
#include "esp_http_server.h"

// ── OTA firmware-update handlers ─────────────────────────────────────────────
// Architecture §4.6, §8.5.
//
// POST /api/ota/upload   — auth-required (via reg_auth wrapper in server.cpp)
//   Content-Type: application/octet-stream
//   X-Firmware-SHA256: <64-char hex SHA-256 of the binary>
//   Body: firmware .bin bytes
//
// GET /api/ota/status    — auth-required
//   Returns current OTA partition state + self-test progress.

namespace web::handlers_ota {

esp_err_t handler_ota_upload(httpd_req_t* req);
esp_err_t handler_ota_status(httpd_req_t* req);

}  // namespace web::handlers_ota
