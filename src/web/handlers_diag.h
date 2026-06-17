#pragma once
#include "esp_http_server.h"

namespace web {

// GET /api/diag — streaming JSON with runtime visibility data.
esp_err_t handle_diag(httpd_req_t* req);

// GET /api/diag/coredump.bin — returns raw coredump binary if present.
// 404 if no valid coredump exists. Per Finding 9.
esp_err_t handle_diag_coredump(httpd_req_t* req);

// POST /api/diag/tls-burst?n=<1..10>
// Dev-only TLS burst trigger for V3.1 Phase A gate-hardening.
// Only compiled and registered when BLE_SPIKE_DEV_BURST=1 (platformio.ini spike builds).
// Fires N back-to-back notify::send() calls via the real TLS path.
// Production-inert: endpoint not registered when the flag is off.
#if BLE_SPIKE_DEV_BURST
esp_err_t handle_diag_tls_burst(httpd_req_t* req);
#endif

}  // namespace web
