#pragma once
#include "storage/config.h"

// ── HTTP server ───────────────────────────────────────────────────────────────
// Wraps esp_http_server. Registers all Phase F routes. Disconnect-detection
// contract: every chunked send checks the return value of httpd_resp_send_chunk;
// on ESP_ERR_HTTPD_RESP_SEND the handler returns immediately.

namespace web {

// Start httpd with the Phase F route set. Call after WiFi is up.
// Returns false if httpd_start fails.
bool start_httpd(const Config& cfg);

// Stop httpd gracefully.
void stop_httpd();

}  // namespace web
