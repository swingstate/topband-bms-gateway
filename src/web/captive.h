#pragma once
#include "esp_http_server.h"

// ── Captive portal HTTP server ────────────────────────────────────────────────
// Separate from the main httpd (web::server). Started only in AP mode.
// Serves the WiFi setup UI and API endpoints; 302-redirects everything else to
// /setup.html so OS captive-portal detection triggers the built-in browser.
//
// Architecture §8.9, D4.1.

namespace web::captive {

// Start the captive portal HTTP server on port 80.
// Returns true on success.
bool start();

// Stop and release the captive portal HTTP server.
void stop();

}  // namespace web::captive
