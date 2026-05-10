#pragma once
#include <cstdint>
#include <cstddef>

// ── WiFi module ───────────────────────────────────────────────────────────────
// STA mode: connect using credentials persisted via esp_wifi_set_config (NVS).
// AP fallback: open AP "TopBand-Setup-XXXX" at 192.168.4.1 when no creds or
// connection fails. Phase G adds captive-portal DNS hijack on top of this.

namespace net::wifi {

// Initialize esp_netif + default event loop. Call once before start_sta/ap.
bool init();

// Attempt STA connection using stored credentials. Blocks until connected or
// timeout_ms elapses. Returns true if connected.
bool start_sta(uint32_t timeout_ms = 15000);

// Start AP mode. SSID is "TopBand-Setup-XXXX" (last 4 hex digits of MAC).
// Open network (no password). Gateway IP is 192.168.4.1.
void start_ap();

// Persist WiFi credentials via esp_wifi_set_config (→ NVS net80211 namespace).
// Returns true on success.
bool save_creds(const char* ssid, const char* pass);

// Read stored SSID into buf. Returns false if no credentials stored.
bool load_ssid(char* buf, size_t len);

bool is_connected();

// Fill buf with the station IPv4 address string, e.g. "192.168.1.42".
// Returns "0.0.0.0" if not connected.
void get_ip(char* buf, size_t len);

// True if currently in AP (setup) mode.
bool is_ap_mode();

}  // namespace net::wifi
