#pragma once
#include <cstdint>
#include <cstddef>
#include <string>
#include <vector>
#include "esp_netif.h"

// ── WiFi module ───────────────────────────────────────────────────────────────
// Manages STA (client) and AP (captive portal) modes.
//
// Normal boot path:
//   init() → start_sta(30 000) → connected? → normal HTTP server
//                              → failed?    → start_ap() → captive portal
//
// Captive-portal connect path (from HTTP handler):
//   save_creds(ssid, pass) → start_connection_async() → browser polls get_state()
//   On StaConnected: esp_restart() is scheduled automatically by the task.

namespace net::wifi {

enum class Mode : uint8_t {
  Off,            // before init()
  StaConnecting,  // esp_wifi_connect() in progress
  StaConnected,   // IP obtained
  StaFailed,      // connection timed out / credential error
  ApActive,       // open AP, no STA session
};

struct ScanResult {
  std::string ssid;
  int8_t      rssi;
  bool        secure;  // authmode != WIFI_AUTH_OPEN
};

// ── Lifecycle ─────────────────────────────────────────────────────────────────

// Initialize esp_netif + default event loop. Call once at boot before any
// start_* function.
bool init();

// Attempt STA connection using credentials stored in esp_wifi NVS.
// Blocks until connected or timeout_ms elapses.
// Returns true (Mode becomes StaConnected) or false (StaFailed).
bool start_sta(uint32_t timeout_ms = 15000);

// Start AP mode (APSTA internally so scans work while AP is active).
// SSID is auto-generated: "TopBand-Setup-XXXX" using last 4 MAC hex digits.
// Open network. Gateway IP 192.168.4.1.
// Returns the SSID string for logging.
std::string start_ap();

// Stop AP interface (call after STA connection succeeds in captive mode).
void stop_ap();

// ── Credentials ───────────────────────────────────────────────────────────────

// Persist WiFi credentials into esp_wifi NVS namespace.
bool save_creds(const char* ssid, const char* pass);

// Read stored SSID into buf. Returns false if no credentials stored.
bool load_ssid(char* buf, size_t len);

// ── Captive-portal connect (non-blocking) ─────────────────────────────────────

// Save credentials and start an async connect task.
// The task: attempts STA connection for up to timeout_ms; on success it
// schedules esp_restart(); on failure it sets Mode=StaFailed.
// Returns immediately (non-blocking from the HTTP handler's perspective).
void start_connection_async(const char* ssid, const char* pass,
                            uint32_t timeout_ms = 30000);

// ── Scan ──────────────────────────────────────────────────────────────────────

// Scan for nearby networks. Blocks up to timeout_ms (typically ~2-3 s).
// Requires WiFi to be running (AP or APSTA mode).
// Returns empty vector on error.
std::vector<ScanResult> scan(uint32_t timeout_ms = 5000);

// ── Status ────────────────────────────────────────────────────────────────────

Mode get_state();

bool is_connected();   // true if Mode == StaConnected
bool is_ap_mode();     // true if Mode == ApActive

// Fill buf with STA IPv4 address string, e.g. "192.168.1.42".
void get_ip(char* buf, size_t len);

// Return STA IP as std::string (empty if not connected).
std::string get_local_ip();

// Expose the AP netif for captdns to query the gateway IP.
esp_netif_t* get_ap_netif();

}  // namespace net::wifi
