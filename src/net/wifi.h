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
//   init() → start_sta(30000, bssid_pin, rssi_floor) → connected? → normal HTTP server
//                                                     → failed?    → start_ap() → captive portal
//
// AP-selection behaviour (always active):
//   scan_method = WIFI_ALL_CHANNEL_SCAN  — scans ALL APs of the SSID, not just first.
//   sort_method = WIFI_CONNECT_AP_BY_SIGNAL — picks the one with the best RSSI.
//   rssi_floor  — configurable dBm minimum; -127 = no floor (accept any).
//
// BSSID pin (optional override):
//   If bssid_pin is a valid MAC, the driver prefers that exact AP.
//   Safety fallback: after WIFI_BSSID_PIN_MAX_RETRY failures the pin is cleared
//   and the normal strongest-AP selection resumes automatically. A log line is
//   emitted on fallback so the operator can see exactly what happened.
//   Periodic retry: once in fallback, the NEXT post-connection reconnect cycle
//   (AP hiccup, roam, DHCP-renewal disconnect — see is_bssid_pin_active()) re-arms
//   the pin automatically, so a temporarily-unreachable preferred AP is retried
//   without waiting for a reboot. No separate timer/task; piggybacks on whatever
//   reconnect already happens.
//
// Roaming / reconnect:
//   On any post-connection disconnect the module immediately attempts to reconnect.
//   Because scan_method=ALL_CHANNEL and sort_method=BY_SIGNAL remain in the driver
//   config, the reconnect naturally picks the best available AP of the same SSID —
//   this is the "roaming" mechanism. Full 802.11r/k/v is not supported by ESP-IDF;
//   roaming here is explicit disconnect+reconnect, not seamless BSS transition.
//
// Runtime outage handling (V3.2 — see net/wifi_backoff.h):
//   Once the STA has connected successfully at least once, a subsequent loss of
//   connectivity NEVER reaches a terminal give-up state: the first reconnect
//   attempt is immediate, and every attempt after that backs off exponentially
//   (capped, see wifi_backoff.h) and retries forever until the AP is reachable
//   again. There is no runtime auto-switch to AP/captive-portal mode — that
//   fallback stays boot-only (see below) or an explicit user action from
//   Settings. This does NOT apply to the very first connection attempt after
//   boot (see start_sta() below), which stays bounded on purpose so a device
//   with no reachable AP yet still falls back to captive portal for setup.
//
// Captive-portal connect path (from HTTP handler):
//   save_creds(ssid, pass) → start_connection_async() → browser polls get_state()
//   On StaConnected: esp_restart() is scheduled automatically by the task.

namespace net::wifi {

// Maximum attempts with BSSID pin before falling back to strongest-AP.
// Exposed so the diag log entry can report it.
static constexpr int WIFI_BSSID_PIN_MAX_RETRY = 3;

enum class Mode : uint8_t {
  Off,            // before init()
  StaConnecting,  // esp_wifi_connect() attempt in flight, or its backoff wait pending
  StaConnected,   // IP obtained
  // Terminal ONLY for the bounded attempts that use it: the very first
  // connection attempt after boot (start_sta()'s blocking wait) and the
  // explicit captive-portal "test connect" (start_connection_async()). Once
  // the STA has connected successfully at least once, a runtime disconnect
  // never reaches this state again — see wifi_backoff.h and on_wifi_event()'s
  // g_had_ip branch in wifi.cpp.
  StaFailed,      // connection timed out / credential error (bounded attempts only)
  ApActive,       // open AP, no STA session
};

struct ScanResult {
  std::string ssid;
  std::string bssid;  // "xx:xx:xx:xx:xx:xx" lowercase
  int8_t      rssi;
  bool        secure;  // authmode != WIFI_AUTH_OPEN
};

// ── Lifecycle ─────────────────────────────────────────────────────────────────

// Initialize esp_netif + default event loop. Call once at boot before any
// start_* function.
bool init();

// Attempt STA connection using credentials stored in esp_wifi NVS.
// Applies WIFI_ALL_CHANNEL_SCAN + WIFI_CONNECT_AP_BY_SIGNAL in the driver config
// so every call (including the initial one) uses signal-based AP selection.
//
// bssid_pin: canonical "xx:xx:xx:xx:xx:xx" (lowercase, colons) or nullptr/empty.
//   When non-empty, the driver pins to that specific AP for up to
//   WIFI_BSSID_PIN_MAX_RETRY connection attempts, then falls back to auto-select.
//
// rssi_floor: minimum RSSI (dBm) an AP must exceed to be selected.
//   -127 = disabled (accept any RSSI; ESP-IDF default).
//
// Blocks until connected or timeout_ms elapses.
// Returns true (Mode becomes StaConnected) or false (StaFailed).
bool start_sta(uint32_t timeout_ms = 15000,
               const char* bssid_pin = nullptr,
               int8_t rssi_floor = -127);

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
// Returns empty vector on error. Results include BSSID per entry.
std::vector<ScanResult> scan(uint32_t timeout_ms = 5000);

// ── Status ────────────────────────────────────────────────────────────────────

struct IpInfo {
  char ip[24];
  char gw[24];
  char netmask[24];
  char dns[24];
};

Mode get_state();

bool is_connected();   // true if Mode == StaConnected
bool is_ap_mode();     // true if Mode == ApActive

// Fill buf with STA IPv4 address string, e.g. "192.168.1.42".
void get_ip(char* buf, size_t len);

// Return STA IP as std::string (empty if not connected).
std::string get_local_ip();

// Return connected SSID (empty if not in STA mode).
std::string get_ssid();

// Return STA RSSI in dBm. Returns 0 if not connected.
int8_t get_rssi();

// Return associated BSSID as "xx:xx:xx:xx:xx:xx" (empty if not connected).
std::string get_bssid();

// Fill buf with the mDNS hostname set during init (e.g. "topband-bms-91c4").
void get_hostname(char* buf, size_t len);

// Return full IP info (ip, gw, netmask, dns). Fields are "0.0.0.0" if unavailable.
IpInfo get_ip_info();

// Seconds since STA connected; 0 if not connected.
uint32_t connected_for_s();

// Count of STA disconnection events since boot (coexistence diagnostic).
// Rises monotonically; visible in /api/diag ble_spike.wifi_disconnects.
uint32_t get_disconnect_count();

// True while a BSSID pin is in effect (cleared on fallback or reconnect-after-connected).
bool is_bssid_pin_active();

// ── Reconnect / outage diagnostics (V3.2) ────────────────────────────────────
// Surfaced on the Diagnostics page so a live outage is immediately visible
// there instead of leaving no trace anywhere (the original diagnostic gap
// that let the permanent-give-up bug go unnoticed for months).

// Connect attempts made during the CURRENT outage (0 if not currently in one).
// Resets to 0 on every successful connect.
uint32_t get_reconnect_attempts();

// Lifetime connect attempts since boot (never resets). Includes the initial
// boot-time attempt(s), all runtime reconnects, and BSSID-pin retries.
uint32_t get_reconnect_attempts_total();

// Backoff interval (ms) currently being waited before the next reconnect
// attempt. 0 when connected, or when an attempt is immediately in flight
// (the first attempt after a disconnect is always immediate — see wifi.h
// runtime-outage comment above).
uint32_t get_reconnect_backoff_ms();

// Seconds since the current outage began; 0 if currently connected (no
// active outage). Doubles as "time since last successful connection" while
// an outage is in progress.
uint32_t get_outage_duration_s();

// Duration (s) of the most recently COMPLETED outage this boot; 0 if none
// has occurred yet. Lets Diagnostics show "the last outage lasted N s" even
// after recovery, without needing to catch it live.
uint32_t get_last_outage_duration_s();

// Expose the AP netif for captdns to query the gateway IP.
esp_netif_t* get_ap_netif();

}  // namespace net::wifi
