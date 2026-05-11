#pragma once
#include <cstdint>

// Forward-declare to avoid pulling in Config everywhere.
struct Config;

namespace net::ntp {

// Start SNTP sync using server and timezone from cfg.
// Non-blocking — sync happens asynchronously. Returns immediately.
bool start(const Config& cfg);

void stop();

// True once the first SNTP sync has completed.
bool is_synced();

// Current unix timestamp in seconds. Returns 0 if not yet synced.
uint32_t now_unix_s();

// Call when user changes NTP server or timezone in settings.
// Re-applies setenv("TZ", ...) and restarts SNTP client.
void reconfigure(const Config& cfg);

}  // namespace net::ntp
