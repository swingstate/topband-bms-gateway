#pragma once
#include <cstdint>

// Minimal captive-portal DNS server. Architecture §8.9, D4.1.
//
// Binds to UDP port 53 in a background task and answers EVERY A-record query
// with a fixed IP address (the AP gateway, 192.168.4.1). This triggers the
// OS captive-portal detection flow on iOS, macOS, Android, and Windows:
// each OS probes well-known domains; our DNS redirects them to the gateway;
// the OS detects a non-standard response and opens the captive-portal browser.
//
// HTTPS probes cannot be hijacked (TLS certificate mismatch). This is
// expected and documented — HTTP probes are sufficient for captive detection
// on all major OSes.

namespace net::captdns {

// Start the DNS server task. ip_to_return must be in NETWORK byte order.
// Typically inet_addr("192.168.4.1") for the AP gateway.
// Returns true if the task started successfully.
bool start(uint32_t ip_to_return);

// Signal the DNS task to stop and wait up to 1.5 s for it to exit.
void stop();

}  // namespace net::captdns
