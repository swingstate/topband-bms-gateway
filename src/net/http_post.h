#pragma once
#include <cstdint>

// ── Shared outbound HTTPS POST helper ────────────────────────────────────────
//
// First outbound TLS client on this device.  Uses esp_http_client over SSL,
// validating against the ESP-IDF certificate bundle (no per-service CA needed).
// Call sites MUST run this from a dedicated task — never from the HTTP handler
// thread or MqttTask (R9 lesson: a slow/unreachable host must not stall the
// web server or live MQTT publisher).

namespace net {

struct HttpPostResult {
  bool success;
  int  http_status;   // HTTP response code, or -1 if no response was received
  char error[96];     // human-readable failure reason; empty on success
  char body_snippet[128]; // first bytes of response body; useful for 4xx diagnosis
};

// HTTPS POST to url with a JSON body.
// header_pairs: null-terminated list of { name, value, name, value, ... nullptr }.
//               Pass nullptr for no custom headers.
// timeout_ms: total wall-clock budget including TLS handshake + response.
//             Recommended: 10000 (10 s).
HttpPostResult https_post(const char*        url,
                          const char*        body,
                          const char* const* header_pairs,
                          int                timeout_ms);

}  // namespace net
