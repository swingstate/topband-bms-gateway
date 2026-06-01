#include "http_post.h"
#include "esp_tls.h"
#include "esp_crt_bundle.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include <cstdio>
#include <cstring>
#include <cstdlib>

#ifndef CONFIG_MBEDTLS_CERTIFICATE_BUNDLE
#error "CONFIG_MBEDTLS_CERTIFICATE_BUNDLE is not enabled. " \
       "Add CONFIG_MBEDTLS_CERTIFICATE_BUNDLE=y to sdkconfig.defaults, " \
       "delete sdkconfig.esp32s3, and run: pio run -e esp32s3 -t clean && pio run -e esp32s3"
#endif

static const char* TAG = "http_post";

// ── Internal URL helpers ──────────────────────────────────────────────────────

namespace {

// Extract host from "https://host/path" → writes into host_buf, returns path ptr.
// path_out points into url; valid as long as url is valid.
static bool parse_https_url(const char* url,
                             char*       host_buf, size_t host_buf_size,
                             const char** path_out) {
  if (strncmp(url, "https://", 8) != 0) return false;
  const char* host_start = url + 8;
  const char* slash = strchr(host_start, '/');
  size_t host_len = slash ? (size_t)(slash - host_start) : strlen(host_start);
  if (host_len == 0 || host_len >= host_buf_size) return false;
  memcpy(host_buf, host_start, host_len);
  host_buf[host_len] = '\0';
  *path_out = slash ? slash : "/";
  return true;
}

}  // namespace

namespace net {

// ── https_post ────────────────────────────────────────────────────────────────
//
// Uses esp_tls directly (not esp_http_client) to avoid esp_http_client_init's
// complex internal failure modes. Builds a minimal HTTP/1.0 POST request,
// writes it over TLS, reads back the status line, returns the result.

HttpPostResult https_post(const char*        url,
                          const char*        body,
                          const char* const* header_pairs,
                          int                timeout_ms) {
  HttpPostResult result = {};
  result.http_status = -1;

  if (!url || url[0] == '\0') {
    snprintf(result.error, sizeof(result.error), "no URL");
    return result;
  }

  char host[128] = {};
  const char* path = nullptr;
  if (!parse_https_url(url, host, sizeof(host), &path)) {
    snprintf(result.error, sizeof(result.error), "invalid HTTPS URL");
    return result;
  }

  // ── TLS connect ────────────────────────────────────────────────────────────
  esp_tls_cfg_t tls_cfg = {};
  tls_cfg.crt_bundle_attach = esp_crt_bundle_attach;
  tls_cfg.timeout_ms        = timeout_ms;

  esp_tls_t* tls = esp_tls_init();
  if (!tls) {
    snprintf(result.error, sizeof(result.error), "esp_tls_init OOM");
    return result;
  }

  // Capture DRAM state immediately before the handshake so failures can be
  // diagnosed.  mbedTLS SSL I/O buffers allocate from DRAM; under MQTT load
  // (lwIP + MQTT TCP buffers in DRAM) the largest contiguous block may be
  // smaller than the handshake needs.  These values surface in the test-result
  // error string so the operator sees them without a serial console.
  uint32_t dram_free    = heap_caps_get_free_size(MALLOC_CAP_INTERNAL);
  uint32_t dram_largest = heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL);
  ESP_LOGI(TAG, "TLS connect: host=%s DRAM free=%u largest_block=%u",
           host, (unsigned)dram_free, (unsigned)dram_largest);

  // Guard: refuse to attempt a handshake when DRAM is already tight.
  // Without CONFIG_MBEDTLS_DYNAMIC_BUFFER, mbedTLS allocates two fixed ~17 KB
  // I/O buffers (34 KB total) from DRAM.  40 KB gives a safe margin and catches
  // fragmentation where the largest free block is under the mbedTLS minimum.
  // With dynamic buffers the actual need is ~20 KB, so 40 KB is conservative but
  // not restrictive on a healthy device (free DRAM is typically 80–150 KB).
  static constexpr uint32_t TLS_DRAM_MIN_BYTES = 40960;  // 40 KB
  if (dram_largest < TLS_DRAM_MIN_BYTES) {
    snprintf(result.error, sizeof(result.error),
             "insufficient DRAM for TLS (largest=%u B < %u B)",
             (unsigned)dram_largest, (unsigned)TLS_DRAM_MIN_BYTES);
    ESP_LOGW(TAG, "TLS connect aborted: %s", result.error);
    esp_tls_conn_destroy(tls);
    return result;
  }

  int conn_ret = esp_tls_conn_http_new_sync(url, &tls_cfg, tls);
  if (conn_ret != 1) {
    esp_tls_error_handle_t err_h = nullptr;
    int tls_code = 0, tls_flags = 0;
    if (esp_tls_get_error_handle(tls, &err_h) == ESP_OK && err_h) {
      esp_tls_get_and_clear_last_error(err_h, &tls_code, &tls_flags);
    }
    if (tls_code != 0) {
      snprintf(result.error, sizeof(result.error),
               "TLS handshake failed (-0x%04X); DRAM largest=%u B",
               (unsigned)(-tls_code), (unsigned)dram_largest);
    } else {
      snprintf(result.error, sizeof(result.error),
               "TLS connect failed (unreachable/timeout); DRAM largest=%u B",
               (unsigned)dram_largest);
    }
    ESP_LOGW(TAG, "esp_tls_conn_http_new_sync → %d, tls_code=0x%04X, DRAM_largest=%u",
             conn_ret, (unsigned)(-tls_code), (unsigned)dram_largest);
    esp_tls_conn_destroy(tls);
    return result;
  }

  // ── Build and send HTTP request ───────────────────────────────────────────
  // Use HTTP/1.0 (no chunked encoding, connection closes after response).
  int body_len = body ? (int)strlen(body) : 0;

  // Heap-allocate the header to keep stack pressure low.
  // Max size: request line + standard headers + optional extra headers.
  static constexpr size_t HDR_MAX = 512;
  char* hdr = (char*)malloc(HDR_MAX);
  if (!hdr) {
    snprintf(result.error, sizeof(result.error), "OOM building request");
    esp_tls_conn_destroy(tls);
    return result;
  }

  int hdr_len = snprintf(hdr, HDR_MAX,
    "POST %s HTTP/1.0\r\n"
    "Host: %s\r\n"
    "Content-Type: application/json\r\n"
    "Content-Length: %d\r\n"
    "Connection: close\r\n",
    path, host, body_len);

  if (header_pairs) {
    for (int i = 0; header_pairs[i] && header_pairs[i + 1]; i += 2) {
      int n = snprintf(hdr + hdr_len, HDR_MAX - (size_t)hdr_len,
                       "%s: %s\r\n", header_pairs[i], header_pairs[i + 1]);
      if (n > 0 && hdr_len + n < (int)HDR_MAX) hdr_len += n;
    }
  }
  // Blank line ends headers.
  if (hdr_len + 2 < (int)HDR_MAX) {
    hdr[hdr_len++] = '\r';
    hdr[hdr_len++] = '\n';
    hdr[hdr_len]   = '\0';
  }

  // Send headers.
  bool write_ok = true;
  for (int sent = 0; sent < hdr_len && write_ok; ) {
    ssize_t n = esp_tls_conn_write(tls, hdr + sent, (size_t)(hdr_len - sent));
    if (n <= 0) { write_ok = false; break; }
    sent += (int)n;
  }
  free(hdr);

  // Send body.
  if (write_ok && body && body_len > 0) {
    for (int sent = 0; sent < body_len && write_ok; ) {
      ssize_t n = esp_tls_conn_write(tls, body + sent, (size_t)(body_len - sent));
      if (n <= 0) { write_ok = false; break; }
      sent += (int)n;
    }
  }

  if (!write_ok) {
    snprintf(result.error, sizeof(result.error), "TLS write error");
    esp_tls_conn_destroy(tls);
    return result;
  }

  // ── Read response (status line + headers + body snippet) ─────────────────
  // We use HTTP/1.0 with Connection:close so the server closes after sending.
  // Read up to RES_MAX bytes: enough to capture status + headers + the short
  // JSON error body that Telegram returns on 4xx responses.
  static constexpr size_t RES_MAX = 768;
  char resp[RES_MAX + 1] = {};
  int  resp_len  = 0;
  while (resp_len < (int)RES_MAX) {
    ssize_t n = esp_tls_conn_read(tls, resp + resp_len,
                                  (size_t)(RES_MAX - (size_t)resp_len));
    if (n <= 0) break;
    resp_len += (int)n;
  }
  resp[resp_len] = '\0';

  esp_tls_conn_destroy(tls);

  // Parse: "HTTP/1.x NNN ..."
  int status = -1;
  if (resp_len > 9) {
    const char* sp = strchr(resp, ' ');
    if (sp) status = atoi(sp + 1);
  }

  // Extract body snippet: everything after the header-body separator \r\n\r\n.
  // Telegram includes a JSON body on 4xx responses with a "description" field.
  const char* body_start = strstr(resp, "\r\n\r\n");
  if (body_start) {
    body_start += 4;
    size_t blen = (size_t)(resp + resp_len - body_start);
    if (blen > sizeof(result.body_snippet) - 1) blen = sizeof(result.body_snippet) - 1;
    memcpy(result.body_snippet, body_start, blen);
    result.body_snippet[blen] = '\0';
  }

  result.http_status = status;
  result.success     = (status >= 200 && status < 300);
  if (!result.success) {
    if (status > 0)
      snprintf(result.error, sizeof(result.error), "HTTP %d", status);
    else
      snprintf(result.error, sizeof(result.error), "No HTTP response received");
  }

  return result;
}

}  // namespace net
