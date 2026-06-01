#include "http_post.h"
#include "esp_http_client.h"
#include "esp_crt_bundle.h"  // esp_crt_bundle_attach
#include "esp_log.h"
#include <cstdio>
#include <cstring>

static const char* TAG = "http_post";

// Capture response body up to this many bytes (we only need the status/error).
static constexpr size_t RESP_BUF = 256;

namespace {

struct ClientCtx {
  char  resp_buf[RESP_BUF];
  size_t resp_len = 0;
  esp_err_t event_err = ESP_OK;
};

static esp_err_t event_handler(esp_http_client_event_t* evt) {
  auto* ctx = static_cast<ClientCtx*>(evt->user_data);
  if (!ctx) return ESP_OK;

  switch (evt->event_id) {
    case HTTP_EVENT_ON_DATA:
      if (evt->data_len > 0 && ctx->resp_len < RESP_BUF - 1) {
        size_t copy = evt->data_len;
        if (ctx->resp_len + copy >= RESP_BUF) copy = RESP_BUF - 1 - ctx->resp_len;
        memcpy(ctx->resp_buf + ctx->resp_len, evt->data, copy);
        ctx->resp_len += copy;
        ctx->resp_buf[ctx->resp_len] = '\0';
      }
      break;
    case HTTP_EVENT_ERROR:
      ctx->event_err = ESP_FAIL;
      break;
    default:
      break;
  }
  return ESP_OK;
}

}  // namespace

namespace net {

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

  ClientCtx ctx = {};

  esp_http_client_config_t cfg = {};
  cfg.url              = url;
  cfg.method           = HTTP_METHOD_POST;
  cfg.timeout_ms       = timeout_ms;
  cfg.crt_bundle_attach = esp_crt_bundle_attach;  // validates against common CA roots
  cfg.event_handler    = event_handler;
  cfg.user_data        = &ctx;
  cfg.keep_alive_enable = false;
  // Buffer sizes: keep modest to limit DRAM usage during TLS handshake.
  cfg.buffer_size       = 512;
  cfg.buffer_size_tx    = 512;

  esp_http_client_handle_t client = esp_http_client_init(&cfg);
  if (!client) {
    snprintf(result.error, sizeof(result.error), "esp_http_client_init failed");
    return result;
  }

  // Content-Type header is always required for JSON.
  esp_http_client_set_header(client, "Content-Type", "application/json");

  // Caller-supplied additional headers.
  if (header_pairs) {
    for (int i = 0; header_pairs[i] && header_pairs[i + 1]; i += 2) {
      esp_http_client_set_header(client, header_pairs[i], header_pairs[i + 1]);
    }
  }

  if (body && body[0] != '\0') {
    esp_http_client_set_post_field(client, body, (int)strlen(body));
  }

  esp_err_t err = esp_http_client_perform(client);

  if (err == ESP_OK) {
    result.http_status = esp_http_client_get_status_code(client);
    result.success     = (result.http_status >= 200 && result.http_status < 300);
    if (!result.success) {
      snprintf(result.error, sizeof(result.error),
               "HTTP %d", result.http_status);
    }
  } else {
    // Map common IDF error codes to readable strings.
    if (err == ESP_ERR_HTTP_CONNECT) {
      snprintf(result.error, sizeof(result.error),
               "Connection refused or host unreachable");
    } else if (err == ESP_ERR_HTTP_EAGAIN || err == ESP_ERR_TIMEOUT) {
      snprintf(result.error, sizeof(result.error), "Timeout (%d ms)", timeout_ms);
    } else if (err == ESP_ERR_HTTP_CONNECTION_CLOSED) {
      snprintf(result.error, sizeof(result.error), "Connection closed by remote host");
    } else {
      snprintf(result.error, sizeof(result.error),
               "HTTP client error: %s", esp_err_to_name(err));
    }
    ESP_LOGW(TAG, "https_post %s failed: %s", url, esp_err_to_name(err));
  }

  esp_http_client_cleanup(client);
  return result;
}

}  // namespace net
