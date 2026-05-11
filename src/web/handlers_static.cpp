#include "handlers_static.h"
#include "auth.h"
#include "net/wifi.h"
#include "storage/lfs_store.h"
#include "storage/config.h"
#include "app/boot.h"
#include "esp_log.h"
#include <cstring>
#include <cstdio>
#include <cstdlib>

static const char* TAG = "web_static";

// Extension → MIME-type lookup.
static const char* mime_for(const char* path) {
  const char* ext = strrchr(path, '.');
  if (!ext) return "application/octet-stream";
  if (strcmp(ext, ".html") == 0) return "text/html; charset=utf-8";
  if (strcmp(ext, ".css")  == 0) return "text/css";
  if (strcmp(ext, ".js")   == 0) return "application/javascript";
  if (strcmp(ext, ".json") == 0) return "application/json";
  if (strcmp(ext, ".svg")  == 0) return "image/svg+xml";
  if (strcmp(ext, ".ico")  == 0) return "image/x-icon";
  if (strcmp(ext, ".png")  == 0) return "image/png";
  if (strcmp(ext, ".txt")  == 0) return "text/plain";
  return "application/octet-stream";
}

// ── Template-substitute index.html ────────────────────────────────────────────
// Replaces {{CSRF_TOKEN}} with the live session token so JS can send it on
// mutations.  File is read fully (< 16 KB in practice), substituted in-place,
// then streamed to the client.  Substitution is case-exact and single-pass.

static esp_err_t serve_index_html(httpd_req_t* req, const char* lfs_path) {
  FILE* f = fopen(lfs_path, "rb");
  if (!f) {
    httpd_resp_set_status(req, "500 Internal Server Error");
    return httpd_resp_sendstr(req, "{\"error\":\"File open failed\"}");
  }

  fseek(f, 0, SEEK_END);
  long file_size = ftell(f);
  rewind(f);

  if (file_size <= 0 || file_size > 32768) {
    fclose(f);
    httpd_resp_set_status(req, "500 Internal Server Error");
    return httpd_resp_sendstr(req, "{\"error\":\"File size invalid\"}");
  }

  char* raw = (char*)malloc((size_t)file_size + 1);
  if (!raw) {
    fclose(f);
    httpd_resp_set_status(req, "500 Internal Server Error");
    return httpd_resp_sendstr(req, "{\"error\":\"OOM\"}");
  }

  size_t n = fread(raw, 1, (size_t)file_size, f);
  fclose(f);
  raw[n] = '\0';

  const char* token = web::auth::get_csrf_token();
  const char* placeholder = "{{CSRF_TOKEN}}";
  size_t ph_len    = strlen(placeholder);
  size_t token_len = strlen(token);

  // Find the placeholder and replace it.
  char* pos = strstr(raw, placeholder);
  if (!pos) {
    // No placeholder found — serve as-is.
    httpd_resp_set_type(req, "text/html; charset=utf-8");
    httpd_resp_set_hdr(req, "Cache-Control", "no-cache, must-revalidate");
    esp_err_t ret = httpd_resp_send(req, raw, (ssize_t)n);
    free(raw);
    return ret;
  }

  // Build substituted output: pre + token + post.
  size_t pre_len  = (size_t)(pos - raw);
  size_t post_len = n - pre_len - ph_len;
  size_t out_len  = pre_len + token_len + post_len;
  char* out = (char*)malloc(out_len + 1);
  if (!out) {
    free(raw);
    httpd_resp_set_status(req, "500 Internal Server Error");
    return httpd_resp_sendstr(req, "{\"error\":\"OOM\"}");
  }
  memcpy(out, raw, pre_len);
  memcpy(out + pre_len, token, token_len);
  memcpy(out + pre_len + token_len, pos + ph_len, post_len);
  free(raw);

  httpd_resp_set_type(req, "text/html; charset=utf-8");
  httpd_resp_set_hdr(req, "Cache-Control", "no-cache, must-revalidate");
  esp_err_t ret = httpd_resp_send(req, out, (ssize_t)out_len);
  free(out);
  return ret;
}

// ── Template-substitute login.html ────────────────────────────────────────────
// Replaces {{AUTH_ENABLED}} with "true"/"false" so the page renders the
// correct form (set-password vs sign-in).

static esp_err_t serve_login_html(httpd_req_t* req, const char* lfs_path) {
  FILE* f = fopen(lfs_path, "rb");
  if (!f) {
    httpd_resp_set_status(req, "500 Internal Server Error");
    return httpd_resp_sendstr(req, "{\"error\":\"File open failed\"}");
  }

  fseek(f, 0, SEEK_END);
  long file_size = ftell(f);
  rewind(f);

  if (file_size <= 0 || file_size > 32768) {
    fclose(f);
    httpd_resp_set_status(req, "500 Internal Server Error");
    return httpd_resp_sendstr(req, "{\"error\":\"File size invalid\"}");
  }

  char* raw = (char*)malloc((size_t)file_size + 1);
  if (!raw) {
    fclose(f);
    httpd_resp_set_status(req, "500 Internal Server Error");
    return httpd_resp_sendstr(req, "{\"error\":\"OOM\"}");
  }

  size_t n = fread(raw, 1, (size_t)file_size, f);
  fclose(f);
  raw[n] = '\0';

  const Config& cfg = app::get_config();
  const char* value      = cfg.auth_enabled ? "true" : "false";
  const char* placeholder = "{{AUTH_ENABLED}}";
  size_t ph_len    = strlen(placeholder);
  size_t value_len = strlen(value);

  char* pos = strstr(raw, placeholder);
  if (!pos) {
    httpd_resp_set_type(req, "text/html; charset=utf-8");
    httpd_resp_set_hdr(req, "Cache-Control", "no-cache, must-revalidate");
    esp_err_t ret = httpd_resp_send(req, raw, (ssize_t)n);
    free(raw);
    return ret;
  }

  size_t pre_len  = (size_t)(pos - raw);
  size_t post_len = n - pre_len - ph_len;
  size_t out_len  = pre_len + value_len + post_len;
  char* out = (char*)malloc(out_len + 1);
  if (!out) {
    free(raw);
    httpd_resp_set_status(req, "500 Internal Server Error");
    return httpd_resp_sendstr(req, "{\"error\":\"OOM\"}");
  }
  memcpy(out, raw, pre_len);
  memcpy(out + pre_len, value, value_len);
  memcpy(out + pre_len + value_len, pos + ph_len, post_len);
  free(raw);

  httpd_resp_set_type(req, "text/html; charset=utf-8");
  httpd_resp_set_hdr(req, "Cache-Control", "no-cache, must-revalidate");
  esp_err_t ret = httpd_resp_send(req, out, (ssize_t)out_len);
  free(out);
  return ret;
}

namespace web {

esp_err_t handle_wifi_setup_page(httpd_req_t* req) {
  // Redirect to the proper setup page if LittleFS is available.
  const char* path = "/lfs/ui/setup.html";
  if (storage::lfs::exists(path)) {
    httpd_resp_set_status(req, "302 Found");
    httpd_resp_set_hdr(req, "Location", "/setup.html");
    return httpd_resp_sendstr(req, "");
  }
  // LittleFS not available — return minimal inline page.
  httpd_resp_set_type(req, "text/html; charset=utf-8");
  return httpd_resp_sendstr(req, "<html><body><h2>WiFi setup — LittleFS not ready</h2></body></html>");
}

esp_err_t handle_static(httpd_req_t* req) {
  const char* uri = req->uri;

  // AP mode: redirect everything to setup page.
  if (net::wifi::is_ap_mode()) {
    httpd_resp_set_status(req, "302 Found");
    httpd_resp_set_hdr(req, "Location", "http://192.168.4.1/setup.html");
    return httpd_resp_sendstr(req, "");
  }

  // Map URI to LittleFS path.
  char lfs_path[600];
  if (strcmp(uri, "/") == 0 || strcmp(uri, "/index.html") == 0) {
    snprintf(lfs_path, sizeof(lfs_path), "/lfs/ui/index.html");
  } else {
    snprintf(lfs_path, sizeof(lfs_path), "/lfs/ui%s", uri);
  }

  // Security: reject path traversal.
  if (strstr(lfs_path, "..")) {
    httpd_resp_set_status(req, "403 Forbidden");
    return httpd_resp_sendstr(req, "Forbidden");
  }

  if (!storage::lfs::exists(lfs_path)) {
    // SPA fallback: paths without a file extension are client-side routes
    // (e.g. /dashboard, /settings). Serve index.html so the JS router takes over.
    const char* last_seg = strrchr(uri, '/');
    bool has_extension   = last_seg && strchr(last_seg, '.');
    if (!has_extension) {
      snprintf(lfs_path, sizeof(lfs_path), "/lfs/ui/index.html");
      return serve_index_html(req, lfs_path);
    }
    httpd_resp_set_status(req, "404 Not Found");
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_sendstr(req, "{\"error\":\"Not found\"}");
  }

  // Special-case templates.
  bool is_index = (strcmp(lfs_path, "/lfs/ui/index.html") == 0);
  bool is_login = (strcmp(lfs_path, "/lfs/ui/login.html") == 0);

  if (is_index) return serve_index_html(req, lfs_path);
  if (is_login) return serve_login_html(req, lfs_path);

  // Generic file: stream in 2 KB chunks.
  FILE* f = fopen(lfs_path, "rb");
  if (!f) {
    httpd_resp_set_status(req, "500 Internal Server Error");
    return httpd_resp_sendstr(req, "{\"error\":\"File open failed\"}");
  }

  const char* mime = mime_for(lfs_path);
  httpd_resp_set_type(req, mime);
  httpd_resp_set_hdr(req, "Cache-Control", "max-age=3600");

  static constexpr size_t CHUNK = 2048;
  char* buf = (char*)malloc(CHUNK);
  if (!buf) {
    fclose(f);
    httpd_resp_set_status(req, "500 Internal Server Error");
    return httpd_resp_sendstr(req, "{\"error\":\"OOM\"}");
  }

  size_t nr;
  esp_err_t ret = ESP_OK;
  while ((nr = fread(buf, 1, CHUNK, f)) > 0) {
    ret = httpd_resp_send_chunk(req, buf, (ssize_t)nr);
    if (ret != ESP_OK) {
      ESP_LOGD(TAG, "client disconnected during static send: %s", lfs_path);
      break;
    }
  }
  fclose(f);
  free(buf);
  if (ret == ESP_OK) httpd_resp_send_chunk(req, nullptr, 0);
  return ret;
}

}  // namespace web
