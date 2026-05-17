#include "handlers_static.h"
#include "auth.h"
#include "net/wifi.h"
#include "storage/lfs_store.h"
#include "storage/config.h"
#include "storage/ui_provisioner.h"
#include "app/boot.h"
#include "esp_log.h"
#include "esp_http_server.h"
#include <cstring>
#include <cstdio>
#include <cstdlib>

static const char* TAG = "web_static";

// Extension → MIME-type lookup.
// Strips a trailing ".gz" before matching so pre-compressed files get the
// correct Content-Type of the underlying asset.
static const char* mime_for(const char* path) {
  // Work on a copy with .gz stripped.
  char tmp[256];
  size_t plen = strlen(path);
  if (plen > 3 && strcmp(path + plen - 3, ".gz") == 0) {
    size_t base_len = plen - 3;
    if (base_len >= sizeof(tmp)) base_len = sizeof(tmp) - 1;
    memcpy(tmp, path, base_len);
    tmp[base_len] = '\0';
    path = tmp;
  }
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

// Replaces every occurrence of `find` in `src` (length `src_len`) with
// `replace`. Returns a new malloc'd buffer; caller must free(). Sets *out_len.
// Returns nullptr on OOM.
static char* multi_replace(const char* src, size_t src_len,
                            const char* find, const char* replace,
                            size_t* out_len) {
  size_t find_len    = strlen(find);
  size_t replace_len = strlen(replace);

  size_t count = 0;
  const char* p = src;
  while ((p = strstr(p, find)) != nullptr) { count++; p += find_len; }

  size_t result_len = src_len - count * find_len + count * replace_len;
  char* out = (char*)malloc(result_len + 1);
  if (!out) return nullptr;

  char* dst = out;
  const char* s = src;
  const char* match;
  while ((match = strstr(s, find)) != nullptr) {
    size_t before = (size_t)(match - s);
    memcpy(dst, s, before);  dst += before;
    memcpy(dst, replace, replace_len); dst += replace_len;
    s = match + find_len;
  }
  size_t tail = (size_t)((src + src_len) - s);
  memcpy(dst, s, tail);
  dst[tail] = '\0';
  *out_len = result_len;
  return out;
}

// ── Template-substitute index.html ────────────────────────────────────────────
// Two-pass substitution on the raw file bytes:
//   {{CSRF_TOKEN}}  → live session token
//   {{UI_VERSION}}  → cache-buster string in asset query params

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

  char* buf = (char*)malloc((size_t)file_size + 1);
  if (!buf) {
    fclose(f);
    httpd_resp_set_status(req, "500 Internal Server Error");
    return httpd_resp_sendstr(req, "{\"error\":\"OOM\"}");
  }

  size_t buf_len = fread(buf, 1, (size_t)file_size, f);
  fclose(f);
  buf[buf_len] = '\0';

  // Pass 1: {{CSRF_TOKEN}}
  {
    size_t out_len;
    char* next = multi_replace(buf, buf_len, "{{CSRF_TOKEN}}",
                               web::auth::get_csrf_token(), &out_len);
    free(buf);
    if (!next) {
      httpd_resp_set_status(req, "500 Internal Server Error");
      return httpd_resp_sendstr(req, "{\"error\":\"OOM\"}");
    }
    buf = next; buf_len = out_len;
  }

  // Pass 2: {{UI_VERSION}} (appears in ?v= query params on asset URLs)
  {
    size_t out_len;
    char* next = multi_replace(buf, buf_len, "{{UI_VERSION}}",
                               storage::ui_provisioner::UI_VERSION, &out_len);
    free(buf);
    if (!next) {
      httpd_resp_set_status(req, "500 Internal Server Error");
      return httpd_resp_sendstr(req, "{\"error\":\"OOM\"}");
    }
    buf = next; buf_len = out_len;
  }

  httpd_resp_set_type(req, "text/html; charset=utf-8");
  httpd_resp_set_hdr(req, "Cache-Control", "no-cache, must-revalidate");
  esp_err_t ret = httpd_resp_send(req, buf, (ssize_t)buf_len);
  free(buf);
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
  // Strip query string (e.g. ?v=h2-mvp-2 cache-buster) before path lookup.
  char uri_buf[256];
  const char* raw_uri = req->uri;
  const char* q = strchr(raw_uri, '?');
  if (q) {
    size_t plen = (size_t)(q - raw_uri);
    if (plen >= sizeof(uri_buf)) plen = sizeof(uri_buf) - 1;
    memcpy(uri_buf, raw_uri, plen);
    uri_buf[plen] = '\0';
  } else {
    strncpy(uri_buf, raw_uri, sizeof(uri_buf) - 1);
    uri_buf[sizeof(uri_buf) - 1] = '\0';
  }
  const char* uri = uri_buf;

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

  // Check if client accepts gzip and a pre-compressed companion exists.
  char enc_buf[64] = {};
  bool client_gzip = false;
  if (httpd_req_get_hdr_value_str(req, "Accept-Encoding",
                                   enc_buf, sizeof(enc_buf)) == ESP_OK) {
    client_gzip = (strstr(enc_buf, "gzip") != nullptr);
  }

  char gz_path[608];
  bool serve_gz = false;
  if (client_gzip) {
    snprintf(gz_path, sizeof(gz_path), "%s.gz", lfs_path);
    serve_gz = storage::lfs::exists(gz_path);
  }

  const char* serve_path = serve_gz ? gz_path : lfs_path;
  FILE* f = fopen(serve_path, "rb");
  if (!f) {
    httpd_resp_set_status(req, "500 Internal Server Error");
    return httpd_resp_sendstr(req, "{\"error\":\"File open failed\"}");
  }

  httpd_resp_set_type(req, mime_for(lfs_path));  // MIME from uncompressed name
  httpd_resp_set_hdr(req, "Cache-Control", "max-age=3600");
  if (serve_gz) {
    httpd_resp_set_hdr(req, "Content-Encoding", "gzip");
    httpd_resp_set_hdr(req, "Vary", "Accept-Encoding");
  }

  static constexpr size_t CHUNK = 4096;
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
      ESP_LOGD(TAG, "client disconnected during static send: %s", serve_path);
      break;
    }
  }
  fclose(f);
  free(buf);
  if (ret == ESP_OK) httpd_resp_send_chunk(req, nullptr, 0);
  return ret;
}

}  // namespace web
