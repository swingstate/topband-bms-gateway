#include "auth.h"
#include "app/boot.h"
#include "storage/config.h"
#include "esp_log.h"
#include "esp_random.h"
#include "esp_http_server.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "mbedtls/sha256.h"
#include <ArduinoJson.h>
#include <cstring>
#include <cstdio>

static const char* TAG = "auth";

// 16 random bytes → 32 hex chars + '\0'
static char g_session_token[33] = {};

static constexpr size_t BODY_MAX = 512;

// ── Helpers ───────────────────────────────────────────────────────────────────

// Read the full request body (up to max_len bytes).
static size_t read_body(httpd_req_t* req, char* buf, size_t max_len) {
  size_t remaining = req->content_len;
  if (remaining == 0 || remaining > max_len) return 0;
  size_t total = 0;
  while (remaining > 0) {
    int n = httpd_req_recv(req, buf + total, remaining);
    if (n <= 0) return total;
    total     += (size_t)n;
    remaining -= (size_t)n;
  }
  buf[total] = '\0';
  return total;
}

static esp_err_t send_json_err(httpd_req_t* req, const char* status,
                               const char* msg) {
  char body[256];
  snprintf(body, sizeof(body), "{\"error\":\"%s\"}", msg);
  httpd_resp_set_status(req, status);
  httpd_resp_set_type(req, "application/json");
  return httpd_resp_sendstr(req, body);
}

// Parse a single cookie value from the Cookie header.
// Returns empty string if not found.
static std::string get_cookie(httpd_req_t* req, const char* name) {
  size_t len = httpd_req_get_hdr_value_len(req, "Cookie");
  if (len == 0) return {};
  std::string hdr(len + 1, '\0');
  httpd_req_get_hdr_value_str(req, "Cookie", &hdr[0], len + 1);
  hdr.resize(len);  // strip null terminator so substr() to npos doesn't include it

  // Find "name=" in the cookie string.
  std::string needle = std::string(name) + "=";
  size_t pos = hdr.find(needle);
  if (pos == std::string::npos) return {};
  pos += needle.size();
  size_t end = hdr.find(';', pos);
  return hdr.substr(pos, end == std::string::npos ? end : end - pos);
}

// Read a header value by name.
static std::string get_header(httpd_req_t* req, const char* name) {
  size_t len = httpd_req_get_hdr_value_len(req, name);
  if (len == 0) return {};
  std::string val(len + 1, '\0');
  httpd_req_get_hdr_value_str(req, name, &val[0], len + 1);
  val.resize(len);
  return val;
}

// ── Public API ────────────────────────────────────────────────────────────────

namespace web::auth {

void init() {
  uint32_t rnd[4];
  for (int i = 0; i < 4; i++) rnd[i] = esp_random();
  for (int i = 0; i < 4; i++) {
    snprintf(g_session_token + i * 8, 9, "%08x", (unsigned)rnd[i]);
  }
  ESP_LOGI(TAG, "Session token generated (first 8: %.8s…)", g_session_token);
}

bool check(httpd_req_t* req) {
  const Config& cfg = app::get_config();
  if (!cfg.auth_enabled) return true;  // bypass for initial setup / 5x reset

  // Validate session cookie.
  std::string cookie = get_cookie(req, "tbsid");
  if (cookie != g_session_token) {
    ESP_LOGD(TAG, "check: cookie mismatch or missing");
    return false;
  }

  // For mutating methods also validate CSRF header.
  const char* method = req->method == HTTP_POST ? "POST"
                     : req->method == HTTP_PUT  ? "PUT"
                     : req->method == HTTP_DELETE ? "DELETE"
                     : req->method == HTTP_PATCH  ? "PATCH"
                     : nullptr;
  if (method) {
    std::string csrf = get_header(req, "X-CSRF-Token");
    if (csrf != g_session_token) {
      ESP_LOGD(TAG, "check: CSRF token mismatch for %s", method);
      return false;
    }
  }
  return true;
}

// ── Login ─────────────────────────────────────────────────────────────────────

esp_err_t handler_login(httpd_req_t* req) {
  char body[BODY_MAX] = {};
  if (read_body(req, body, BODY_MAX - 1) == 0) {
    return send_json_err(req, "400 Bad Request", "Empty body");
  }

  JsonDocument doc;
  if (deserializeJson(doc, body) != DeserializationError::Ok) {
    return send_json_err(req, "400 Bad Request", "JSON parse error");
  }

  const char* pw = doc["password"] | "";

  const Config& cfg = app::get_config();

  // If auth not yet enabled, treat as first-time setup: accept any password.
  bool ok = false;
  if (!cfg.auth_enabled) {
    ok = true;
  } else {
    std::string hash = hash_password(std::string(pw));
    ok = (hash == cfg.auth_hash);
  }

  if (!ok) {
    ESP_LOGW(TAG, "Login failed (wrong password)");
    // Small delay to blunt brute force attempts.
    vTaskDelay(pdMS_TO_TICKS(50));
    httpd_resp_set_status(req, "401 Unauthorized");
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_sendstr(req, "{\"ok\":false,\"error\":\"Wrong password\"}");
  }

  // Set session cookie — 30-day Max-Age; SameSite=Lax is sufficient on a LAN
  // device (mutating requests are separately protected by the CSRF double-submit).
  char cookie_hdr[128];
  snprintf(cookie_hdr, sizeof(cookie_hdr),
           "tbsid=%s; HttpOnly; SameSite=Lax; Path=/; Max-Age=2592000",
           g_session_token);
  httpd_resp_set_hdr(req, "Set-Cookie", cookie_hdr);
  httpd_resp_set_type(req, "application/json");

  char resp[128];
  snprintf(resp, sizeof(resp),
           "{\"ok\":true,\"csrf\":\"%s\"}", g_session_token);
  ESP_LOGI(TAG, "Login successful");
  return httpd_resp_sendstr(req, resp);
}

// ── Logout ────────────────────────────────────────────────────────────────────

esp_err_t handler_logout(httpd_req_t* req) {
  // Rotate the session token so the old cookie is immediately invalid
  // server-side (not just expired on the client).
  init();
  httpd_resp_set_hdr(req, "Set-Cookie",
                     "tbsid=; HttpOnly; SameSite=Lax; Path=/; Max-Age=0");
  httpd_resp_set_type(req, "application/json");
  ESP_LOGI(TAG, "User logged out — session token rotated");
  return httpd_resp_sendstr(req, "{\"ok\":true}");
}

// ── Set password ──────────────────────────────────────────────────────────────

esp_err_t handler_set_password(httpd_req_t* req) {
  char body[BODY_MAX] = {};
  if (read_body(req, body, BODY_MAX - 1) == 0) {
    return send_json_err(req, "400 Bad Request", "Empty body");
  }

  JsonDocument doc;
  if (deserializeJson(doc, body) != DeserializationError::Ok) {
    return send_json_err(req, "400 Bad Request", "JSON parse error");
  }

  const char* current_pw = doc["current"] | "";
  const char* new_pw     = doc["new"]     | "";

  if (new_pw[0] == '\0') {
    return send_json_err(req, "400 Bad Request", "new password must not be empty");
  }

  const Config& old_cfg = app::get_config();

  if (old_cfg.auth_enabled) {
    // Verify current password before allowing change.
    std::string cur_hash = hash_password(std::string(current_pw));
    if (cur_hash != old_cfg.auth_hash) {
      vTaskDelay(pdMS_TO_TICKS(50));
      return send_json_err(req, "403 Forbidden", "Wrong current password");
    }
  }
  // else: auth_enabled==false → initial setup, skip current-password check.

  Config new_cfg = old_cfg;
  std::string hash_str = hash_password(std::string(new_pw));
  snprintf(new_cfg.auth_hash, sizeof(new_cfg.auth_hash), "%s", hash_str.c_str());
  new_cfg.auth_enabled = true;

  if (!app::update_and_save_config(new_cfg)) {
    return send_json_err(req, "500 Internal Server Error", "NVS save failed");
  }

  // Issue a fresh session cookie now that auth is enabled.
  char cookie_hdr[128];
  snprintf(cookie_hdr, sizeof(cookie_hdr),
           "tbsid=%s; HttpOnly; SameSite=Lax; Path=/; Max-Age=2592000",
           g_session_token);
  httpd_resp_set_hdr(req, "Set-Cookie", cookie_hdr);
  httpd_resp_set_type(req, "application/json");

  char resp[128];
  snprintf(resp, sizeof(resp),
           "{\"ok\":true,\"csrf\":\"%s\"}", g_session_token);

  ESP_LOGI(TAG, "Password set — auth_enabled=true");
  return httpd_resp_sendstr(req, resp);
}

// ── Helpers ───────────────────────────────────────────────────────────────────

const char* get_csrf_token() {
  return g_session_token;
}

std::string hash_password(const std::string& plain) {
  uint8_t hash[32] = {};
  mbedtls_sha256((const unsigned char*)plain.c_str(), plain.size(), hash, 0);
  char hex[65] = {};
  for (int i = 0; i < 32; i++) {
    snprintf(hex + 2 * i, 3, "%02x", hash[i]);
  }
  return std::string(hex, 64);
}

}  // namespace web::auth
