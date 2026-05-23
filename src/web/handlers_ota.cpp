#include "handlers_ota.h"
#include "app/self_test.h"
#include "diag/alerts.h"
#include "esp_ota_ops.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "esp_http_server.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "mbedtls/sha256.h"
#include <ArduinoJson.h>
#include <cstring>
#include <cstdio>

static const char* TAG = "web_ota";

// ── Size bounds ───────────────────────────────────────────────────────────────
static constexpr size_t OTA_MIN_SIZE = 200  * 1024;           // 200 KB
static constexpr size_t OTA_MAX_SIZE = 3584 * 1024;           // 3.5 MB

static constexpr uint8_t ESP_IMAGE_MAGIC = 0xE9;

// ── Helpers ───────────────────────────────────────────────────────────────────

// Convert a 2-char uppercase/lowercase hex pair to a byte.
// Returns -1 on invalid input.
static int hex2byte(char hi, char lo) {
  auto decode = [](char c) -> int {
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    return -1;
  };
  int h = decode(hi), l = decode(lo);
  if (h < 0 || l < 0) return -1;
  return (h << 4) | l;
}

// Decode a 64-char hex string into 32 bytes. Returns false on bad input.
static bool hex_to_bytes(const char* hex, uint8_t out[32]) {
  if (strlen(hex) != 64) return false;
  for (int i = 0; i < 32; ++i) {
    int b = hex2byte(hex[2*i], hex[2*i+1]);
    if (b < 0) return false;
    out[i] = (uint8_t)b;
  }
  return true;
}

static const char* ota_state_str(esp_ota_img_states_t s) {
  switch (s) {
    case ESP_OTA_IMG_VALID:          return "valid";
    case ESP_OTA_IMG_PENDING_VERIFY: return "pending_validation";
    default:                         return "unknown";
  }
}

// For the non-running (next) partition, map states to the spec strings.
static const char* next_part_state_str(esp_err_t err, esp_ota_img_states_t s) {
  if (err != ESP_OK) return "empty";          // partition not yet written / not OTA
  switch (s) {
    case ESP_OTA_IMG_VALID:                    return "valid";
    case ESP_OTA_IMG_INVALID:
    case ESP_OTA_IMG_ABORTED:                  return "invalid";
    case ESP_OTA_IMG_NEW:
    case ESP_OTA_IMG_PENDING_VERIFY:           return "valid";  // firmware is there
    default:                                   return "unknown";
  }
}

// Delayed-restart task: sends the HTTP response before rebooting.
static void reboot_task(void* /*arg*/) {
  vTaskDelay(pdMS_TO_TICKS(3000));
  ESP_LOGI(TAG, "Executing OTA reboot");
  esp_restart();
}

// ── Upload handler ────────────────────────────────────────────────────────────

namespace web::handlers_ota {

esp_err_t handler_ota_upload(httpd_req_t* req) {
  // ── 1. Size check ────────────────────────────────────────────────────────
  size_t content_len = req->content_len;
  if (content_len < OTA_MIN_SIZE || content_len > OTA_MAX_SIZE) {
    ESP_LOGW(TAG, "OTA upload rejected: content_len=%u (min=%u max=%u)",
             (unsigned)content_len, (unsigned)OTA_MIN_SIZE, (unsigned)OTA_MAX_SIZE);
    httpd_resp_set_status(req, "400 Bad Request");
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_sendstr(req,
        "{\"error\":\"firmware size out of range (200 KB – 3.5 MB)\"}");
  }

  // ── 2. SHA-256 header ────────────────────────────────────────────────────
  char sha_hdr[72] = {};
  esp_err_t hdr_err = httpd_req_get_hdr_value_str(req, "X-Firmware-SHA256",
                                                   sha_hdr, sizeof(sha_hdr));
  if (hdr_err != ESP_OK || strlen(sha_hdr) != 64) {
    ESP_LOGW(TAG, "OTA upload rejected: missing or malformed X-Firmware-SHA256");
    httpd_resp_set_status(req, "400 Bad Request");
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_sendstr(req,
        "{\"error\":\"X-Firmware-SHA256 header missing or invalid (expected 64 hex chars)\"}");
  }

  uint8_t expected_sha[32];
  if (!hex_to_bytes(sha_hdr, expected_sha)) {
    httpd_resp_set_status(req, "400 Bad Request");
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_sendstr(req,
        "{\"error\":\"X-Firmware-SHA256 header contains invalid hex\"}");
  }

  // ── 3. Acquire the inactive OTA partition ────────────────────────────────
  const esp_partition_t* update_part = esp_ota_get_next_update_partition(nullptr);
  if (!update_part) {
    ESP_LOGE(TAG, "No OTA update partition found");
    httpd_resp_set_status(req, "500 Internal Server Error");
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_sendstr(req, "{\"error\":\"no OTA partition available\"}");
  }
  ESP_LOGI(TAG, "OTA target partition: %s at 0x%x",
           update_part->label, (unsigned)update_part->address);

  esp_ota_handle_t ota_handle = 0;
  esp_err_t err = esp_ota_begin(update_part, OTA_WITH_SEQUENTIAL_WRITES, &ota_handle);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "esp_ota_begin: %s", esp_err_to_name(err));
    httpd_resp_set_status(req, "500 Internal Server Error");
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_sendstr(req, "{\"error\":\"OTA begin failed\"}");
  }

  // ── 4. Stream write with incremental SHA-256 ────────────────────────────
  mbedtls_sha256_context sha_ctx;
  mbedtls_sha256_init(&sha_ctx);
  mbedtls_sha256_starts(&sha_ctx, 0);  // 0 = SHA-256 (not SHA-224)

  char chunk[4096];
  size_t total  = 0;
  bool   failed = false;
  bool   magic_checked = false;

  while (total < content_len) {
    size_t to_read = sizeof(chunk);
    if (content_len - total < to_read) to_read = content_len - total;

    int n = httpd_req_recv(req, chunk, to_read);
    if (n <= 0) {
      if (n == HTTPD_SOCK_ERR_TIMEOUT) {
        ESP_LOGW(TAG, "OTA recv timeout at byte %u", (unsigned)total);
      } else {
        ESP_LOGE(TAG, "OTA recv error %d at byte %u", n, (unsigned)total);
      }
      failed = true;
      break;
    }

    // ── 5. Magic byte check (first byte of first chunk) ──────────────────
    if (!magic_checked) {
      magic_checked = true;
      if ((uint8_t)chunk[0] != ESP_IMAGE_MAGIC) {
        ESP_LOGW(TAG, "Magic byte 0x%02x != 0xE9 — not an ESP-IDF binary",
                 (unsigned)(uint8_t)chunk[0]);
        esp_ota_abort(ota_handle);
        mbedtls_sha256_free(&sha_ctx);
        httpd_resp_set_status(req, "400 Bad Request");
        httpd_resp_set_type(req, "application/json");
        return httpd_resp_sendstr(req,
            "{\"error\":\"not an ESP-IDF firmware image (magic byte mismatch)\"}");
      }
    }

    err = esp_ota_write(ota_handle, chunk, (size_t)n);
    if (err != ESP_OK) {
      ESP_LOGE(TAG, "esp_ota_write: %s after %u bytes", esp_err_to_name(err), (unsigned)total);
      failed = true;
      break;
    }

    mbedtls_sha256_update(&sha_ctx, (const uint8_t*)chunk, (size_t)n);
    total += (size_t)n;

    if ((total % (256 * 1024)) < (size_t)n) {
      ESP_LOGI(TAG, "OTA progress: %u / %u KB",
               (unsigned)(total / 1024), (unsigned)(content_len / 1024));
    }
  }

  if (failed) {
    esp_ota_abort(ota_handle);
    mbedtls_sha256_free(&sha_ctx);
    httpd_resp_set_status(req, "500 Internal Server Error");
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_sendstr(req, "{\"error\":\"upload interrupted\"}");
  }

  // ── 6. Finalise OTA write ────────────────────────────────────────────────
  err = esp_ota_end(ota_handle);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "esp_ota_end: %s", esp_err_to_name(err));
    mbedtls_sha256_free(&sha_ctx);
    httpd_resp_set_status(req, "500 Internal Server Error");
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_sendstr(req, "{\"error\":\"OTA end failed — image may be corrupt\"}");
  }

  // ── 7. SHA-256 verify ────────────────────────────────────────────────────
  uint8_t computed_sha[32];
  mbedtls_sha256_finish(&sha_ctx, computed_sha);
  mbedtls_sha256_free(&sha_ctx);

  if (memcmp(computed_sha, expected_sha, 32) != 0) {
    char got[65], expected[65];
    for (int i = 0; i < 32; ++i) {
      snprintf(got      + 2*i, 3, "%02x", computed_sha[i]);
      snprintf(expected + 2*i, 3, "%02x", expected_sha[i]);
    }
    ESP_LOGE(TAG, "SHA-256 mismatch\n  computed: %s\n  expected: %s", got, expected);
    esp_ota_abort(ota_handle);  // already ended — abort the partition state
    httpd_resp_set_status(req, "400 Bad Request");
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_sendstr(req, "{\"error\":\"SHA-256 mismatch — upload corrupt\"}");
  }
  ESP_LOGI(TAG, "SHA-256 verified OK");

  // ── 8. Set boot partition ────────────────────────────────────────────────
  err = esp_ota_set_boot_partition(update_part);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "esp_ota_set_boot_partition: %s", esp_err_to_name(err));
    httpd_resp_set_status(req, "500 Internal Server Error");
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_sendstr(req, "{\"error\":\"failed to set boot partition\"}");
  }
  ESP_LOGI(TAG, "Boot partition set — will reboot in 3 s");

  diag::alerts::emit(diag::alerts::Severity::Info, "ota",
                     "OTA upload complete (%u KB) — rebooting", (unsigned)(total / 1024));

  // ── 9. Respond 200 and schedule reboot ───────────────────────────────────
  httpd_resp_set_type(req, "application/json");
  httpd_resp_sendstr(req,
      "{\"status\":\"ok\",\"message\":\"Firmware uploaded. Rebooting in 3s.\",\"rebooting_in_s\":3}");

  // Detached task so the HTTP response is flushed before restart.
  xTaskCreate(reboot_task, "ota_reboot", 2048, nullptr, 1, nullptr);
  return ESP_OK;
}

// ── Status handler ────────────────────────────────────────────────────────────

esp_err_t handler_ota_status(httpd_req_t* req) {
  const esp_partition_t* running = esp_ota_get_running_partition();
  const esp_partition_t* next    = esp_ota_get_next_update_partition(nullptr);

  // Running partition state.
  esp_ota_img_states_t running_state = ESP_OTA_IMG_UNDEFINED;
  esp_err_t running_err = ESP_FAIL;
  if (running) {
    running_err = esp_ota_get_state_partition(running, &running_state);
  }

  // Next (non-running) partition state.
  esp_ota_img_states_t next_state = ESP_OTA_IMG_UNDEFINED;
  esp_err_t next_err = ESP_FAIL;
  if (next) {
    next_err = esp_ota_get_state_partition(next, &next_state);
  }

  // Self-test status.
  app::self_test::Status st = app::self_test::get_status();

  char body[512];
  int n = snprintf(body, sizeof(body),
    "{"
      "\"running_partition\":\"%s\","
      "\"running_partition_offset\":%u,"
      "\"running_state\":\"%s\","
      "\"next_partition\":\"%s\","
      "\"next_partition_offset\":%u,"
      "\"next_state\":\"%s\","
      "\"self_test\":{"
        "\"in_progress\":%s,"
        "\"elapsed_s\":%lu,"
        "\"deadline_s\":%lu,"
        "\"checks\":{"
          "\"wifi_connected\":%s,"
          "\"http_server_up\":%s,"
          "\"snapshot_published\":%s,"
          "\"controltask_alive\":%s"
        "}"
      "}"
    "}",
    running ? running->label        : "unknown",
    running ? (unsigned)running->address : 0u,
    running_err == ESP_OK ? ota_state_str(running_state) : "unknown",
    next    ? next->label           : "none",
    next    ? (unsigned)next->address : 0u,
    next    ? next_part_state_str(next_err, next_state) : "empty",
    st.in_progress ? "true" : "false",
    (unsigned long)st.elapsed_s,
    (unsigned long)st.deadline_s,
    (st.checks_passed_mask & app::self_test::WIFI_CONNECTED)     ? "true" : "false",
    (st.checks_passed_mask & app::self_test::HTTP_SERVER_UP)     ? "true" : "false",
    (st.checks_passed_mask & app::self_test::SNAPSHOT_PUBLISHED) ? "true" : "false",
    (st.checks_passed_mask & app::self_test::CONTROLTASK_ALIVE)  ? "true" : "false"
  );

  httpd_resp_set_type(req, "application/json");
  httpd_resp_set_hdr(req, "Cache-Control", "no-cache");
  return httpd_resp_send(req, body, n);
}

}  // namespace web::handlers_ota
