#include "self_test.h"
#include "diag/alerts.h"
#include "esp_ota_ops.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <atomic>

static const char* TAG = "self_test";

static constexpr uint32_t VALIDATION_WINDOW_S = 300;   // 5 minutes
static constexpr uint32_t MIN_UPTIME_S        = 30;
static constexpr uint8_t  ALL_CHECKS          = 0x0F;  // all 4 bits

static bool                  s_active      = false;
static int64_t               s_start_us    = 0;
static std::atomic<uint8_t>  s_checks{0};

// Watchdog task: validates when all checks pass + min uptime met, or
// forces reboot for rollback when the 5-minute deadline expires.
static void watchdog_task(void* /*arg*/) {
  for (;;) {
    vTaskDelay(pdMS_TO_TICKS(2000));

    if (!s_active) {
      vTaskDelete(nullptr);
      return;
    }

    int64_t elapsed_us = esp_timer_get_time() - s_start_us;
    if (elapsed_us < 0) elapsed_us = 0;
    uint32_t elapsed_s = (uint32_t)(elapsed_us / 1000000LL);
    uint32_t uptime_s  = (uint32_t)(esp_timer_get_time() / 1000000LL);
    uint8_t  checks    = s_checks.load(std::memory_order_relaxed);

    // All checks passed and minimum uptime elapsed — validate and exit.
    if ((checks & ALL_CHECKS) == ALL_CHECKS && uptime_s >= MIN_UPTIME_S) {
      esp_err_t err = esp_ota_mark_app_valid_cancel_rollback();
      s_active = false;
      if (err == ESP_OK) {
        ESP_LOGI(TAG, "firmware validated at uptime=%lu s", (unsigned long)uptime_s);
        diag::alerts::emit(diag::alerts::Severity::Info, "ota",
                           "app self-test: firmware validated");
      } else {
        ESP_LOGE(TAG, "mark_valid failed: %s", esp_err_to_name(err));
      }
      vTaskDelete(nullptr);
      return;
    }

    // Deadline exceeded without all checks — reboot for rollback.
    if (elapsed_s >= VALIDATION_WINDOW_S) {
      ESP_LOGE(TAG,
               "self-test deadline exceeded (checks=0x%02x, uptime=%lu s) — rebooting for rollback",
               (unsigned)checks, (unsigned long)uptime_s);
      diag::alerts::emit(diag::alerts::Severity::Critical, "ota",
                         "app self-test: rollback triggered — deadline exceeded");
      vTaskDelay(pdMS_TO_TICKS(500));  // let the alert flush
      esp_restart();
    }
  }
}

namespace app::self_test {

void init() {
  const esp_partition_t* running = esp_ota_get_running_partition();
  if (!running) return;

  esp_ota_img_states_t state;
  esp_err_t err = esp_ota_get_state_partition(running, &state);
  if (err != ESP_OK) {
    // Normal for factory partition or when OTA has never been used.
    ESP_LOGD(TAG, "no OTA state for running partition (%s) — normal boot",
             esp_err_to_name(err));
    return;
  }

  if (state != ESP_OTA_IMG_PENDING_VERIFY) {
    // Partition already validated — normal boot.
    return;
  }

  s_active   = true;
  s_start_us = esp_timer_get_time();
  s_checks.store(0, std::memory_order_relaxed);

  ESP_LOGI(TAG, "first boot after OTA — 5-min self-test started");
  diag::alerts::emit(diag::alerts::Severity::Info, "ota",
                     "app self-test: validating new firmware");

  // Watchdog on Core 1, lowest priority, 4 KB stack (small — no I/O, no large locals).
  xTaskCreatePinnedToCore(watchdog_task, "ota_wdg", 4096, nullptr, 1, nullptr, 1);
}

void mark_passed(Check c) {
  if (!s_active) return;
  uint8_t prev = s_checks.fetch_or((uint8_t)c, std::memory_order_relaxed);
  if (!(prev & (uint8_t)c)) {
    ESP_LOGI(TAG, "check 0x%02x passed (mask now 0x%02x)",
             (unsigned)c, (unsigned)(prev | (uint8_t)c));
  }
}

bool in_validation_window() {
  return s_active;
}

Status get_status() {
  Status st{};
  st.in_progress = s_active;
  if (s_active) {
    int64_t elapsed_us = esp_timer_get_time() - s_start_us;
    if (elapsed_us < 0) elapsed_us = 0;
    st.elapsed_s          = (uint32_t)(elapsed_us / 1000000LL);
    st.deadline_s         = VALIDATION_WINDOW_S;
    st.checks_passed_mask = s_checks.load(std::memory_order_relaxed);
  }
  return st;
}

}  // namespace app::self_test
