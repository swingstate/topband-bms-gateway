#include "boot.h"
#include "version.h"
#include "storage/config.h"
#include "storage/nvs_store.h"
#include "bus/queues.h"
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"

static const char* TAG = "boot";

namespace app {

// NVS flash init with standard erase-and-retry recovery.
// Returns true on success.
static bool init_nvs() {
  esp_err_t ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    // NVS_NEW_VERSION_FOUND: partition came from V2.67; per architecture D4.6,
    // no in-NVS migration — erase and start fresh.
    ESP_LOGW(TAG, "NVS partition unusable (%s) — erasing and re-initializing",
             esp_err_to_name(ret));
    ESP_ERROR_CHECK(nvs_flash_erase());
    ret = nvs_flash_init();
  }
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "nvs_flash_init failed: %s", esp_err_to_name(ret));
    return false;
  }
  ESP_LOGI(TAG, "NVS initialized");
  return true;
}

void run_boot() {
  ESP_LOGI(TAG, "TopBand BMS Gateway %s  git=%s  built=%s %s",
           FW_VERSION, GIT_SHA, BUILD_DATE, BUILD_TIME);

  // ── Step 1: NVS ──────────────────────────────────────────────────────────
  if (!init_nvs()) {
    // Without NVS, no config can be persisted. Continue with defaults in RAM.
    ESP_LOGE(TAG, "NVS init failed — running with defaults (no persistence)");
  }

  // ── Step 2: Config ───────────────────────────────────────────────────────
  Config cfg;
  bool loaded = storage::loadConfig(cfg);
  if (loaded) {
    ESP_LOGI(TAG, "Config loaded from NVS (schema v%u)", cfg.schema_version);
  } else {
    ESP_LOGW(TAG, "Config not in NVS — applying and saving defaults");
    if (storage::saveConfig(DEFAULT_CONFIG)) {
      ESP_LOGI(TAG, "Default config written to NVS");
    } else {
      ESP_LOGE(TAG, "Failed to write default config to NVS");
    }
  }

  // ── Step 3: FreeRTOS queues ──────────────────────────────────────────────
  uint32_t heap_before = esp_get_free_heap_size();
  if (!bus::createQueues()) {
    ESP_LOGE(TAG, "Queue creation failed — system may be unstable");
  } else {
    uint32_t heap_after = esp_get_free_heap_size();
    ESP_LOGI(TAG, "Queues created. Heap before=%u after=%u (queues=%u B)",
             (unsigned)heap_before, (unsigned)heap_after,
             (unsigned)(heap_before - heap_after));
  }

  uint32_t psram = heap_caps_get_free_size(MALLOC_CAP_SPIRAM);
  ESP_LOGI(TAG, "Free heap: %u B  Free PSRAM: %u B",
           (unsigned)esp_get_free_heap_size(), (unsigned)psram);

  // ── Step 4: Heartbeat loop ───────────────────────────────────────────────
  // No tasks spawned in Phase A. app_main IS the only app task.
  ESP_LOGI(TAG, "Boot complete — entering heartbeat loop (5 s)");
  for (;;) {
    vTaskDelay(pdMS_TO_TICKS(5000));
    uint32_t uptime_s = (uint32_t)(esp_timer_get_time() / 1000000LL);
    uint32_t heap     = esp_get_free_heap_size();
    uint32_t spiram   = heap_caps_get_free_size(MALLOC_CAP_SPIRAM);
    ESP_LOGI(TAG, "Heartbeat | uptime=%lu s | heap=%lu B | psram=%lu B",
             (unsigned long)uptime_s, (unsigned long)heap, (unsigned long)spiram);
  }
}

}  // namespace app
