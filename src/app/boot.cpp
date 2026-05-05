#include "boot.h"
#include "version.h"
#include "storage/config.h"
#include "storage/nvs_store.h"
#include "bus/queues.h"
#include "bus/snapshot_bus.h"
#include "bms/poller.h"
#include "app/smoke_reader.h"
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"

static const char* TAG = "boot";

// g_config: the loaded (or default) config, accessible to tasks spawned here.
// Declared static so it lives for the device lifetime without a global.
static Config g_config;

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
    ESP_LOGE(TAG, "NVS init failed — running with defaults (no persistence)");
  }

  // ── Step 2: Config ───────────────────────────────────────────────────────
  bool loaded = storage::loadConfig(g_config);
  if (loaded) {
    ESP_LOGI(TAG, "Config loaded from NVS (schema v%u)", g_config.schema_version);
  } else {
    g_config = DEFAULT_CONFIG;
    ESP_LOGW(TAG, "Config not in NVS — applying and saving defaults");
    if (storage::saveConfig(DEFAULT_CONFIG)) {
      ESP_LOGI(TAG, "Default config written to NVS");
    } else {
      ESP_LOGE(TAG, "Failed to write default config to NVS");
    }
  }

  // ── Step 3: FreeRTOS queues ──────────────────────────────────────────────
  {
    uint32_t heap_before = esp_get_free_heap_size();
    if (!bus::createQueues()) {
      ESP_LOGE(TAG, "Queue creation failed — system may be unstable");
    } else {
      uint32_t heap_after = esp_get_free_heap_size();
      ESP_LOGI(TAG, "Queues created. Heap before=%u after=%u (queues=%u B)",
               (unsigned)heap_before, (unsigned)heap_after,
               (unsigned)(heap_before - heap_after));
    }
  }

  // ── Step 4: Snapshot bus (PSRAM double-buffer) ───────────────────────────
  {
    uint32_t psram_before = heap_caps_get_free_size(MALLOC_CAP_SPIRAM);
    if (!bus::snapshot_bus::init()) {
      // Architecture §10 R3: do NOT fall back to internal SRAM for the
      // snapshot bus. PSRAM availability is a hard requirement for V3.0.
      ESP_LOGE(TAG, "Snapshot bus PSRAM init FAILED — cannot continue safely. Halting.");
      // Halt: the heartbeat loop below will never run, which is intentional.
      // A watchdog reset will follow. Better than running with no bus.
      for (;;) { vTaskDelay(pdMS_TO_TICKS(1000)); }
    }
    uint32_t psram_after = heap_caps_get_free_size(MALLOC_CAP_SPIRAM);
    ESP_LOGI(TAG, "Snapshot bus init OK. PSRAM before=%u after=%u allocated=%u B",
             (unsigned)psram_before, (unsigned)psram_after,
             (unsigned)(psram_before - psram_after));
  }

  // ── Step 5: Spawn ControlTask ────────────────────────────────────────────
  if (!bms::poller::start(g_config)) {
    ESP_LOGE(TAG, "ControlTask creation failed");
  } else {
    ESP_LOGI(TAG, "ControlTask created on Core 0 (bms_count=%u)", g_config.bms_count);
  }

  // ── Step 6: Smoke reader (Phase C validation — Core 1) ──────────────────
#if SMOKE_READER_ENABLED
  app::start_smoke_reader();
  ESP_LOGI(TAG, "Smoke reader task created on Core 1 (SMOKE_READER_ENABLED=1)");
#endif

  // ── Step 7: Heartbeat loop ───────────────────────────────────────────────
  // app_main stays alive here. All work is done in spawned tasks.
  ESP_LOGI(TAG, "Boot complete — heartbeat every 5 s");
  for (;;) {
    vTaskDelay(pdMS_TO_TICKS(5000));

    uint32_t uptime_s = (uint32_t)(esp_timer_get_time() / 1000000LL);
    uint32_t heap     = esp_get_free_heap_size();
    uint32_t spiram   = heap_caps_get_free_size(MALLOC_CAP_SPIRAM);

    bms::poller::PollerStats ps{};
    bms::poller::get_stats(ps);

    ESP_LOGI(TAG,
             "Heartbeat | uptime=%lu s | heap=%lu B | psram=%lu B | "
             "publishes=%llu | reads=%llu | retries=%llu | "
             "cycle_avg=%lu ms | cycle_max=%lu ms",
             (unsigned long)uptime_s, (unsigned long)heap, (unsigned long)spiram,
             (unsigned long long)bus::snapshot_bus::total_publishes(),
             (unsigned long long)bus::snapshot_bus::total_reads(),
             (unsigned long long)bus::snapshot_bus::total_read_retries(),
             (unsigned long)ps.cycle_avg_ms,
             (unsigned long)ps.cycle_max_ms);
  }
}

}  // namespace app
