#include "boot.h"
#include "version.h"
#include "storage/config.h"
#include "storage/nvs_store.h"
#include "storage/lfs_store.h"
#include "storage/ui_provisioner.h"
#include "storage/boot_reasons.h"
#include "storage/history_store.h"
#include "storage/energy_store.h"
#include "bus/queues.h"
#include "bus/snapshot_bus.h"
#include "bms/poller.h"
#include "net/wifi.h"
#include "net/ntp.h"
#include "net/captdns.h"
#include "web/server.h"
#include "web/auth.h"
#include "web/captive.h"
#include "app/smoke_reader.h"
#include "app/housekeeping.h"
#include "app/history_task.h"
#include "mqtt/publisher.h"
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include <cstring>

static const char* TAG = "boot";

// Runtime config — lives for device lifetime, accessible via app::get_config().
static Config g_config;

namespace app {

const Config& get_config() {
  return g_config;
}

bool update_and_save_config(const Config& new_cfg) {
  char field_err[64] = {};
  ValidationError verr = storage::validate(new_cfg, field_err, sizeof(field_err));
  if (verr != ValidationError::None) {
    ESP_LOGW(TAG, "update_and_save_config: validation failed on field=%s", field_err);
    return false;
  }
  if (!storage::saveConfig(new_cfg)) {
    ESP_LOGE(TAG, "update_and_save_config: NVS save failed");
    return false;
  }
  g_config = new_cfg;
  return true;
}

// NVS flash init with standard erase-and-retry recovery.
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

// Start captive portal: AP + DNS hijack + captive HTTP server.
static void enter_captive_portal() {
  std::string ssid = net::wifi::start_ap();
  ESP_LOGI(TAG, "AP mode — SSID=%s", ssid.c_str());

  // DNS hijack: all queries resolve to 192.168.4.1 (the AP gateway).
  // inet_addr() returns network-byte-order uint32_t.
  net::captdns::start(0x0104A8C0u);  // 192.168.4.1 in network byte order

  web::auth::init();
  if (!web::captive::start()) {
    ESP_LOGE(TAG, "Captive portal HTTP server failed to start");
  }
}

void run_boot() {
  ESP_LOGI(TAG, "TopBand BMS Gateway %s  git=%s  built=%s %s",
           FW_VERSION, GIT_SHA, BUILD_DATE, BUILD_TIME);

  // ── Step 1: NVS ──────────────────────────────────────────────────────────
  if (!init_nvs()) {
    ESP_LOGE(TAG, "NVS init failed — running with defaults (no persistence)");
  }

  // ── Step 1a: Record this boot (NVS must be ready first) ──────────────────
  // Called before Config load so we capture the raw early uptime.
  storage::boot_reasons::record_this_boot();

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
      ESP_LOGE(TAG, "Snapshot bus PSRAM init FAILED — cannot continue safely. Halting.");
      for (;;) { vTaskDelay(pdMS_TO_TICKS(1000)); }
    }
    uint32_t psram_after = heap_caps_get_free_size(MALLOC_CAP_SPIRAM);
    ESP_LOGI(TAG, "Snapshot bus init OK. PSRAM before=%u after=%u allocated=%u B",
             (unsigned)psram_before, (unsigned)psram_after,
             (unsigned)(psram_before - psram_after));
  }

  // ── Step 5: LittleFS ────────────────────────────────────────────────────
  {
    if (storage::lfs::init()) {
      ESP_LOGI(TAG, "LittleFS ready — total=%u free=%u B",
               (unsigned)storage::lfs::total_bytes(),
               (unsigned)storage::lfs::free_bytes());
    } else {
      ESP_LOGE(TAG, "LittleFS unavailable — UI will not function");
    }
  }

  // ── Step 6: UI provisioner ───────────────────────────────────────────────
  storage::ui_provisioner::provision_ui_if_needed();

  // ── Step 7: WiFi init ─────────────────────────────────────────────────────
  bool wifi_init_ok = net::wifi::init();
  if (!wifi_init_ok) {
    ESP_LOGE(TAG, "WiFi subsystem init failed");
  }

  // ── Step 7a: 5x reset detection (requires WiFi init for credential clear) ─
  if (wifi_init_ok && storage::boot_reasons::is_5x_reset_detected()) {
    ESP_LOGW(TAG, "5x rapid reset detected — clearing auth credentials and WiFi");

    Config cleared = g_config;
    cleared.auth_enabled = false;
    cleared.auth_hash[0] = '\0';
    cleared.wifi_ssid[0] = '\0';
    if (storage::saveConfig(cleared)) {
      g_config = cleared;
    } else {
      ESP_LOGE(TAG, "5x reset: Config save failed");
    }

    // Clear esp_wifi NVS credentials (persisted in WiFi driver namespace).
    wifi_config_t empty = {};
    esp_wifi_set_config(WIFI_IF_STA, &empty);

    storage::boot_reasons::clear();
    ESP_LOGW(TAG, "5x reset applied — device will start in captive portal mode");
  }

  // ── Step 8: WiFi STA attempt or captive portal ────────────────────────────
  bool sta_connected = false;
  if (wifi_init_ok) {
    char ssid_check[33] = {};
    bool has_creds = net::wifi::load_ssid(ssid_check, sizeof(ssid_check));

    if (!has_creds) {
      ESP_LOGI(TAG, "No WiFi credentials — starting captive portal");
      enter_captive_portal();
      // Fall through to Step 9 (ControlTask) — BMS polling still runs.
    } else {
      sta_connected = net::wifi::start_sta(30000);
      if (!sta_connected) {
        ESP_LOGW(TAG, "STA connect failed — falling back to captive portal");
        enter_captive_portal();
      } else {
        char ip_buf[24] = {};
        net::wifi::get_ip(ip_buf, sizeof(ip_buf));
        ESP_LOGI(TAG, "WiFi connected — IP %s", ip_buf);
      }
    }
  }

  // ── Step 8.5: History and energy stores (LittleFS must be ready) ─────────
  {
    bool hist_ok = storage::history_store::init();
    bool ener_ok = storage::energy_store::init();
    ESP_LOGI(TAG, "history_store=%s energy_store=%s",
             hist_ok ? "ok" : "FAIL", ener_ok ? "ok" : "FAIL");
  }

  // ── Step 9: Main HTTP server (STA mode only) ──────────────────────────────
  if (sta_connected) {
    web::auth::init();
    if (!web::start_httpd(g_config)) {
      ESP_LOGE(TAG, "HTTP server failed to start");
    }
  }

  // ── Step 9.4: NTP (STA mode — non-blocking, async sync) ──────────────────
  if (sta_connected) {
    if (!net::ntp::start(g_config)) {
      ESP_LOGW(TAG, "NTP start failed");
    }
  }

  // ── Step 9.5: MQTT publisher (STA mode only, when enabled) ──────────────
  if (sta_connected && g_config.mqtt_enabled) {
    if (!mqtt::publisher::start(g_config)) {
      ESP_LOGE(TAG, "MQTT publisher failed to start");
    }
  }

  // ── Step 9.6: Housekeeping task (STA mode only) ───────────────────────────
  if (sta_connected) {
    if (!app::housekeeping::start(g_config)) {
      ESP_LOGE(TAG, "Housekeeping task failed to start");
    }
  }

  // ── Step 9.7: HistoryTask (STA mode only) ────────────────────────────────
  if (sta_connected) {
    if (!app::history_task::start()) {
      ESP_LOGW(TAG, "HistoryTask failed to start — history collection disabled");
    }
  }

  // ── Step 10: Spawn ControlTask ────────────────────────────────────────────
  if (!bms::poller::start(g_config)) {
    ESP_LOGE(TAG, "ControlTask creation failed");
  } else {
    ESP_LOGI(TAG, "ControlTask created on Core 0 (bms_count=%u)", g_config.bms_count);
  }

  // ── Step 11: Smoke reader (Phase C validation — Core 1) ──────────────────
#if SMOKE_READER_ENABLED
  if (sta_connected) {
    app::start_smoke_reader();
    ESP_LOGI(TAG, "Smoke reader task created on Core 1");
  }
#endif

  // ── Step 12: Heartbeat loop ───────────────────────────────────────────────
  ESP_LOGI(TAG, "Boot complete — heartbeat every 5 s");
  for (;;) {
    vTaskDelay(pdMS_TO_TICKS(5000));

    uint32_t uptime_s = (uint32_t)(esp_timer_get_time() / 1000000LL);
    uint32_t heap     = esp_get_free_heap_size();
    uint32_t spiram   = heap_caps_get_free_size(MALLOC_CAP_SPIRAM);

    bms::poller::PollerStats ps{};
    bms::poller::get_stats(ps);

    char ip_buf[24] = {};
    if (net::wifi::is_connected()) {
      net::wifi::get_ip(ip_buf, sizeof(ip_buf));
    } else {
      const char* mode = net::wifi::is_ap_mode() ? "AP-192.168.4.1" : "disconnected";
      snprintf(ip_buf, sizeof(ip_buf), "%s", mode);
    }

    ESP_LOGI(TAG,
             "Heartbeat | uptime=%lu s | heap=%lu B | psram=%lu B | ip=%s | "
             "publishes=%llu | reads=%llu | retries=%llu | "
             "cycle_avg=%lu ms | cycle_max=%lu ms",
             (unsigned long)uptime_s, (unsigned long)heap, (unsigned long)spiram,
             ip_buf,
             (unsigned long long)bus::snapshot_bus::total_publishes(),
             (unsigned long long)bus::snapshot_bus::total_reads(),
             (unsigned long long)bus::snapshot_bus::total_read_retries(),
             (unsigned long)ps.cycle_avg_ms,
             (unsigned long)ps.cycle_max_ms);
  }
}

}  // namespace app
