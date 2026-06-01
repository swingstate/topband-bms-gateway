#include "boot_reasons.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "nvs.h"
#include <cstring>

static const char* TAG = "boot_reasons";

static constexpr const char* NVS_NS   = "boot_rsns";
static constexpr const char* KEY_RING = "ring";
static constexpr size_t      RING_SIZE = 5;
// Uptime threshold: a SW reset is "quick" (deliberate rapid-press) only when
// the device hadn't been running long enough to be in normal operation.
static constexpr uint32_t RAPID_THRESHOLD_MS  = 5000;
// A SW reset that follows a long-running boot is a normal operation restart
// (OTA apply, settings save). Clear the ring rather than accumulate.
static constexpr uint32_t NORMAL_BOOT_MS = 30000;

// In-RAM copy of the NVS ring. 0 = empty (never written).
static uint32_t g_ring[RING_SIZE] = {};
static bool g_loaded = false;

static void load_ring() {
  if (g_loaded) return;
  nvs_handle_t h;
  if (nvs_open(NVS_NS, NVS_READONLY, &h) == ESP_OK) {
    size_t len = sizeof(g_ring);
    nvs_get_blob(h, KEY_RING, g_ring, &len);
    nvs_close(h);
  }
  g_loaded = true;
}

static void save_ring() {
  nvs_handle_t h;
  esp_err_t err = nvs_open(NVS_NS, NVS_READWRITE, &h);
  if (err != ESP_OK) {
    ESP_LOGW(TAG, "nvs_open failed: %s", esp_err_to_name(err));
    return;
  }
  nvs_set_blob(h, KEY_RING, g_ring, sizeof(g_ring));
  nvs_commit(h);
  nvs_close(h);
}

namespace storage::boot_reasons {

void record_this_boot() {
  esp_reset_reason_t reason = esp_reset_reason();

  // Only a deliberate software restart (esp_restart() → ESP_RST_SW) counts
  // toward the rapid-reset tally.  Every other reset reason is excluded:
  //
  //  POWERON / BROWNOUT: mains power event.  A mains-powered BMS gateway must
  //    survive any number of power outages without losing WiFi/config.
  //
  //  PANIC / INT_WDT / TASK_WDT / WDT: firmware crash.  A crashing device must
  //    NOT also lose its WiFi credentials — that compounds one failure into two
  //    and prevents the operator from reaching the Web UI to diagnose the crash.
  //    Root-cause P3: TLS-under-load crash (ESP_RST_PANIC) was filling the ring
  //    and triggering a WiFi wipe.
  //
  //  All other reasons (UNKNOWN, EXT, DEEPSLEEP, SDIO): excluded conservatively.
  if (reason != ESP_RST_SW) {
    ESP_LOGD(TAG,
             "record_this_boot: reset_reason=%d — excluded from rapid-reset tally "
             "(only ESP_RST_SW counts; crashes/WDT/power-events never wipe config)",
             (int)reason);
    return;
  }

  uint32_t uptime_ms = (uint32_t)(esp_timer_get_time() / 1000ULL);

  // A SW reset that follows a long-running boot is a normal operation restart
  // (OTA apply, settings save). Clear the ring so routine restarts never
  // accumulate toward the factory-reset threshold.
  if (uptime_ms >= NORMAL_BOOT_MS) {
    load_ring();
    memset(g_ring, 0, sizeof(g_ring));
    g_loaded = true;
    save_ring();
    ESP_LOGD(TAG, "record_this_boot: SW restart after %u ms (normal) — ring cleared",
             (unsigned)uptime_ms);
    return;
  }

  // Quick SW reset (uptime < 30 s): counts toward deliberate rapid-reset gesture.
  ESP_LOGI(TAG, "record_this_boot: uptime=%u ms (SW reset, quick — counting)",
           (unsigned)uptime_ms);
  load_ring();
  memmove(g_ring, g_ring + 1, sizeof(uint32_t) * (RING_SIZE - 1));
  g_ring[RING_SIZE - 1] = uptime_ms;
  save_ring();
}

bool is_5x_reset_detected() {
  load_ring();
  for (size_t i = 0; i < RING_SIZE; i++) {
    if (g_ring[i] == 0 || g_ring[i] >= RAPID_THRESHOLD_MS) {
      return false;
    }
  }
  ESP_LOGW(TAG, "5x rapid reset detected (all 5 uptimes < %u ms)",
           (unsigned)RAPID_THRESHOLD_MS);
  return true;
}

void clear() {
  memset(g_ring, 0, sizeof(g_ring));
  g_loaded = true;
  save_ring();
  ESP_LOGI(TAG, "boot_reasons ring cleared");
}

}  // namespace storage::boot_reasons
