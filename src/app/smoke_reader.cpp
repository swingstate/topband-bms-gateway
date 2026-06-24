#include "app/smoke_reader.h"

#if SMOKE_READER_ENABLED

#include "bus/snapshot_bus.h"
#include "bms/poller.h"
#include "can/tx.h"
#include "esp_attr.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char* TAG = "smoke";

static void smoke_reader_task(void* /*param*/) {
  ESP_LOGI(TAG, "Smoke reader task started on Core 1");
  // BmsSystemSnapshot (~4.6 KB) moved to PSRAM BSS — CPU/task-context only,
  // no ISR, no DMA. Same pattern as history_task.cpp:29 (proven).
  static EXT_RAM_BSS_ATTR BmsSystemSnapshot snap;
  static SafetyState safety;  // small; keep in DRAM

  for (;;) {
    vTaskDelay(pdMS_TO_TICKS(100));

    if (!bus::snapshot_bus::read(snap)) {
      ESP_LOGI(TAG, "no snapshot yet (publishes=%llu)",
               (unsigned long long)bus::snapshot_bus::total_publishes());
      continue;
    }

    // One-line summary: cycle, online count, pack[0] data.
    float p0v = snap.pack[0].pack_voltage;
    float p0c = snap.pack[0].pack_current;
    float p0cell = (snap.pack[0].cell_count > 0) ? snap.pack[0].cell_v[0] : 0.0f;

    ESP_LOGI(TAG,
             "cycle=%" PRIu32 " online=%u/%u "
             "pack0_v=%.3f pack0_c=%.2f first_cell=%.3f "
             "retries=%llu",
             (unsigned long)snap.cycle_id,
             snap.pack_count_online,
             snap.pack_count_configured,
             p0v, p0c, p0cell,
             (unsigned long long)bus::snapshot_bus::total_read_retries());

    if (bms::poller::read_safety_state(safety)) {
      can::tx::CanStats cs{};
      can::tx::get_stats(cs);
      ESP_LOGI(TAG,
               "safety: flags=0x%02X ccl=%.0f dcl=%.0f cvl=%.2f packs=%u msg=%s",
               safety.alarm_flags,
               safety.ccl_amps, safety.dcl_amps, safety.cvl_volts,
               safety.packs_online, safety.sys_message);
      ESP_LOGI(TAG,
               "can: ok=%llu fail=%llu hbeat=%lu exprss=%lu busoff=%lu",
               (unsigned long long)cs.tx_ok,
               (unsigned long long)cs.tx_fail,
               (unsigned long)cs.heartbeats,
               (unsigned long)cs.express_sends,
               (unsigned long)cs.bus_off_count);
    }
  }
}

namespace app {

void start_smoke_reader() {
  static TaskHandle_t handle = nullptr;
  BaseType_t r = xTaskCreatePinnedToCore(
      smoke_reader_task,
      "smoke",
      /*stack*/ 4096,
      nullptr,
      /*priority*/ 1,
      &handle,
      /*core*/ 1
  );
  if (r != pdPASS) {
    ESP_LOGE(TAG, "Failed to create smoke reader task");
  }
}

}  // namespace app

#endif  // SMOKE_READER_ENABLED
