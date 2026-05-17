#include "app/housekeeping.h"
#include "app/boot.h"
#include "mqtt/payloads.h"
#include "bus/queues.h"
#include "bus/snapshot_bus.h"
#include "bms/poller.h"
#include "can/tx.h"
#include "storage/alerts_store.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <cstring>

static const char* TAG = "housekeep";

// Large working buffers as module-level statics: BmsSystemSnapshot is ~8 KB,
// MqttPublishRequest is ~1 KB. Declaring them as locals in the task function
// causes GCC (-Os) to reserve the full frame at function entry, overflowing
// the 6144-byte stack even when the mqtt_enabled guard short-circuits the loop.
// HousekeepingTask is single-instance so there is no aliasing risk.
static BmsSystemSnapshot s_snap;
static SafetyState       s_safety;
static MqttPublishRequest s_req;

// Helper: post a pre-built MqttPublishRequest to q_mqtt_publish.
// Drops oldest on full queue (architecture §5.8).
// Uses a static receive buffer so the 1030-byte MqttPublishRequest for the
// dropped item does not land on the caller's stack frame.
static void post_mqtt(const MqttPublishRequest& req) {
  if (!q_mqtt_publish) return;
  if (xQueueSend(q_mqtt_publish, &req, 0) != pdTRUE) {
    // Queue full — drop oldest, then re-enqueue.
    static MqttPublishRequest s_dropped;
    xQueueReceive(q_mqtt_publish, &s_dropped, 0);
    xQueueSend(q_mqtt_publish, &req, 0);
    ESP_LOGD(TAG, "q_mqtt_publish full — dropped oldest");
  }
}

static void housekeeping_task_entry(void* /*arg*/) {
  ESP_LOGI(TAG, "HousekeepingTask started on Core 1");

  uint32_t last_data_ms        = 0;
  uint32_t last_diag_ms        = 0;
  uint32_t last_cells_ms[16]   = {};
  uint32_t last_alert_flush_ms = 0;
  uint8_t  cells_rr            = 0;

  static constexpr uint32_t DATA_PERIOD_MS         = 5000;
  static constexpr uint32_t DIAG_PERIOD_MS         = 30000;
  static constexpr uint32_t CELLS_PERIOD_MS        = 20000;
  static constexpr uint32_t ALERT_FLUSH_PERIOD_MS  = 300000;  // 5 min

  for (;;) {
    uint32_t now_ms   = (uint32_t)(esp_timer_get_time() / 1000);
    uint32_t uptime_s = (uint32_t)(esp_timer_get_time() / 1000000LL);
    uint64_t ts_ms    = (uint64_t)(esp_timer_get_time() / 1000);

    const Config& cfg = app::get_config();

    // ── Alert queue drain (always, regardless of MQTT state) ─────────────────
    // Process up to 8 alerts per cycle to bound stack time; use a static
    // buffer to keep the AlertEntry (132 B) off the task stack.
    {
      static AlertEntry s_alert_entry;
      for (int i = 0; i < 8; i++) {
        if (!q_alert) break;
        if (xQueueReceive(q_alert, &s_alert_entry, 0) != pdTRUE) break;
        storage::alerts_store::append(s_alert_entry);
      }
    }

    // ── Alert ring persist every 5 min ────────────────────────────────────────
    if ((now_ms - last_alert_flush_ms) >= ALERT_FLUSH_PERIOD_MS ||
        last_alert_flush_ms == 0) {
      storage::alerts_store::persist();
      last_alert_flush_ms = now_ms;
    }

    if (!cfg.mqtt_enabled) {
      vTaskDelay(pdMS_TO_TICKS(1000));
      continue;
    }

    // ── Data topic every 5 s ──────────────────────────────────────────────────
    if ((now_ms - last_data_ms) >= DATA_PERIOD_MS || last_data_ms == 0) {
      bus::snapshot_bus::read(s_snap);
      bms::poller::read_safety_state(s_safety);

      s_req.topic    = MqttPublishRequest::Topic::Data;
      s_req.pack_id  = 0xFF;
      s_req.retained = false;
      size_t n = mqtt::payloads::build_data(s_snap, s_safety, ts_ms, uptime_s,
                                             s_req.payload, sizeof(s_req.payload));
      if (n > 0) {
        s_req.payload_len = (uint16_t)n;
        post_mqtt(s_req);
      }
      last_data_ms = now_ms;
    }

    // ── Diag topic every 30 s (when enabled) ─────────────────────────────────
    if (cfg.mqtt_diag_enabled &&
        ((now_ms - last_diag_ms) >= DIAG_PERIOD_MS || last_diag_ms == 0)) {
      bus::snapshot_bus::read(s_snap);
      bms::poller::read_safety_state(s_safety);

      bms::poller::PollerStats ps{};
      bms::poller::get_stats(ps);
      can::tx::CanStats cs{};
      can::tx::get_stats(cs);

      s_req.topic    = MqttPublishRequest::Topic::Diag;
      s_req.pack_id  = 0xFF;
      s_req.retained = false;
      size_t n = mqtt::payloads::build_diag(s_snap, s_safety, ps, cs, ts_ms, uptime_s,
                                             s_req.payload, sizeof(s_req.payload));
      if (n > 0) {
        s_req.payload_len = (uint16_t)n;
        post_mqtt(s_req);
      }
      last_diag_ms = now_ms;
    }

    // ── Per-cell publishing every 20 s at PerCell level ───────────────────────
    if (cfg.mqtt_level >= Config::MqttLevel::PerCell) {
      bus::snapshot_bus::read(s_snap);
      uint8_t n_packs = s_snap.pack_count_configured;

      if (n_packs > 0) {
        for (uint8_t tries = 0; tries < n_packs; ++tries) {
          uint8_t pi = cells_rr % n_packs;
          cells_rr = (cells_rr + 1) % n_packs;

          if (!s_snap.pack[pi].online) continue;
          if ((now_ms - last_cells_ms[pi]) < CELLS_PERIOD_MS && last_cells_ms[pi] != 0) continue;

          s_req.topic    = MqttPublishRequest::Topic::Cells;
          s_req.pack_id  = pi;
          s_req.retained = false;
          size_t nb = mqtt::payloads::build_cells(s_snap.pack[pi], ts_ms,
                                                   s_req.payload, sizeof(s_req.payload));
          if (nb > 0) {
            s_req.payload_len = (uint16_t)nb;
            post_mqtt(s_req);
          }
          last_cells_ms[pi] = now_ms;
          break;
        }
      }
    }

    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

namespace app::housekeeping {

bool start(const Config& /*cfg*/) {
  static TaskHandle_t s_handle = nullptr;
  if (s_handle) {
    return true;
  }
  BaseType_t r = xTaskCreatePinnedToCore(
      housekeeping_task_entry, "housekeep",
      4096, nullptr,
      /*priority*/ 1,
      &s_handle,
      /*core*/ 1);
  if (r != pdPASS) {
    ESP_LOGE(TAG, "xTaskCreatePinnedToCore failed for HousekeepingTask");
    return false;
  }
  return true;
}

}  // namespace app::housekeeping
