#include "app/housekeeping.h"
#include "app/boot.h"
#include "mqtt/payloads.h"
#include "bus/queues.h"
#include "bus/snapshot_bus.h"
#include "bms/poller.h"
#include "can/tx.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <cstring>

static const char* TAG = "housekeep";

// Helper: post a pre-built MqttPublishRequest to q_mqtt_publish.
// Drops oldest on full queue (architecture §5.8).
static void post_mqtt(const MqttPublishRequest& req) {
  if (!q_mqtt_publish) return;
  if (xQueueSend(q_mqtt_publish, &req, 0) != pdTRUE) {
    // Queue full — drop oldest, then re-enqueue.
    MqttPublishRequest dropped;
    xQueueReceive(q_mqtt_publish, &dropped, 0);
    xQueueSend(q_mqtt_publish, &req, 0);
    ESP_LOGD(TAG, "q_mqtt_publish full — dropped oldest");
  }
}

static void housekeeping_task_entry(void* /*arg*/) {
  ESP_LOGI(TAG, "HousekeepingTask started on Core 1");

  uint32_t last_data_ms       = 0;
  uint32_t last_diag_ms       = 0;
  uint32_t last_cells_ms[16]  = {};
  uint8_t  cells_rr           = 0;   // round-robin pack index for per-cell publish

  static constexpr uint32_t DATA_PERIOD_MS  = 5000;
  static constexpr uint32_t DIAG_PERIOD_MS  = 30000;
  static constexpr uint32_t CELLS_PERIOD_MS = 20000;

  for (;;) {
    uint32_t now_ms  = (uint32_t)(esp_timer_get_time() / 1000);
    uint32_t uptime_s = (uint32_t)(esp_timer_get_time() / 1000000LL);
    uint64_t ts_ms   = (uint64_t)(esp_timer_get_time() / 1000);

    const Config& cfg = app::get_config();

    if (!cfg.mqtt_enabled) {
      vTaskDelay(pdMS_TO_TICKS(1000));
      continue;
    }

    // ── Data topic every 5 s ──────────────────────────────────────────────────
    if ((now_ms - last_data_ms) >= DATA_PERIOD_MS || last_data_ms == 0) {
      BmsSystemSnapshot snap{};
      SafetyState       safety{};
      bus::snapshot_bus::read(snap);
      bms::poller::read_safety_state(safety);

      MqttPublishRequest req{};
      req.topic      = MqttPublishRequest::Topic::Data;
      req.pack_id    = 0xFF;
      req.retained   = false;
      size_t n = mqtt::payloads::build_data(snap, safety, ts_ms, uptime_s,
                                             req.payload, sizeof(req.payload));
      if (n > 0) {
        req.payload_len = (uint16_t)n;
        post_mqtt(req);
      }
      last_data_ms = now_ms;
    }

    // ── Diag topic every 30 s (when enabled) ─────────────────────────────────
    if (cfg.mqtt_diag_enabled &&
        ((now_ms - last_diag_ms) >= DIAG_PERIOD_MS || last_diag_ms == 0)) {
      BmsSystemSnapshot snap{};
      SafetyState       safety{};
      bus::snapshot_bus::read(snap);
      bms::poller::read_safety_state(safety);

      bms::poller::PollerStats ps{};
      bms::poller::get_stats(ps);
      can::tx::CanStats cs{};
      can::tx::get_stats(cs);

      MqttPublishRequest req{};
      req.topic      = MqttPublishRequest::Topic::Diag;
      req.pack_id    = 0xFF;
      req.retained   = false;
      size_t n = mqtt::payloads::build_diag(snap, safety, ps, cs, ts_ms, uptime_s,
                                             req.payload, sizeof(req.payload));
      if (n > 0) {
        req.payload_len = (uint16_t)n;
        post_mqtt(req);
      }
      last_diag_ms = now_ms;
    }

    // ── Per-cell publishing every 20 s at PerCell level ───────────────────────
    if (cfg.mqtt_level >= Config::MqttLevel::PerCell) {
      BmsSystemSnapshot snap{};
      bus::snapshot_bus::read(snap);
      uint8_t n_packs = snap.pack_count_configured;

      if (n_packs > 0) {
        // Round-robin: find the next pack that is due (at most one per tick)
        for (uint8_t tries = 0; tries < n_packs; ++tries) {
          uint8_t pi = cells_rr % n_packs;
          cells_rr = (cells_rr + 1) % n_packs;

          if (!snap.pack[pi].online) continue;
          if ((now_ms - last_cells_ms[pi]) < CELLS_PERIOD_MS && last_cells_ms[pi] != 0) continue;

          MqttPublishRequest req{};
          req.topic    = MqttPublishRequest::Topic::Cells;
          req.pack_id  = pi;
          req.retained = false;
          size_t nb = mqtt::payloads::build_cells(snap.pack[pi], ts_ms,
                                                   req.payload, sizeof(req.payload));
          if (nb > 0) {
            req.payload_len = (uint16_t)nb;
            post_mqtt(req);
          }
          last_cells_ms[pi] = now_ms;
          break;  // one pack per tick
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
    // Already running — idempotent
    return true;
  }
  BaseType_t r = xTaskCreatePinnedToCore(
      housekeeping_task_entry, "housekeep",
      6144, nullptr,
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
