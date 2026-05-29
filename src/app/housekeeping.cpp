#include "app/housekeeping.h"
#include "app/boot.h"
#include "mqtt/payloads.h"
#include "bus/queues.h"
#include "bus/snapshot_bus.h"
#include "bms/poller.h"
#include "bms/runtime_estimator.h"
#include "can/tx.h"
#include "storage/alerts_store.h"
#include "storage/energy_store.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <cstring>
#include <cstdio>

static const char* TAG = "housekeep";

// Large working buffers as module-level statics: BmsSystemSnapshot is ~8 KB,
// MqttPublishRequest is ~1 KB. Declaring them as locals in the task function
// causes GCC (-Os) to reserve the full frame at function entry, overflowing
// the 6144-byte stack even when the mqtt_enabled guard short-circuits the loop.
// HousekeepingTask is single-instance so there is no aliasing risk.
static BmsSystemSnapshot  s_snap;
static SafetyState        s_safety;
static MqttPublishRequest s_req;

// Individual system IndivTopic values — 20 × 72 bytes = 1440 bytes.
// Previously a stack-local inside housekeeping_task_entry; with interrupt save
// frames, nested call frames, and log_hook overhead that brought HousekeepingTask
// within ~1400 bytes of its 4096-byte budget. Same BSS-promotion pattern as the
// Phase H1 stack overflow fix (architecture §10 R9).
// Per docs/diag-mqtt-crash-review.md Finding 3.
struct IndivTopic { const char* suffix; char value[64]; };
static IndivTopic s_iv_topics[20] = {
  { "/soc",              {} },
  { "/voltage",          {} },
  { "/current",          {} },
  { "/power",            {} },
  { "/temperature",      {} },
  { "/cell_v_min",       {} },
  { "/cell_v_max",       {} },
  { "/cell_drift",       {} },
  { "/soh",              {} },
  { "/cvl",              {} },
  { "/ccl",              {} },
  { "/dcl",              {} },
  { "/alarm_flags",      {} },
  { "/sys_message",      {} },
  { "/bms_online",       {} },
  { "/bms_configured",   {} },
  { "/energy_today_in",  {} },
  { "/energy_today_out", {} },
  { "/runtime_est_min",  {} },
  { "/runtime_est_state",{} },
};

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

      // ── Individual plain-text topics (retained, for HA Discovery) ─────────
      // Aggregate cell_v_min/max/drift across online packs.
      float iv_cell_min = 0.0f, iv_cell_max = 0.0f, iv_cell_drift = 0.0f;
      bool  iv_have_cells = false;
      for (uint8_t i = 0; i < s_snap.pack_count_configured && i < 16; ++i) {
        const BmsPackSnapshot& pp = s_snap.pack[i];
        if (!pp.online) continue;
        if (!iv_have_cells) {
          iv_cell_min   = pp.cell_min_v;
          iv_cell_max   = pp.cell_max_v;
          iv_cell_drift = pp.cell_drift_v;
          iv_have_cells = true;
        } else {
          if (pp.cell_min_v   < iv_cell_min)   iv_cell_min   = pp.cell_min_v;
          if (pp.cell_max_v   > iv_cell_max)   iv_cell_max   = pp.cell_max_v;
          if (pp.cell_drift_v > iv_cell_drift) iv_cell_drift = pp.cell_drift_v;
        }
      }

      int32_t iv_power = (int32_t)(s_safety.pack_voltage_avg * s_safety.pack_current_total);

      bms::runtime_estimator::RuntimeStateEst iv_rt_state = bms::runtime_estimator::RuntimeStateEst::Idle;
      int32_t iv_rt_min = bms::runtime_estimator::estimate_min(s_safety, iv_rt_state);

      const char* iv_rt_state_str =
        (iv_rt_state == bms::runtime_estimator::RuntimeStateEst::UntilEmpty) ? "until_empty" :
        (iv_rt_state == bms::runtime_estimator::RuntimeStateEst::UntilFull)  ? "until_full"  : "idle";

      // Fill value strings into s_iv_topics[] (indices match module-level array).
      snprintf(s_iv_topics[0].value,  sizeof(s_iv_topics[0].value),  "%u",    (unsigned)s_safety.soc_avg);
      snprintf(s_iv_topics[1].value,  sizeof(s_iv_topics[1].value),  "%.2f",  s_safety.pack_voltage_avg);
      snprintf(s_iv_topics[2].value,  sizeof(s_iv_topics[2].value),  "%.1f",  s_safety.pack_current_total);
      snprintf(s_iv_topics[3].value,  sizeof(s_iv_topics[3].value),  "%d",    (int)iv_power);
      snprintf(s_iv_topics[4].value,  sizeof(s_iv_topics[4].value),  "%.1f",  s_safety.temp_avg);
      if (iv_have_cells) {
        snprintf(s_iv_topics[5].value, sizeof(s_iv_topics[5].value), "%.3f", iv_cell_min);
        snprintf(s_iv_topics[6].value, sizeof(s_iv_topics[6].value), "%.3f", iv_cell_max);
        snprintf(s_iv_topics[7].value, sizeof(s_iv_topics[7].value), "%.3f", iv_cell_drift);
      } else {
        snprintf(s_iv_topics[5].value, sizeof(s_iv_topics[5].value), "0.000");
        snprintf(s_iv_topics[6].value, sizeof(s_iv_topics[6].value), "0.000");
        snprintf(s_iv_topics[7].value, sizeof(s_iv_topics[7].value), "0.000");
      }
      snprintf(s_iv_topics[8].value,  sizeof(s_iv_topics[8].value),  "%u",    (unsigned)s_safety.soh_avg);
      snprintf(s_iv_topics[9].value,  sizeof(s_iv_topics[9].value),  "%.1f",  s_safety.cvl_volts);
      snprintf(s_iv_topics[10].value, sizeof(s_iv_topics[10].value), "%u",    (unsigned)s_safety.ccl_amps);
      snprintf(s_iv_topics[11].value, sizeof(s_iv_topics[11].value), "%u",    (unsigned)s_safety.dcl_amps);
      snprintf(s_iv_topics[12].value, sizeof(s_iv_topics[12].value), "%u",    (unsigned)s_safety.alarm_flags);
      snprintf(s_iv_topics[13].value, sizeof(s_iv_topics[13].value), "%s",    s_safety.sys_message);
      snprintf(s_iv_topics[14].value, sizeof(s_iv_topics[14].value), "%u",    (unsigned)s_safety.packs_online);
      snprintf(s_iv_topics[15].value, sizeof(s_iv_topics[15].value), "%u",    (unsigned)s_safety.packs_configured);
      snprintf(s_iv_topics[16].value, sizeof(s_iv_topics[16].value), "%.2f",  storage::energy_store::today_in_kwh());
      snprintf(s_iv_topics[17].value, sizeof(s_iv_topics[17].value), "%.2f",  storage::energy_store::today_out_kwh());
      snprintf(s_iv_topics[18].value, sizeof(s_iv_topics[18].value), "%d",    (int)iv_rt_min);
      snprintf(s_iv_topics[19].value, sizeof(s_iv_topics[19].value), "%s",    iv_rt_state_str);

      for (auto& t : s_iv_topics) {
        s_req.topic    = MqttPublishRequest::Topic::IndividualValue;
        s_req.pack_id  = 0xFF;
        s_req.retained = true;
        snprintf(s_req.topic_suffix, sizeof(s_req.topic_suffix), "%s", t.suffix);
        size_t vlen = strlen(t.value);
        memcpy(s_req.payload, t.value, vlen + 1);
        s_req.payload_len = (uint16_t)vlen;
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

    // ── Per-pack plain-text retained topics every 5 s at PerPack level ─────────
    // Published together with the system data block above.
    // When a pack is offline only "online=false" is published; retained values
    // from the last online period remain available to subscribers.
    if (cfg.mqtt_level >= Config::MqttLevel::PerPack) {
      uint8_t n_packs = s_snap.pack_count_configured;
      for (uint8_t pi = 0; pi < n_packs && pi < 16; ++pi) {
        const BmsPackSnapshot& pp = s_snap.pack[pi];
        const uint8_t  pack_n = pi + 1;     // 1-indexed label

        // Helper: build and post a single retained IndividualValue publish.
        // Reuses the module-level s_req to keep the per-pack loop off the stack.
        auto post_pack = [&](const char* suffix, const char* value) {
          s_req.topic    = MqttPublishRequest::Topic::IndividualValue;
          s_req.pack_id  = pi;
          s_req.retained = true;
          snprintf(s_req.topic_suffix, sizeof(s_req.topic_suffix),
                   "/pack%u/%s", (unsigned)pack_n, suffix);
          size_t vlen = strlen(value);
          memcpy(s_req.payload, value, vlen + 1);
          s_req.payload_len = (uint16_t)vlen;
          post_mqtt(s_req);
        };

        // "online" is always published (tells subscribers whether pack is active).
        post_pack("online", pp.online ? "true" : "false");

        if (!pp.online) continue;

        // Remaining values only when online; char val[32] keeps stack minimal.
        char val[32];
        snprintf(val, sizeof(val), "%u",   (unsigned)pp.soc);
        post_pack("soc", val);
        snprintf(val, sizeof(val), "%.2f", pp.pack_voltage);
        post_pack("voltage", val);
        snprintf(val, sizeof(val), "%.1f", pp.pack_current);
        post_pack("current", val);
        snprintf(val, sizeof(val), "%d",
                 (int)(pp.pack_voltage * pp.pack_current));
        post_pack("power", val);
        // Use temp_avg_c (computed by fill_from_analog) as representative temp.
        snprintf(val, sizeof(val), "%.1f", pp.temp_avg_c);
        post_pack("temperature", val);
        snprintf(val, sizeof(val), "%.3f", pp.cell_min_v);
        post_pack("cell_v_min", val);
        snprintf(val, sizeof(val), "%.3f", pp.cell_max_v);
        post_pack("cell_v_max", val);
        snprintf(val, sizeof(val), "%.3f", pp.cell_drift_v);
        post_pack("cell_drift", val);
        snprintf(val, sizeof(val), "%u",   (unsigned)pp.soh);
        post_pack("soh", val);
        snprintf(val, sizeof(val), "%u",   (unsigned)pp.cycles);
        post_pack("cycles", val);
      }
    }

    // ── Per-cell JSON publishing every 20 s at PerCell level ────────────────────
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

          // Individual per-cell retained plain-text topics: {base}/pack{N}/cell_v_{NN}
          {
            const BmsPackSnapshot& pcell = s_snap.pack[pi];
            const uint8_t pack_n  = pi + 1;
            const uint8_t cell_lim = (pcell.cell_count < 16) ? pcell.cell_count : 16;
            for (uint8_t ci = 0; ci < cell_lim; ++ci) {
              char cval[12];
              snprintf(cval, sizeof(cval), "%.3f", pcell.cell_v[ci]);
              s_req.topic    = MqttPublishRequest::Topic::IndividualValue;
              s_req.pack_id  = pi;
              s_req.retained = true;
              snprintf(s_req.topic_suffix, sizeof(s_req.topic_suffix),
                       "/pack%u/cell_v_%02u", (unsigned)pack_n, (unsigned)(ci + 1));
              size_t vlen = strlen(cval);
              memcpy(s_req.payload, cval, vlen + 1);
              s_req.payload_len = (uint16_t)vlen;
              post_mqtt(s_req);
            }
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
