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
#include "storage/boot_reasons.h"
#include "sources/registry.h"
#include "sources/mppt_source.h"
#include "driver/temperature_sensor.h"
#include "esp_attr.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <cstring>
#include <cstdio>

static const char* TAG = "housekeep";

// ESP32-S3 internal die temperature sensor handle. Installed once before the
// loop. Sentinel value -128.0f means not available.
static temperature_sensor_handle_t s_tsens     = nullptr;
static volatile float              s_cpu_temp_c = -128.0f;

// Large working buffers as module-level statics in PSRAM BSS: BmsSystemSnapshot
// is ~4.6 KB, MqttPublishRequest is ~1.2 KB. CPU/task-context only — no ISR,
// no DMA. Same pattern as history_task.cpp:29 (proven).  HousekeepingTask is
// single-instance so there is no aliasing risk.
// EXT_RAM_BSS_ATTR: moves these from DRAM BSS to PSRAM (~5.8 KB DRAM freed).
static EXT_RAM_BSS_ATTR BmsSystemSnapshot  s_snap;
static SafetyState                         s_safety;  // hot-path internal: keep in DRAM
static EXT_RAM_BSS_ATTR MqttPublishRequest s_req;

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

// ── Staggered-publish cursors ─────────────────────────────────────────────────
// Each 1 Hz tick advances these, distributing up to ~10 publishes per tick
// instead of 43-59 per 5-second DATA burst. Eliminates queue overflow and
// reduces burst load on the MQTT internal task.
// Per docs/diag-mqtt-crash-review.md Findings 2 and 4.
static uint8_t  s_sys_cursor  = 0;   // 0..19  — rotates through 20 system topics
static uint8_t  s_pack_cursor = 0;   // 0..9   — rotates through 10 per-pack value topics
static uint8_t  s_cell_cursor = 0;   // 0..N*15-1 — rotates through individual cell_v topics
static uint32_t s_tick        = 0;   // monotonic 1 Hz counter

static void housekeeping_task_entry(void* /*arg*/) {
  ESP_LOGI(TAG, "HousekeepingTask started on Core 1");

  // Install the ESP32-S3 internal temperature sensor once, on this Core 1 task.
  // Range -10..80 °C covers normal electronics operating conditions.
  {
    temperature_sensor_config_t tsens_cfg = TEMPERATURE_SENSOR_CONFIG_DEFAULT(-10, 80);
    if (temperature_sensor_install(&tsens_cfg, &s_tsens) == ESP_OK) {
      if (temperature_sensor_enable(s_tsens) != ESP_OK) {
        ESP_LOGW(TAG, "temperature_sensor_enable failed — CPU temp unavailable");
        temperature_sensor_uninstall(s_tsens);
        s_tsens = nullptr;
      }
    } else {
      ESP_LOGW(TAG, "temperature_sensor_install failed — CPU temp unavailable");
      s_tsens = nullptr;
    }
  }

  uint32_t last_cells_ms[16]   = {};
  uint32_t last_alert_flush_ms = 0;
  bool     ring_cleared        = false;  // one-shot at ≥ 30 s uptime

  static constexpr uint32_t CELLS_JSON_PERIOD_MS   = 30000;  // Cells JSON blob per pack
  static constexpr uint32_t ALERT_FLUSH_PERIOD_MS  = 300000; // 5 min
  static constexpr uint32_t HEALTHY_UPTIME_S        = 30;

  for (;;) {
    uint32_t now_ms   = (uint32_t)(esp_timer_get_time() / 1000);
    uint32_t uptime_s = (uint32_t)(esp_timer_get_time() / 1000000LL);
    uint64_t ts_ms    = (uint64_t)(esp_timer_get_time() / 1000);
    s_tick++;

    // Read ESP32-S3 die temperature once per 1 Hz tick (diag page refreshes every 5 s).
    if (s_tsens) {
      float t;
      if (temperature_sensor_get_celsius(s_tsens, &t) == ESP_OK) {
        s_cpu_temp_c = t;
      }
    }

    const Config& cfg = app::get_config();

    // ── Healthy-uptime ring clear (once, at ≥ 30 s) ──────────────────────────
    // record_this_boot() always sees ~100 ms at call time (esp_timer resets on
    // every hardware reset), so it cannot detect whether the PREVIOUS boot ran
    // long.  We clear the rapid-reset ring here instead, once we've been running
    // stably for ≥ 30 s — ensuring that legitimate SW restarts (OTA apply, MQTT
    // settings save, self-test timeout) never accumulate toward the 5x-wipe tally.
    if (!ring_cleared && uptime_s >= HEALTHY_UPTIME_S) {
      storage::boot_reasons::mark_healthy();
      ring_cleared = true;
    }

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

    // ── Read snapshot once per tick for all publish paths below ───────────────
    bus::snapshot_bus::read(s_snap);
    bms::poller::read_safety_state(s_safety);

    // ── Data JSON every 5 ticks (5 s) ─────────────────────────────────────────
    if (s_tick % 5 == 0) {
      s_req.topic    = MqttPublishRequest::Topic::Data;
      s_req.pack_id  = 0xFF;
      s_req.retained = false;
      size_t n = mqtt::payloads::build_data(s_snap, s_safety, ts_ms, uptime_s,
                                             s_req.payload, sizeof(s_req.payload));
      if (n > 0) {
        s_req.payload_len = (uint16_t)n;
        post_mqtt(s_req);
      }
    }

    // ── System IndivTopics: refresh all values, post 4 per tick (rotation ~5 s)
    // Fill all 20 values each tick (cheap snprintf); post only the 4 at cursor.
    // Full rotation: 20 topics / 4 per tick = 5 ticks = 5 s per topic.
    {
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
      bms::runtime_estimator::RuntimeStateEst iv_rt_state =
          bms::runtime_estimator::RuntimeStateEst::Idle;
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

      // Post 4 topics at s_sys_cursor, wrapping around 20-element ring.
      for (uint8_t i = 0; i < 4; ++i) {
        uint8_t idx = (s_sys_cursor + i) % 20;
        const IndivTopic& t = s_iv_topics[idx];
        s_req.topic    = MqttPublishRequest::Topic::IndividualValue;
        s_req.pack_id  = 0xFF;
        s_req.retained = true;
        snprintf(s_req.topic_suffix, sizeof(s_req.topic_suffix), "%s", t.suffix);
        size_t vlen = strlen(t.value);
        memcpy(s_req.payload, t.value, vlen + 1);
        s_req.payload_len = (uint16_t)vlen;
        post_mqtt(s_req);
      }
      s_sys_cursor = (s_sys_cursor + 4) % 20;
    }

    // ── Diag JSON every 30 ticks (30 s) ──────────────────────────────────────
    if (cfg.mqtt_diag_enabled && (s_tick % 30 == 0)) {
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
    }

    // ── Solar MPPT IndivTopics every 10 ticks (10 s) ─────────────────────────
    // Published only when BLE MPPT is enabled. When the source is stale (> 30 s)
    // or a field has a Victron sentinel, publish "unavailable" so HA shows the
    // sensor as unavailable rather than the last decoded value.
    if (cfg.ble_mppt_enabled && (s_tick % 10 == 0)) {
      sources::MpptSource* mppt = sources::mppt_source();
      if (mppt && mppt->enabled()) {
        sources::MpptSource::DiagSnap d = mppt->diag_snap();
        bool stale = !d.seen || (mppt->ms_since_last_seen((uint32_t)(esp_timer_get_time() / 1000LL))
                                  > 30000u);   // matches MpptSource::STALE_MS

        auto post_solar = [&](const char* suffix, const char* value) {
          s_req.topic    = MqttPublishRequest::Topic::IndividualValue;
          s_req.pack_id  = 0xFF;
          s_req.retained = true;
          snprintf(s_req.topic_suffix, sizeof(s_req.topic_suffix), "%s", suffix);
          size_t vlen = strlen(value);
          memcpy(s_req.payload, value, vlen + 1);
          s_req.payload_len = (uint16_t)vlen;
          post_mqtt(s_req);
        };

        char sv[32];

        if (stale || !d.pv_power_valid) {
          post_solar("/solar/pv_power", "unavailable");
        } else {
          snprintf(sv, sizeof(sv), "%.1f", d.pv_power_w);
          post_solar("/solar/pv_power", sv);
        }

        if (stale || !d.batt_v_valid) {
          post_solar("/solar/output_voltage", "unavailable");
        } else {
          snprintf(sv, sizeof(sv), "%.2f", d.batt_voltage_v);
          post_solar("/solar/output_voltage", sv);
        }

        if (stale || !d.batt_i_valid) {
          post_solar("/solar/output_current", "unavailable");
        } else {
          snprintf(sv, sizeof(sv), "%.2f", d.batt_current_a);
          post_solar("/solar/output_current", sv);
        }

        if (stale || !d.batt_v_valid || !d.batt_i_valid) {
          post_solar("/solar/output_power", "unavailable");
        } else {
          snprintf(sv, sizeof(sv), "%.1f", d.batt_voltage_v * d.batt_current_a);
          post_solar("/solar/output_power", sv);
        }

        if (stale || !d.yield_valid) {
          post_solar("/solar/yield_today", "unavailable");
        } else {
          snprintf(sv, sizeof(sv), "%.3f", d.yield_today_wh / 1000.0f);
          post_solar("/solar/yield_today", sv);
        }

        if (stale) {
          post_solar("/solar/charger_state", "unavailable");
        } else {
          static const char* CS_LABELS[] = {
            "Off", "Low power", "Fault", "Bulk", "Absorption", "Float",
            "Storage", "Equalize",
          };
          if (d.charge_state < 8) {
            post_solar("/solar/charger_state", CS_LABELS[d.charge_state]);
          } else if (d.charge_state == 252) {
            post_solar("/solar/charger_state", "ESS");
          } else if (d.charge_state == 255) {
            post_solar("/solar/charger_state", "unavailable");
          } else {
            snprintf(sv, sizeof(sv), "State %u", (unsigned)d.charge_state);
            post_solar("/solar/charger_state", sv);
          }
        }
      }
    }

    // ── Per-pack IndivTopics: "online" always + 1 value topic rotating per tick
    // Cadence: online = every tick (1 s), value topics ~11 s each (10 topics / 1 per tick).
    if (cfg.mqtt_level >= Config::MqttLevel::PerPack) {
      uint8_t n_packs = s_snap.pack_count_configured;
      for (uint8_t pi = 0; pi < n_packs && pi < 16; ++pi) {
        const BmsPackSnapshot& pp = s_snap.pack[pi];
        const uint8_t pack_n = pi + 1;

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

        // "online" on every tick so subscribers see state changes promptly.
        post_pack("online", pp.online ? "true" : "false");
        if (!pp.online) continue;

        // Rotate through 10 value topics; char val[32] keeps stack minimal.
        char val[32];
        switch (s_pack_cursor) {
          case 0: snprintf(val, sizeof(val), "%u",   (unsigned)pp.soc);                        post_pack("soc",         val); break;
          case 1: snprintf(val, sizeof(val), "%.2f", pp.pack_voltage);                         post_pack("voltage",     val); break;
          case 2: snprintf(val, sizeof(val), "%.1f", pp.pack_current);                         post_pack("current",     val); break;
          case 3: snprintf(val, sizeof(val), "%d",   (int)(pp.pack_voltage*pp.pack_current));  post_pack("power",       val); break;
          case 4: snprintf(val, sizeof(val), "%.1f", pp.temp_avg_c);                           post_pack("temperature", val); break;
          case 5: snprintf(val, sizeof(val), "%.3f", pp.cell_min_v);                           post_pack("cell_v_min",  val); break;
          case 6: snprintf(val, sizeof(val), "%.3f", pp.cell_max_v);                           post_pack("cell_v_max",  val); break;
          case 7: snprintf(val, sizeof(val), "%.3f", pp.cell_drift_v);                         post_pack("cell_drift",  val); break;
          case 8: snprintf(val, sizeof(val), "%u",   (unsigned)pp.soh);                        post_pack("soh",         val); break;
          case 9: snprintf(val, sizeof(val), "%u",   (unsigned)pp.cycles);                     post_pack("cycles",      val); break;
        }
      }
      s_pack_cursor = (s_pack_cursor + 1) % 10;
    }

    // ── Per-cell: 2 individual cell_v topics per tick + Cells JSON 30 s/pack ──
    // Individual cadence: (n_packs×15 cells / 2 per tick) ticks per full rotation.
    // Cells JSON blob still published every 30 s per pack (feeds value_template).
    if (cfg.mqtt_level >= Config::MqttLevel::PerCell) {
      uint8_t n_packs = s_snap.pack_count_configured;
      if (n_packs > 0) {
        // 2 individual cell_v topics this tick via cursor.
        uint8_t total_slots = n_packs * 15;
        for (uint8_t i = 0; i < 2; ++i) {
          uint8_t slot = (s_cell_cursor + i) % total_slots;
          uint8_t pi   = slot / 15;
          uint8_t ci   = slot % 15;
          if (!s_snap.pack[pi].online) continue;
          if (ci >= s_snap.pack[pi].cell_count) continue;
          const uint8_t pack_n = pi + 1;
          char cval[12];
          snprintf(cval, sizeof(cval), "%.3f", s_snap.pack[pi].cell_v[ci]);
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
        s_cell_cursor = (s_cell_cursor + 2) % total_slots;

        // Cells JSON blob: consider ONE cursor-selected pack per tick; publish
        // when online and due (30 s each). Deliberately no retry with other
        // packs this tick — the cursor advances every second, so a skipped
        // pack is considered again within n_packs ticks. (The previous loop
        // syntax suggested retries but every branch broke out — review F8.)
        {
          uint8_t pi = (s_cell_cursor / 2) % n_packs;  // reuse cursor as pack selector
          bool due = (last_cells_ms[pi] == 0) ||
                     ((now_ms - last_cells_ms[pi]) >= CELLS_JSON_PERIOD_MS);
          if (s_snap.pack[pi].online && due) {
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
          }
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

float get_cpu_temp_c() {
  return s_cpu_temp_c;
}

}  // namespace app::housekeeping
