#include "bms/poller.h"
#include "app/self_test.h"
#include "bms/snapshot.h"
#include "bms/protocol.h"
#include "bms/energy_integrator.h"
#include "bus/snapshot_bus.h"
#include "safety/runSafety.h"
#include "safety_state.h"
#include "can/tx.h"
#include "can/busoff.h"
#include "bus/queues.h"
#include "mqtt/payloads.h"
#include "net/ntp.h"
#include "app/boot.h"
#include "diag/alerts.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <cstring>

static const char* TAG = "poller";

// ── Module-private state ─────────────────────────────────────────────────────

static portMUX_TYPE  s_stats_mux   = portMUX_INITIALIZER_UNLOCKED;
static bms::poller::PollerStats s_stats{};

// Per-pack online state for alert edge detection.
static bool s_pack_was_online[16] = {};
// Last safety alarm_flags for all-clear edge detection.
static uint8_t s_last_alarm_flags = 0;

// ── Safety state (Phase D) ────────────────────────────────────────────────────
// Single-slot with a critical-section copy. Phase E can upgrade to seqlock.
static SafetyState    s_safety{};
static bool           s_safety_valid = false;
static portMUX_TYPE   s_safety_mux   = portMUX_INITIALIZER_UNLOCKED;
static PrevSafetyState s_safety_prev{};

// ── RS485 helpers ─────────────────────────────────────────────────────────────

// UART number used for RS485. Hardcoded to UART1 per architecture §4.3.
static constexpr uart_port_t RS485_UART = UART_NUM_1;

// Drive the RS485 direction pin. Positive level = TX, zero level = RX.
// Skipped when dir_pin < 0 (hardware auto-direction mode).
static inline void rs485_set_dir(int8_t dir_pin, int level) {
  if (dir_pin >= 0) {
    gpio_set_level(static_cast<gpio_num_t>(dir_pin), level);
  }
}

// Configure UART1 and (optionally) the direction GPIO.
static bool rs485_init(const Config& cfg) {
  uart_config_t ucfg{};  // zero-init all fields (including flags.*)
  ucfg.baud_rate    = 9600;
  ucfg.data_bits    = UART_DATA_8_BITS;
  ucfg.parity       = UART_PARITY_DISABLE;
  ucfg.stop_bits    = UART_STOP_BITS_1;
  ucfg.flow_ctrl    = UART_HW_FLOWCTRL_DISABLE;
  ucfg.source_clk   = UART_SCLK_DEFAULT;

  esp_err_t r;
  r = uart_driver_install(RS485_UART, 512, 0, 0, nullptr, 0);
  if (r != ESP_OK) {
    ESP_LOGE(TAG, "uart_driver_install failed: %s", esp_err_to_name(r));
    return false;
  }
  r = uart_param_config(RS485_UART, &ucfg);
  if (r != ESP_OK) { ESP_LOGE(TAG, "uart_param_config: %s", esp_err_to_name(r)); return false; }

  r = uart_set_pin(RS485_UART,
                   cfg.pins.rs485_tx,
                   cfg.pins.rs485_rx,
                   UART_PIN_NO_CHANGE,
                   UART_PIN_NO_CHANGE);
  if (r != ESP_OK) { ESP_LOGE(TAG, "uart_set_pin: %s", esp_err_to_name(r)); return false; }

  if (cfg.pins.rs485_dir < 0) {
    // Hardware auto-direction (LilyGo T-CAN485)
    r = uart_set_mode(RS485_UART, UART_MODE_RS485_HALF_DUPLEX);
    if (r != ESP_OK) { ESP_LOGE(TAG, "uart_set_mode RS485_HALF_DUPLEX: %s", esp_err_to_name(r)); return false; }
    ESP_LOGI(TAG, "RS485 UART1 TX=%d RX=%d dir=auto (HW half-duplex)", cfg.pins.rs485_tx, cfg.pins.rs485_rx);
  } else {
    // Explicit direction pin (Waveshare)
    r = uart_set_mode(RS485_UART, UART_MODE_UART);
    if (r != ESP_OK) { ESP_LOGE(TAG, "uart_set_mode UART: %s", esp_err_to_name(r)); return false; }

    gpio_config_t gcfg = {
      .pin_bit_mask = (1ULL << cfg.pins.rs485_dir),
      .mode         = GPIO_MODE_OUTPUT,
      .pull_up_en   = GPIO_PULLUP_DISABLE,
      .pull_down_en = GPIO_PULLDOWN_DISABLE,
      .intr_type    = GPIO_INTR_DISABLE,
    };
    gpio_config(&gcfg);
    gpio_set_level(static_cast<gpio_num_t>(cfg.pins.rs485_dir), 0);  // RX mode at start
    ESP_LOGI(TAG, "RS485 UART1 TX=%d RX=%d DIR=%d (manual)", cfg.pins.rs485_tx, cfg.pins.rs485_rx, cfg.pins.rs485_dir);
  }
  return true;
}

// Send a BMS request frame for the given address and CID2 type.
// Returns true if transmit completed without error.
static bool rs485_send_request(int8_t dir_pin, uint8_t bms_addr, uint8_t cid2) {
  uint8_t frame[32];
  size_t len = bms::protocol::build_request(bms_addr, cid2, nullptr, 0, frame, sizeof(frame));
  if (len == 0) {
    ESP_LOGW(TAG, "build_request returned 0 for addr=%u cid2=0x%02X", bms_addr, cid2);
    return false;
  }

  rs485_set_dir(dir_pin, 1);  // Enable TX
  int written = uart_write_bytes(RS485_UART, frame, len);
  if (written != static_cast<int>(len)) {
    rs485_set_dir(dir_pin, 0);
    ESP_LOGW(TAG, "uart_write_bytes short write: %d/%zu", written, len);
    return false;
  }
  esp_err_t e = uart_wait_tx_done(RS485_UART, pdMS_TO_TICKS(100));
  rs485_set_dir(dir_pin, 0);  // Enable RX

  // Flush any echo that might have appeared in the RX buffer during TX.
  uart_flush_input(RS485_UART);
  return (e == ESP_OK);
}

// Receive one response frame (SOI…EOI) within timeout_ms.
// Returns true and fills buf/out_len on success.
static bool rs485_receive_frame(uint8_t* buf, size_t buf_size, size_t& out_len) {
  out_len = 0;
  const int64_t deadline_us = esp_timer_get_time()
                              + static_cast<int64_t>(bms::poller::RS485_FRAME_TIMEOUT_MS) * 1000;

  // Wait for SOI byte (0x7E)
  while (esp_timer_get_time() < deadline_us) {
    uint8_t b;
    int r = uart_read_bytes(RS485_UART, &b, 1, pdMS_TO_TICKS(10));
    if (r > 0 && b == bms::protocol::TB_SOI) {
      buf[out_len++] = b;
      break;
    }
  }
  if (out_len == 0) return false;  // SOI never arrived

  // Read until EOI (0x0D) or timeout. 0x0D cannot appear inside the ASCII-hex
  // payload (that would be encoded as '0','D'), so EOI detection is unambiguous.
  while (esp_timer_get_time() < deadline_us) {
    if (out_len >= buf_size) return false;  // Frame larger than buffer
    uint8_t b;
    int r = uart_read_bytes(RS485_UART, &b, 1,
                            pdMS_TO_TICKS(bms::poller::RS485_INTER_BYTE_TIMEOUT_MS));
    if (r <= 0) return false;  // Inter-byte timeout — frame incomplete
    buf[out_len++] = b;
    if (b == bms::protocol::TB_EOI) return true;
  }
  return false;  // Total timeout
}

// (Phase E: can::tx::init() called at ControlTask startup; can_tx_if_due() called per tick)

// Post one alarm event to q_mqtt_publish. Separated from control_task_entry
// so the 1030-byte MqttPublishRequest never lands on the ControlTask frame
// when MQTT is disabled (GCC reserves locals at function entry with -Os).
// Called only when cfg.mqtt_enabled is true.
static void route_alarm_event_to_mqtt(const SafetyState::EventEntry& entry,
                                       uint64_t ts_ms) {
  if (!q_mqtt_publish) return;
  MqttPublishRequest req{};
  req.topic    = MqttPublishRequest::Topic::Alarm;
  req.pack_id  = entry.bms_id;
  req.retained = false;
  size_t n = mqtt::payloads::build_alarm_event(entry, ts_ms,
                                                req.payload, sizeof(req.payload));
  if (n > 0) {
    req.payload_len = static_cast<uint16_t>(n);
    xQueueSend(q_mqtt_publish, &req, 0);
  }
}

// ── ControlTask ──────────────────────────────────────────────────────────────

static void control_task_entry(void* param) {
  const Config& cfg = *static_cast<const Config*>(param);

  ESP_LOGI(TAG, "ControlTask started on Core 0 (bms_count=%u)", cfg.bms_count);

  s_safety_prev = safety::make_default_prev();

  if (!rs485_init(cfg)) {
    ESP_LOGE(TAG, "RS485 init failed — ControlTask aborting");
    vTaskDelete(nullptr);
    return;
  }

  // ── CAN TX driver init (Phase E) ─────────────────────────────────────────
  if (!can::tx::init(cfg)) {
    ESP_LOGW(TAG, "CAN TX init failed — CAN frames will not be sent");
    // Non-fatal: RS485 BMS polling continues; inverter will timeout gracefully.
  }

  // ── Initial snapshot: all packs offline ──────────────────────────────────
  {
    BmsSystemSnapshot* sys = bus::snapshot_bus::begin_publish();
    sys->cycle_id           = 0;
    sys->pack_count_configured = cfg.bms_count;
    sys->pack_count_online     = 0;
    sys->produced_ms           = static_cast<uint32_t>(esp_timer_get_time() / 1000);
    for (uint8_t i = 0; i < 16; ++i) bms::init_pack_snapshot_offline(sys->pack[i], i);
    bus::snapshot_bus::publish();
    ESP_LOGI(TAG, "Initial snapshot published (all packs offline)");
    app::self_test::mark_passed(app::self_test::SNAPSHOT_PUBLISHED);
  }

  // ── Loop state ───────────────────────────────────────────────────────────
  uint32_t cycle_id         = 0;
  uint32_t last_cycle_start = 0;  // ms timestamp of last analog cycle start
  // Round-robin state for alarm/sysparam polling.
  // Alternates per cycle: even cycle → alarm for pack rr_pack,
  //                        odd cycle → sysparam for pack rr_pack,
  // then rr_pack advances each cycle.
  uint8_t  rr_pack   = 0;
  bool     rr_alarm  = true;   // true = alarm this cycle, false = sysparam
  uint8_t  rx_buf[256];

  // Stats accumulator (local, written to s_stats under critical section per cycle)
  bms::poller::PollerStats local_stats{};

  TickType_t  tick_start = xTaskGetTickCount();
  app::self_test::mark_passed(app::self_test::CONTROLTASK_ALIVE);

  for (;;) {
    uint32_t now_ms = static_cast<uint32_t>(esp_timer_get_time() / 1000);

    // ── Decide whether to start a new analog poll cycle ──────────────────
    bool do_cycle = (last_cycle_start == 0) ||
                    ((now_ms - last_cycle_start) >= bms::poller::ANALOG_POLL_PERIOD_MS);

    if (do_cycle) {
      last_cycle_start = now_ms;
      ++cycle_id;

      uint32_t cycle_t0 = now_ms;

      // ── Phase A: poll all packs for analog data ───────────────────────
      BmsSystemSnapshot* sys = bus::snapshot_bus::begin_publish();
      sys->cycle_id           = cycle_id;
      sys->produced_ms        = now_ms;
      sys->pack_count_configured = cfg.bms_count;
      // Init all packs to offline; fill_from_analog will set online=true.
      for (uint8_t i = 0; i < cfg.bms_count; ++i) {
        bms::init_pack_snapshot_offline(sys->pack[i], i);
      }

      for (uint8_t i = 0; i < cfg.bms_count; ++i) {
        local_stats.analog_polls_attempted++;
        local_stats.pack[i].polls++;

        if (!rs485_send_request(cfg.pins.rs485_dir, i, bms::protocol::TB_CID2_ANALOG_VALUES_FIXED_POINT)) {
          local_stats.analog_polls_timeout++;
          local_stats.pack[i].timeouts++;
          continue;
        }

        size_t rxlen = 0;
        if (!rs485_receive_frame(rx_buf, sizeof(rx_buf), rxlen)) {
          ESP_LOGD(TAG, "analog rx timeout pack=%u", i);
          local_stats.analog_polls_timeout++;
          local_stats.pack[i].timeouts++;
          continue;
        }

        uint8_t     bms_id_out, rtn_out;
        const uint8_t* payload = nullptr;
        size_t      payload_len = 0;
        auto herr = bms::protocol::parse_response_header(
            rx_buf, rxlen, bms_id_out, rtn_out, &payload, payload_len);
        if (herr != bms::protocol::ParseError::Ok || rtn_out != bms::protocol::TB_RTN_OK) {
          ESP_LOGW(TAG, "analog hdr err pack=%u err=%d rtn=0x%02X", i, (int)herr, rtn_out);
          local_stats.analog_polls_parse_err++;
          local_stats.pack[i].errors++;
          continue;
        }

        bms::protocol::tb_analog_values_fixed_point parsed{};
        auto perr = bms::protocol::interpret_analog_values_fixed_point(payload, payload_len, parsed);
        if (perr != bms::protocol::ParseError::Ok) {
          ESP_LOGW(TAG, "analog parse err pack=%u err=%d", i, (int)perr);
          local_stats.analog_polls_parse_err++;
          local_stats.pack[i].errors++;
          continue;
        }

        now_ms = static_cast<uint32_t>(esp_timer_get_time() / 1000);
        bms::fill_from_analog(parsed, now_ms, sys->pack[i]);
        local_stats.analog_polls_ok++;
        local_stats.pack[i].ok++;
        ESP_LOGD(TAG, "pack[%u] V=%.3f I=%.2f SOC=%u%%", i,
                 sys->pack[i].pack_voltage, sys->pack[i].pack_current, sys->pack[i].soc);
      }

      // ── Phase B: round-robin alarm or sysparam for one pack ───────────
      if (cfg.bms_count > 0) {
        uint8_t rr = rr_pack % cfg.bms_count;
        if (rr_alarm) {
          // Alarm poll (0x44)
          if (rs485_send_request(cfg.pins.rs485_dir, rr, bms::protocol::TB_CID2_ALARM_INFO)) {
            size_t rxlen = 0;
            if (rs485_receive_frame(rx_buf, sizeof(rx_buf), rxlen)) {
              uint8_t id_out, rtn_out;
              const uint8_t* pl = nullptr; size_t pl_len = 0;
              if (bms::protocol::parse_response_header(rx_buf, rxlen, id_out, rtn_out, &pl, pl_len)
                    == bms::protocol::ParseError::Ok && rtn_out == bms::protocol::TB_RTN_OK) {
                bms::protocol::tb_alarm_info ai{};
                if (bms::protocol::interpret_alarm_info(pl, pl_len, ai) == bms::protocol::ParseError::Ok) {
                  now_ms = static_cast<uint32_t>(esp_timer_get_time() / 1000);
                  bms::fill_from_alarm(ai, now_ms, sys->pack[rr]);
                  local_stats.alarm_polls_ok++;
                } else { local_stats.alarm_polls_err++; }
              } else { local_stats.alarm_polls_err++; }
            } else { local_stats.alarm_polls_err++; }
          } else { local_stats.alarm_polls_err++; }
        } else {
          // Sysparam poll (0x47) — only if not fresh within SYSPARAM_FRESHNESS_MS
          now_ms = static_cast<uint32_t>(esp_timer_get_time() / 1000);
          bool stale = !sys->pack[rr].sysparam_valid ||
                       (now_ms - sys->pack[rr].last_sysparam_ms) >= bms::poller::SYSPARAM_FRESHNESS_MS;
          if (stale && rs485_send_request(cfg.pins.rs485_dir, rr, bms::protocol::TB_CID2_SYSTEM_PARAMETER)) {
            size_t rxlen = 0;
            if (rs485_receive_frame(rx_buf, sizeof(rx_buf), rxlen)) {
              uint8_t id_out, rtn_out;
              const uint8_t* pl = nullptr; size_t pl_len = 0;
              if (bms::protocol::parse_response_header(rx_buf, rxlen, id_out, rtn_out, &pl, pl_len)
                    == bms::protocol::ParseError::Ok && rtn_out == bms::protocol::TB_RTN_OK) {
                bms::protocol::tb_system_parameter sp{};
                if (bms::protocol::interpret_system_parameter(pl, pl_len, sp) == bms::protocol::ParseError::Ok) {
                  now_ms = static_cast<uint32_t>(esp_timer_get_time() / 1000);
                  bms::fill_from_sysparam(sp, now_ms, sys->pack[rr]);
                  local_stats.sysparam_polls_ok++;
                } else { local_stats.sysparam_polls_err++; }
              } else { local_stats.sysparam_polls_err++; }
            } else { local_stats.sysparam_polls_err++; }
          }
        }
        // Advance round-robin state
        if (!rr_alarm) {
          rr_pack = static_cast<uint8_t>((rr_pack + 1) % cfg.bms_count);
        }
        rr_alarm = !rr_alarm;
      }

      // ── Decay offline / update aggregates ────────────────────────────
      now_ms = static_cast<uint32_t>(esp_timer_get_time() / 1000);
      for (uint8_t i = 0; i < cfg.bms_count; ++i) {
        bool was = s_pack_was_online[i];
        if (bms::decay_online_status(sys->pack[i], now_ms, bms::poller::OFFLINE_THRESHOLD_MS)) {
          ESP_LOGW(TAG, "pack[%u] went offline", i);
        }
        bool now_online = sys->pack[i].online;
        if (was && !now_online) {
          diag::alerts::emit(diag::alerts::Severity::Warn, "poller",
                             "pack %u offline", (unsigned)i + 1);
        } else if (!was && now_online) {
          diag::alerts::emit(diag::alerts::Severity::Info, "poller",
                             "pack %u online", (unsigned)i + 1);
        }
        s_pack_was_online[i] = now_online;
      }
      bms::update_system_aggregates(*sys);

      // ── Publish snapshot ─────────────────────────────────────────────
      sys->produced_ms = static_cast<uint32_t>(esp_timer_get_time() / 1000);
      bus::snapshot_bus::publish();

      // ── Phase D: safety aggregation ──────────────────────────────────
      {
        SafetyState tmp;
        uint32_t safety_now = static_cast<uint32_t>(esp_timer_get_time() / 1000);
        safety::runSafety(*sys, cfg, s_safety_prev, safety_now, tmp);
        safety::update_prev_state(tmp, *sys, s_safety_prev);

        // Log and route safety events to MQTT alarm topic.
        uint64_t ts_ms = static_cast<uint64_t>(esp_timer_get_time() / 1000);
        for (uint8_t ev = 0; ev < tmp.event_count; ++ev) {
          const SafetyState::EventEntry& entry = tmp.events[ev];
          ESP_LOGI(TAG, "safety event %u bms=%u bits=0x%llx",
                   static_cast<unsigned>(entry.type),
                   entry.bms_id,
                   static_cast<unsigned long long>(entry.alarm_bits));
          if (cfg.mqtt_enabled) {
            // Non-blocking: drop if queue full (event already logged above).
            route_alarm_event_to_mqtt(entry, ts_ms);
          }
        }
        if (tmp.alarm_flags)
          ESP_LOGW(TAG, "safety: flags=0x%02X ccl=%.0fA dcl=%.0fA msg=%s",
                   tmp.alarm_flags, tmp.ccl_amps, tmp.dcl_amps, tmp.sys_message);

        // Safety flag alert: emit on rising edge of each new flag bit.
        {
          uint8_t new_bits = tmp.alarm_flags & ~s_last_alarm_flags;
          if (new_bits) {
            diag::alerts::emit(diag::alerts::Severity::Warn, "safety",
                               "flag 0x%02X active", (unsigned)new_bits);
          }
          // All-clear edge: flags just went from non-zero to zero.
          if (s_last_alarm_flags != 0 && tmp.alarm_flags == 0) {
            diag::alerts::emit(diag::alerts::Severity::Info, "safety", "all clear");
          }
          s_last_alarm_flags = tmp.alarm_flags;
        }

        portENTER_CRITICAL(&s_safety_mux);
        memcpy(&s_safety, &tmp, sizeof(SafetyState));
        s_safety_valid = true;
        portEXIT_CRITICAL(&s_safety_mux);

        // ── Energy integration (Phase H2) ─────────────────────────────────
        // Compute system power from new safety state and integrate kWh.
        // dt_s: time since last cycle start, clamped to avoid stale bursts.
        {
          static uint32_t s_last_energy_ms = 0;
          uint32_t now_e = static_cast<uint32_t>(esp_timer_get_time() / 1000);
          if (s_last_energy_ms > 0) {
            float dt_s = (float)(now_e - s_last_energy_ms) / 1000.0f;
            float power_w = tmp.pack_current_total * tmp.pack_voltage_avg;
            bms::energy_integrator::integrate(power_w, dt_s,
                                              net::ntp::now_unix_s(),
                                              app::get_config().timezone_offset_h);
          }
          s_last_energy_ms = now_e;
        }
      }

      // (CAN TX runs outside do_cycle — every 50 ms tick below)

      // ── Cycle timing stats ────────────────────────────────────────────
      uint32_t cycle_ms = static_cast<uint32_t>(esp_timer_get_time() / 1000) - cycle_t0;
      if (local_stats.cycles_completed == 0 || cycle_ms > local_stats.cycle_max_ms) {
        local_stats.cycle_max_ms = cycle_ms;
      }
      // Rolling average: simple exponential (α ≈ 0.125) without floats
      if (local_stats.cycle_avg_ms == 0) {
        local_stats.cycle_avg_ms = cycle_ms;
      } else {
        local_stats.cycle_avg_ms = (local_stats.cycle_avg_ms * 7 + cycle_ms) / 8;
      }
      local_stats.cycles_completed++;

      // Flush stats to shared struct under critical section
      portENTER_CRITICAL(&s_stats_mux);
      s_stats = local_stats;
      portEXIT_CRITICAL(&s_stats_mux);

      ESP_LOGD(TAG, "cycle=%lu online=%u/%u cycle_ms=%lu avg=%lu max=%lu",
               (unsigned long)cycle_id,
               sys->pack_count_online, sys->pack_count_configured,
               (unsigned long)cycle_ms,
               (unsigned long)local_stats.cycle_avg_ms,
               (unsigned long)local_stats.cycle_max_ms);
    }

    // ── Phase E: CAN TX (every 50 ms tick) ──────────────────────────────
    // Uses the most recently committed SafetyState. Before the first poll
    // cycle completes s_safety_valid is false and no frames are sent.
    if (s_safety_valid) {
      // Direct read within ControlTask (sole writer on Core 0) — no lock needed.
      can::tx::can_tx_if_due(s_safety, now_ms);
    }
    can::busoff::tick(now_ms);

    // ── Maintain 50 ms base tick using vTaskDelayUntil ───────────────────
    vTaskDelayUntil(&tick_start, pdMS_TO_TICKS(bms::poller::POLL_TICK_MS));
  }
}

// ── Public API ────────────────────────────────────────────────────────────────

namespace bms::poller {

bool start(const Config& cfg) {
  static Config cfg_copy;
  cfg_copy = cfg;  // Take a private copy so the task pointer stays valid

  static TaskHandle_t handle = nullptr;
  BaseType_t r = xTaskCreatePinnedToCore(
      control_task_entry,
      "ctrl",
      /*stack_depth*/ 12288,
      &cfg_copy,
      /*priority*/ 5,
      &handle,
      /*core_id*/ 0
  );
  if (r != pdPASS) {
    ESP_LOGE(TAG, "xTaskCreatePinnedToCore failed (%d)", (int)r);
    return false;
  }
  return true;
}

void get_stats(PollerStats& out) {
  portENTER_CRITICAL(&s_stats_mux);
  out = s_stats;
  portEXIT_CRITICAL(&s_stats_mux);
}

bool read_safety_state(SafetyState& out) {
  portENTER_CRITICAL(&s_safety_mux);
  if (!s_safety_valid) {
    portEXIT_CRITICAL(&s_safety_mux);
    return false;
  }
  memcpy(&out, &s_safety, sizeof(SafetyState));
  portEXIT_CRITICAL(&s_safety_mux);
  return true;
}

}  // namespace bms::poller
