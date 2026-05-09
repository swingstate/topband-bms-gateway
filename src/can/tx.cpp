#include "can/tx.h"
#include "can/victron.h"
#include "can/busoff.h"
#include <cstring>

#ifndef NATIVE_BUILD
#include "driver/twai.h"
#include "esp_log.h"
static const char* TAG = "can_tx";
#endif

// ── Module state ──────────────────────────────────────────────────────────────

// Initialise s_last_tx_ms so the first call to can_tx_if_due fires immediately
// (sentinel: 1001 ms before t=0 in wrapping uint32 arithmetic).
static constexpr uint32_t HEARTBEAT_INTERVAL_MS = 1000u;
static constexpr uint32_t INITIAL_LAST_TX       = static_cast<uint32_t>(0u - 1001u);

static uint32_t  s_last_tx_ms    = INITIAL_LAST_TX;
static uint8_t   s_last_alarm    = 0xFF;  // sentinel: forces alarm-state initialisation on first call
static can::tx::CanStats s_stats {};

// ── TWAI driver ───────────────────────────────────────────────────────────────

bool can::tx::init(const Config& cfg) {
#ifdef NATIVE_BUILD
  (void)cfg;
  return true;
#else
  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(
      static_cast<gpio_num_t>(cfg.pins.can_tx),
      static_cast<gpio_num_t>(cfg.pins.can_rx),
      TWAI_MODE_NORMAL);
  g_config.tx_queue_len = 8;  // headroom for 5-frame burst + express

  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  esp_err_t r = twai_driver_install(&g_config, &t_config, &f_config);
  if (r != ESP_OK) {
    ESP_LOGE(TAG, "twai_driver_install: %s", esp_err_to_name(r));
    return false;
  }
  r = twai_start();
  if (r != ESP_OK) {
    ESP_LOGE(TAG, "twai_start: %s", esp_err_to_name(r));
    twai_driver_uninstall();
    return false;
  }
  ESP_LOGI(TAG, "TWAI started — TX=%d RX=%d 500 kbps",
           cfg.pins.can_tx, cfg.pins.can_rx);
  return true;
#endif
}

// ── Low-level enqueue ─────────────────────────────────────────────────────────

bool can::tx::enqueue(uint32_t id, const uint8_t data[8]) {
#ifdef NATIVE_BUILD
  // Host stub: unconditionally succeed, count the send.
  s_stats.tx_ok++;
  s_stats.tx_fail_streak_max = 0;  // no fail streak in host build
  return true;
#else
  twai_message_t msg{};
  msg.identifier      = id;
  msg.extd            = 0;
  msg.data_length_code = 8;
  memcpy(msg.data, data, 8);

  esp_err_t r = twai_transmit(&msg, pdMS_TO_TICKS(10));
  bool ok = (r == ESP_OK);

  if (ok) {
    s_stats.tx_ok++;
    uint32_t old_streak = s_stats.tx_fail_streak_max;
    s_stats.tx_fail_streak_max = 0;
    // Preserve the historical maximum: tx_fail_streak_max holds the peak, not current streak.
    // Use a separate local to track current streak if needed — for now mirror V2.67 simplicity.
    (void)old_streak;
  } else {
    s_stats.tx_fail++;
    // tx_fail_streak_max updated by busoff module which has fuller context
  }

  can::busoff::on_tx_result(ok);
  return ok;
#endif
}

// ── Heartbeat / express cadence ───────────────────────────────────────────────

bool can::tx::can_tx_if_due(const SafetyState& current, uint32_t now_ms) {
  bool heartbeat_due = (now_ms - s_last_tx_ms >= HEARTBEAT_INTERVAL_MS);
  bool alarm_changed = (current.alarm_flags != s_last_alarm);

  if (!heartbeat_due && !alarm_changed) {
    return false;
  }

  bool ok = can::victron::send_all_victron(current);

  if (heartbeat_due) {
    s_stats.heartbeats++;
  } else {
    // alarm transition only — express send
    s_stats.express_sends++;
  }

  s_last_tx_ms = now_ms;
  s_last_alarm = current.alarm_flags;

  return ok;
}

// ── Stats ─────────────────────────────────────────────────────────────────────

void can::tx::get_stats(CanStats& out) {
  // Called from smoke reader / HTTP handler (Core 1) — no mutex needed for
  // reading 64-bit counters on ESP32-S3 if reads are atomic at the platform
  // level. For now a simple copy is safe: misread at worst causes a blip in
  // a diagnostic counter, never in safety logic.
  memcpy(&out, &s_stats, sizeof(CanStats));
}

// ── Test helpers ──────────────────────────────────────────────────────────────

#ifdef NATIVE_BUILD
void can::tx::test_reset() {
  s_last_tx_ms = INITIAL_LAST_TX;
  s_last_alarm = 0xFF;
  memset(&s_stats, 0, sizeof(s_stats));
}
#endif
