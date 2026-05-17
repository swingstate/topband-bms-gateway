#include "can/busoff.h"
#include "can/tx.h"
#include "diag/alerts.h"
#include <cstring>

#ifndef NATIVE_BUILD
#include "driver/twai.h"
#include "esp_log.h"
static const char* TAG = "busoff";
#endif

// ── State ─────────────────────────────────────────────────────────────────────

namespace {

enum class State : uint8_t { HEALTHY, BUS_OFF_DETECTED, RECOVERING };

static State    s_state          = State::HEALTHY;
static uint32_t s_retry_at_ms   = 0;   // next recovery attempt timestamp
static uint32_t s_backoff_ms    = 1000; // current backoff window (doubles each failure)
static uint32_t s_consec_fails  = 0;   // consecutive TX failures since last success

// Mock TWAI state for host builds
#ifdef NATIVE_BUILD
static int s_mock_state = 0;  // 0=RUNNING, 1=BUS_OFF, 2=RECOVERING, 3=STOPPED
#endif

}  // namespace

// ── Mock / IDF helpers ────────────────────────────────────────────────────────

#ifdef NATIVE_BUILD
void can::busoff::set_mock_twai_state(int state) { s_mock_state = state; }
void can::busoff::test_reset() {
  s_state         = State::HEALTHY;
  s_retry_at_ms   = 0;
  s_backoff_ms    = 1000;
  s_consec_fails  = 0;
  s_mock_state    = 0;
}
#endif

// ── on_tx_result ──────────────────────────────────────────────────────────────

void can::busoff::on_tx_result(bool tx_succeeded) {
  if (tx_succeeded) {
    s_consec_fails = 0;
  } else {
    s_consec_fails++;
    can::tx::CanStats st{};
    can::tx::get_stats(st);
    // Mirror V2.67 thresholds: log at 50 and 500 consecutive failures.
    // (Logging suppressed in host build — no esp_log available.)
#ifndef NATIVE_BUILD
    if (s_consec_fails == 50) {
      ESP_LOGW(TAG, "CAN: 50 consecutive TX failures");
    } else if (s_consec_fails == 500) {
      ESP_LOGE(TAG, "CAN: 500 consecutive TX failures — check bus");
    }
#endif
  }
}

// ── tick ──────────────────────────────────────────────────────────────────────

void can::busoff::tick(uint32_t now_ms) {
#ifdef NATIVE_BUILD
  // Host: drive state machine from injected mock state.
  bool is_bus_off   = (s_mock_state == 1);
  bool is_recovering = (s_mock_state == 2);

  if (s_state == State::HEALTHY) {
    if (is_bus_off) {
      s_state      = State::BUS_OFF_DETECTED;
      s_retry_at_ms = now_ms + s_backoff_ms;
    }
  } else if (s_state == State::BUS_OFF_DETECTED) {
    if (!is_bus_off && !is_recovering) {
      s_state    = State::HEALTHY;
      s_backoff_ms = 1000;
    } else if (now_ms >= s_retry_at_ms) {
      s_state    = State::RECOVERING;
      s_retry_at_ms = now_ms + s_backoff_ms;
      s_backoff_ms  = (s_backoff_ms * 2 > 30000) ? 30000 : s_backoff_ms * 2;
    }
  } else if (s_state == State::RECOVERING) {
    if (!is_bus_off && !is_recovering) {
      s_state    = State::HEALTHY;
      s_backoff_ms = 1000;
    } else if (now_ms >= s_retry_at_ms) {
      // Still unhealthy after backoff window; loop back to detected
      s_state    = State::BUS_OFF_DETECTED;
      s_retry_at_ms = now_ms + s_backoff_ms;
      s_backoff_ms  = (s_backoff_ms * 2 > 30000) ? 30000 : s_backoff_ms * 2;
    }
  }

#else  // IDF build

  twai_status_info_t info{};
  esp_err_t r = twai_get_status_info(&info);
  if (r != ESP_OK) return;  // Driver not started yet or uninstalled

  bool is_bus_off    = (info.state == TWAI_STATE_BUS_OFF);
  bool is_running    = (info.state == TWAI_STATE_RUNNING);
  bool is_recovering_hw = (info.state == TWAI_STATE_RECOVERING);

  if (s_state == State::HEALTHY) {
    if (is_bus_off) {
      ESP_LOGW(TAG, "BUS-OFF detected — initiating recovery (backoff=%lu ms)",
               (unsigned long)s_backoff_ms);
      s_state       = State::BUS_OFF_DETECTED;
      s_retry_at_ms = now_ms + s_backoff_ms;
      twai_initiate_recovery();  // begins 128+11 recessive bit sequence
      diag::alerts::emit(diag::alerts::Severity::Error, "can", "bus-off entered");
    }
  } else if (s_state == State::BUS_OFF_DETECTED) {
    if (is_running) {
      // Driver recovered on its own (shouldn't happen without twai_start, but guard)
      ESP_LOGI(TAG, "BUS-OFF cleared — HEALTHY");
      s_state      = State::HEALTHY;
      s_backoff_ms = 1000;
    } else if (is_recovering_hw && now_ms >= s_retry_at_ms) {
      // Recovery complete — restart driver
      ESP_LOGI(TAG, "Recovery bit sequence done — calling twai_start");
      s_state       = State::RECOVERING;
      s_retry_at_ms = now_ms + s_backoff_ms;
      s_backoff_ms  = (s_backoff_ms * 2 > 30000u) ? 30000u : s_backoff_ms * 2;
      if (twai_start() != ESP_OK) {
        ESP_LOGE(TAG, "twai_start failed after recovery");
      }
    } else if (is_bus_off && now_ms >= s_retry_at_ms) {
      // Still bus-off; retry recovery
      ESP_LOGW(TAG, "Still BUS-OFF — retry twai_initiate_recovery (backoff=%lu ms)",
               (unsigned long)s_backoff_ms);
      s_retry_at_ms = now_ms + s_backoff_ms;
      s_backoff_ms  = (s_backoff_ms * 2 > 30000u) ? 30000u : s_backoff_ms * 2;
      twai_initiate_recovery();
    }
  } else if (s_state == State::RECOVERING) {
    if (is_running) {
      ESP_LOGI(TAG, "CAN driver resumed — HEALTHY (backoff reset)");
      s_state      = State::HEALTHY;
      s_backoff_ms = 1000;
      s_consec_fails = 0;
      diag::alerts::emit(diag::alerts::Severity::Info, "can", "bus-off recovered");
    } else if (is_bus_off && now_ms >= s_retry_at_ms) {
      // Went bus-off again during recovery window
      ESP_LOGW(TAG, "BUS-OFF re-triggered during RECOVERING — retry (backoff=%lu ms)",
               (unsigned long)s_backoff_ms);
      s_state       = State::BUS_OFF_DETECTED;
      s_retry_at_ms = now_ms + s_backoff_ms;
      s_backoff_ms  = (s_backoff_ms * 2 > 30000u) ? 30000u : s_backoff_ms * 2;
      twai_initiate_recovery();
    }
  }
#endif  // NATIVE_BUILD
}

// ── is_healthy ────────────────────────────────────────────────────────────────

bool can::busoff::is_healthy() {
  return s_state == State::HEALTHY;
}
