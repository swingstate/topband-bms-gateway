#include "notify.h"
#include "telegram.h"
#include "app/boot.h"
#include "net/ntp.h"
#include "net/wifi.h"
#include "diag/alerts.h"
#include "storage/boot_reasons.h"
#include "sources/ble_scanner.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/portmacro.h"
#include <cstring>
#include <cstdio>

// ── Provider registry ─────────────────────────────────────────────────────────
//
// To add a provider: append its singleton instance here.
// Nothing else in this file changes.

static notify::TelegramProvider s_telegram;

static const notify::INotifyProvider* k_providers[] = {
  &s_telegram,
  // &s_ntfy,   // next provider goes here
};
static constexpr size_t k_provider_count =
    sizeof(k_providers) / sizeof(k_providers[0]);

// ── Internal helpers ──────────────────────────────────────────────────────────

static const char* TAG = "notify";

static const notify::INotifyProvider* find_provider(const char* id) {
  if (!id) return nullptr;
  for (size_t i = 0; i < k_provider_count; ++i) {
    if (strcmp(k_providers[i]->id(), id) == 0) return k_providers[i];
  }
  return nullptr;
}

// ── TLS serialization semaphore ───────────────────────────────────────────────
// At most one TLS handshake runs at a time to bound DRAM pressure.
// send() and test() both acquire this before spawning their tasks.
// Tasks release it before vTaskDelete.
static SemaphoreHandle_t s_tls_sem = nullptr;

// ── Degraded mode ─────────────────────────────────────────────────────────────
// Set by notify::init() when the boot-loop guard detects repeated panic reboots.
// In degraded mode all outbound TLS operations are suppressed; alert logging and
// safety/control are unaffected.  This ensures the notify subsystem can never
// sustain a reboot loop even if a provider-side bug re-emerges.
static bool s_degraded = false;

// ── Verified state ────────────────────────────────────────────────────────────
static portMUX_TYPE  s_status_mux = portMUX_INITIALIZER_UNLOCKED;
static bool          s_verified     = false;
static uint32_t      s_last_ok_ts   = 0;

// ── Rate-limit state (alert sends) ───────────────────────────────────────────
// SafetyState::SafetyEvent values run 0..17; index 0 (None) is never used.
static constexpr uint8_t K_EVENT_COUNT = 18;
static portMUX_TYPE  s_event_mux          = portMUX_INITIALIZER_UNLOCKED;
static uint32_t      s_last_any_send_ms   = 0;          // global poll-interval gate
static uint32_t      s_last_type_ms[K_EVENT_COUNT] = {};// per-type cooldown

// ── notify::init ─────────────────────────────────────────────────────────────

namespace notify {

void init() {
  // If the panic-loop guard detected repeated early crashes, enter degraded mode:
  // disable all outbound TLS operations for this boot.  Safety/control and the
  // alert log are unaffected.  The UI surfaces this state via is_degraded().
  if (storage::boot_reasons::is_crash_loop_suspected()) {
    s_degraded = true;
    ESP_LOGW(TAG, "init: panic-loop guard triggered — notify disabled for this boot");
    diag::alerts::emit(diag::alerts::Severity::Critical, "notify",
                       "Repeated crash-boot detected: notify disabled this boot "
                       "(safety/CAN unaffected). Check Diagnostics for coredump.");
    return;
  }

  s_tls_sem = xSemaphoreCreateBinary();
  if (s_tls_sem) {
    xSemaphoreGive(s_tls_sem);  // initially available
  } else {
    ESP_LOGE(TAG, "init: failed to create TLS semaphore");
  }

  // Load persisted verified state so the UI shows the correct status after reboot.
  const Config& cfg = app::get_config();
  portENTER_CRITICAL(&s_status_mux);
  s_verified   = cfg.notify_telegram_verified;
  s_last_ok_ts = cfg.notify_telegram_last_ok_ts;
  portEXIT_CRITICAL(&s_status_mux);
}

}  // namespace notify

// ── notify::send — fan-out, fire-and-forget ───────────────────────────────────

struct SendTaskArgs {
  notify::NotifyMessage msg;
  // Deep copies of title/body so the originals can be freed by the caller.
  char title_buf[64];
  char body_buf[256];
  // Snapshot of config at dispatch time.
  Config cfg;
};

static void send_task(void* arg) {
  auto* a = static_cast<SendTaskArgs*>(arg);

  // Pause BLE scan for the duration of the TLS handshake so NimBLE mempools
  // and mbedTLS buffers never compete for contiguous internal SRAM at once.
  // No-op when BLE is off (default); always resumed below before vTaskDelete.
  sources::ble_scanner::pause_scan();

  for (size_t i = 0; i < k_provider_count; ++i) {
    const notify::INotifyProvider* p = k_providers[i];
    if (!p->is_enabled(a->cfg)) continue;

    char err[96] = {};
    bool ok = p->send(a->msg, a->cfg, err, sizeof(err));
    if (!ok) {
      ESP_LOGW(TAG, "notify::send [%s] failed: %s", p->id(), err);
    }
  }

  sources::ble_scanner::resume_scan();

  // Release TLS slot so the next queued send or test can proceed.
  if (s_tls_sem) xSemaphoreGive(s_tls_sem);
  free(a);
  vTaskDelete(nullptr);
}

namespace notify {

void send(Severity severity, const char* title, const char* body) {
  // Degraded mode: notify was disabled at init due to crash-loop detection.
  if (s_degraded) return;

  // Serialize: only one TLS handshake at a time.
  if (!s_tls_sem || xSemaphoreTake(s_tls_sem, 0) != pdTRUE) {
    ESP_LOGD(TAG, "send: TLS busy — dropping message");
    return;
  }

  auto* a = static_cast<SendTaskArgs*>(malloc(sizeof(SendTaskArgs)));
  if (!a) {
    ESP_LOGE(TAG, "send: OOM allocating task args");
    xSemaphoreGive(s_tls_sem);
    return;
  }

  a->msg.severity = severity;
  snprintf(a->title_buf, sizeof(a->title_buf), "%s", title ? title : "");
  snprintf(a->body_buf,  sizeof(a->body_buf),  "%s", body  ? body  : "");
  a->msg.title = a->title_buf;
  a->msg.body  = a->body_buf;

  // Get a snapshot of the current live config.
  a->cfg = app::get_config();

  // 16 KB stack: mbedTLS TLS 1.2 handshake alone consumes 4-8 KB of stack
  // (cipher suite negotiation, cert parsing, key exchange) depending on cipher
  // suite and server certificate chain depth.  TelegramProvider::send() adds
  // ~1.6 KB of local buffers (url, text, body arrays).  https_post() adds
  // ~1.3 KB (including the 768-byte response buffer).  Plus FreeRTOS task
  // overhead (~0.5 KB).  12 KB was dangerously tight under MQTT load; 16 KB
  // gives ~4 KB headroom at the deepest TLS call frame.
  // pvPortMallocStackBuf always uses MALLOC_CAP_INTERNAL so this stays in DRAM
  // (stacks cannot be in PSRAM — ESP32-S3 context-switch requires DRAM stacks).
  BaseType_t ok = xTaskCreate(send_task, "notify_send", 16384, a, 2, nullptr);
  if (ok != pdPASS) {
    ESP_LOGE(TAG, "send: xTaskCreate failed");
    xSemaphoreGive(s_tls_sem);
    free(a);
  }
}

}  // namespace notify

// ── on_safety_event — alert-to-notification wiring ───────────────────────────

// Human-readable labels for each SafetyEvent value.
// Index matches SafetyState::SafetyEvent enum; index 0 (None) is nullptr.
// A2: this table is the SINGLE source of truth for event text — both the alert
// log entry and the Telegram message body derive from it via format_event_body().
struct EventDesc {
  const char*    label;      // short message description
  notify::Severity severity;
};

// Max SafetyEvent index (PacksOnlineRecovered = 17).
static constexpr uint8_t K_EV_MAX = 17;

static const EventDesc k_event_desc[K_EVENT_COUNT] = {
  { nullptr, notify::Severity::Info },                              //  0: None
  { "Pack went offline",               notify::Severity::Warning  }, //  1: BmsWentOffline
  { "Pack came online",                notify::Severity::Info     }, //  2: BmsCameOnline
  { "Over-voltage started",            notify::Severity::Critical }, //  3: PackOvervoltStart
  { "Over-voltage cleared",            notify::Severity::Info     }, //  4: PackOvervoltClear
  { "Cell over-voltage started",       notify::Severity::Critical }, //  5: CellOvervoltStart
  { "Cell over-voltage cleared",       notify::Severity::Info     }, //  6: CellOvervoltClear
  { "Under-voltage started",           notify::Severity::Critical }, //  7: PackUndervoltStart
  { "Under-voltage cleared",           notify::Severity::Info     }, //  8: PackUndervoltClear
  { "Charge stopped: temperature",     notify::Severity::Warning  }, //  9: TempChargeStop
  { "Charge resumed: temperature",     notify::Severity::Info     }, // 10: TempChargeResume
  { "Discharge stopped: temperature",  notify::Severity::Warning  }, // 11: TempDischargeStop
  { "Discharge resumed: temperature",  notify::Severity::Info     }, // 12: TempDischargeResume
  { "Cell imbalance detected",         notify::Severity::Warning  }, // 13: CellImbalanceStart
  { "Cell imbalance cleared",          notify::Severity::Info     }, // 14: CellImbalanceClear
  { "BMS reported alarm",              notify::Severity::Critical }, // 15: BmsReportedAlarm
  { "No packs online",                 notify::Severity::Critical }, // 16: NoPacksOnline
  { "Packs online recovered",          notify::Severity::Info     }, // 17: PacksOnlineRecovered
};

// ── A2: single formatting function ───────────────────────────────────────────
// Writes the human-readable body string for a safety event into buf.
// Used by both the alert log emit and the Telegram send, so they always match.
static void format_event_body(const SafetyState::EventEntry& entry, char* buf, size_t sz) {
  uint8_t ev_id = static_cast<uint8_t>(entry.type);
  if (ev_id == 0 || ev_id > K_EV_MAX || !k_event_desc[ev_id].label) {
    snprintf(buf, sz, "Unknown safety event %u", (unsigned)ev_id);
    return;
  }
  const char* label = k_event_desc[ev_id].label;
  if (entry.bms_id != 0xFF) {
    snprintf(buf, sz, "Pack %u: %s", (unsigned)(entry.bms_id + 1), label);
  } else {
    snprintf(buf, sz, "%s", label);
  }
}

// ── A2: alert log + notify fire helper ───────────────────────────────────────
// Emits an alert log entry and sends a Telegram notification for a safety event.
// Called after debounce clears (or immediately when debounce=0).
// Rate-limit is applied here before sending to Telegram.
static void fire_alert_and_notify(const SafetyState::EventEntry& entry, uint32_t now_ms) {
  uint8_t ev_id = static_cast<uint8_t>(entry.type);
  if (ev_id == 0 || ev_id > K_EV_MAX) return;

  const EventDesc& desc = k_event_desc[ev_id];
  if (!desc.label) return;

  // Build body text (shared by alert log and Telegram).
  char body[160];
  format_event_body(entry, body, sizeof(body));

  // Always write to the alert log regardless of Telegram config.
  diag::alerts::Severity alert_sev;
  switch (desc.severity) {
    case notify::Severity::Critical: alert_sev = diag::alerts::Severity::Critical; break;
    case notify::Severity::Warning:  alert_sev = diag::alerts::Severity::Warn;     break;
    default:                         alert_sev = diag::alerts::Severity::Info;     break;
  }
  diag::alerts::emit(alert_sev, "safety", "%s", body);

  // Telegram: only if configured.
  if (!s_tls_sem) return;
  const Config& cfg = app::get_config();
  if (!cfg.notify_telegram_enabled) return;
  if (cfg.notify_telegram_token[0]   == '\0') return;
  if (cfg.notify_telegram_chat_id[0] == '\0') return;
  if (!(cfg.notify_alert_flags & (1u << ev_id))) return;

  // Enforce rate-limit floors (never below 60 s).
  uint32_t poll_ms = (cfg.notify_poll_interval_s >= 60u)
                     ? (uint32_t)cfg.notify_poll_interval_s * 1000u
                     : 60000u;
  uint32_t cool_ms = (cfg.notify_cooldown_s >= 60u)
                     ? (uint32_t)cfg.notify_cooldown_s * 1000u
                     : 60000u;

  bool fire = false;
  portENTER_CRITICAL(&s_event_mux);
  bool global_ok = (s_last_any_send_ms == 0) ||
                   ((now_ms - s_last_any_send_ms) >= poll_ms);
  bool type_ok   = (s_last_type_ms[ev_id] == 0) ||
                   ((now_ms - s_last_type_ms[ev_id]) >= cool_ms);
  if (global_ok && type_ok) {
    fire = true;
    s_last_any_send_ms    = now_ms;
    s_last_type_ms[ev_id] = now_ms;
  }
  portEXIT_CRITICAL(&s_event_mux);

  if (!fire) return;

  char sender[48];
  if (cfg.notify_sender_name[0] != '\0') {
    snprintf(sender, sizeof(sender), "%s", cfg.notify_sender_name);
  } else {
    net::wifi::get_hostname(sender, sizeof(sender));
  }

  ESP_LOGI(TAG, "alert notify: event=%u pack=%u: %s", ev_id, entry.bms_id, body);
  send(desc.severity, sender, body);
}

// ── A1: debounce pending table ────────────────────────────────────────────────
// Tracks begin events waiting for the debounce window to expire.
// If the matching clear arrives before the window expires, both are suppressed.
//
// Sizing: worst case is all 16 packs each contributing CellImbalanceStart +
// BmsReportedAlarm simultaneously, plus a handful of system-level events.
// 32 slots covers 16 packs × 2 event types + 4 system events with headroom.
// Deduplication (same type + bms_id already pending) prevents per-cycle re-fires
// from ever consuming new slots, so overflow is now only possible when genuinely
// distinct (type, pack) pairs accumulate beyond the table capacity.
struct PendingAlert {
  bool     active;
  SafetyState::SafetyEvent begin_type;  // the begin-edge event type
  uint8_t  bms_id;
  SafetyState::EventEntry  entry;       // full entry for deferred emit
  uint32_t start_ms;
};

static constexpr size_t K_PENDING_CAP = 32;
static PendingAlert       s_pending[K_PENDING_CAP];
static portMUX_TYPE       s_pending_mux = portMUX_INITIALIZER_UNLOCKED;

// Overflow drop counter — incremented inside s_pending_mux, read via
// notify::dropped_count() from any task.
static uint32_t s_dropped_count = 0;
// Timestamp of last overflow warning (written from ControlTask only — no lock needed).
static uint32_t s_last_overflow_warn_ms = 0;

// Returns the begin event that a given clear event cancels, or None if not a clear.
static SafetyState::SafetyEvent begin_for_clear(SafetyState::SafetyEvent ev) {
  switch (ev) {
    case SafetyState::SafetyEvent::BmsCameOnline:        return SafetyState::SafetyEvent::BmsWentOffline;
    case SafetyState::SafetyEvent::PackOvervoltClear:    return SafetyState::SafetyEvent::PackOvervoltStart;
    case SafetyState::SafetyEvent::CellOvervoltClear:    return SafetyState::SafetyEvent::CellOvervoltStart;
    case SafetyState::SafetyEvent::PackUndervoltClear:   return SafetyState::SafetyEvent::PackUndervoltStart;
    case SafetyState::SafetyEvent::TempChargeResume:     return SafetyState::SafetyEvent::TempChargeStop;
    case SafetyState::SafetyEvent::TempDischargeResume:  return SafetyState::SafetyEvent::TempDischargeStop;
    case SafetyState::SafetyEvent::CellImbalanceClear:   return SafetyState::SafetyEvent::CellImbalanceStart;
    case SafetyState::SafetyEvent::PacksOnlineRecovered: return SafetyState::SafetyEvent::NoPacksOnline;
    default:                                              return SafetyState::SafetyEvent::None;
  }
}

// Returns true if this event is a "begin" (condition starting).
static bool is_begin_event(SafetyState::SafetyEvent ev) {
  return begin_for_clear(ev) == SafetyState::SafetyEvent::None && ev != SafetyState::SafetyEvent::None;
}

namespace notify {

void on_safety_event(const SafetyState::EventEntry& entry, uint32_t now_ms) {
  uint8_t ev_id = static_cast<uint8_t>(entry.type);
  if (ev_id == 0 || ev_id > K_EV_MAX) return;

  const Config& cfg = app::get_config();
  uint32_t debounce_ms = (uint32_t)cfg.notify_debounce_s * 1000u;

  SafetyState::SafetyEvent pair_begin = begin_for_clear(entry.type);
  bool is_clear = (pair_begin != SafetyState::SafetyEvent::None);
  bool is_begin = is_begin_event(entry.type);

  if (is_clear) {
    // Check if the corresponding begin is pending (not yet fired).
    bool suppressed = false;
    portENTER_CRITICAL(&s_pending_mux);
    for (size_t i = 0; i < K_PENDING_CAP; ++i) {
      if (s_pending[i].active &&
          s_pending[i].begin_type == pair_begin &&
          s_pending[i].bms_id    == entry.bms_id) {
        // Begin is still pending (within debounce window) — suppress both.
        s_pending[i].active = false;
        suppressed = true;
        break;
      }
    }
    portEXIT_CRITICAL(&s_pending_mux);

    if (suppressed) {
      ESP_LOGD(TAG, "debounce: suppressed flap event=%u bms=%u", ev_id, entry.bms_id);
      return;
    }
    // Begin already fired (or debounce=0) — emit clear normally.
    fire_alert_and_notify(entry, now_ms);
    return;
  }

  if (is_begin) {
    if (debounce_ms == 0) {
      // Debounce disabled: emit immediately.
      fire_alert_and_notify(entry, now_ms);
      return;
    }
    // Debounce enabled: add to pending table unless a duplicate is already pending.
    // Deduplication: per-cycle events (CellImbalanceStart, BmsReportedAlarm) fire
    // every cycle while the condition is active.  Without this check those events
    // would fill the table on the very first cycle and overflow on every subsequent
    // cycle — the root cause of the June 2026 production crash loop.
    portENTER_CRITICAL(&s_pending_mux);
    int slot = -1;
    bool dup = false;
    for (int i = 0; i < (int)K_PENDING_CAP; ++i) {
      if (s_pending[i].active &&
          s_pending[i].begin_type == entry.type &&
          s_pending[i].bms_id    == entry.bms_id) {
        dup = true;  // same (type, pack) already pending — skip
        break;
      }
      if (!s_pending[i].active && slot < 0) {
        slot = i;
      }
    }
    bool dropped = false;
    if (!dup) {
      if (slot >= 0) {
        s_pending[slot].active     = true;
        s_pending[slot].begin_type = entry.type;
        s_pending[slot].bms_id     = entry.bms_id;
        s_pending[slot].entry      = entry;
        s_pending[slot].start_ms   = now_ms;
      } else {
        // True overflow after dedup: a new distinct (type, pack) pair has nowhere
        // to go.  Drop with counter — never abort, never crash the device.
        ++s_dropped_count;
        dropped = true;
      }
    }
    portEXIT_CRITICAL(&s_pending_mux);

    // Rate-limited warning outside the critical section.
    if (dropped && (now_ms - s_last_overflow_warn_ms) >= 10000u) {
      s_last_overflow_warn_ms = now_ms;
      portENTER_CRITICAL(&s_pending_mux);
      uint32_t cnt = s_dropped_count;
      portEXIT_CRITICAL(&s_pending_mux);
      ESP_LOGW(TAG, "debounce overflow: ev=%u dropped (total=%lu)",
               ev_id, (unsigned long)cnt);
    }
    return;
  }

  // Neither a begin nor a clear — should not occur with current SafetyEvent enum;
  // kept as a defensive fallback.
  if (debounce_ms == 0) {
    fire_alert_and_notify(entry, now_ms);
  } else {
    portENTER_CRITICAL(&s_pending_mux);
    int slot = -1;
    bool dup = false;
    for (int i = 0; i < (int)K_PENDING_CAP; ++i) {
      if (s_pending[i].active &&
          s_pending[i].begin_type == entry.type &&
          s_pending[i].bms_id    == entry.bms_id) {
        dup = true;
        break;
      }
      if (!s_pending[i].active && slot < 0) slot = i;
    }
    bool dropped = false;
    if (!dup) {
      if (slot >= 0) {
        s_pending[slot].active     = true;
        s_pending[slot].begin_type = entry.type;
        s_pending[slot].bms_id     = entry.bms_id;
        s_pending[slot].entry      = entry;
        s_pending[slot].start_ms   = now_ms;
      } else {
        ++s_dropped_count;
        dropped = true;
      }
    }
    portEXIT_CRITICAL(&s_pending_mux);
    if (dropped && (now_ms - s_last_overflow_warn_ms) >= 10000u) {
      s_last_overflow_warn_ms = now_ms;
      portENTER_CRITICAL(&s_pending_mux);
      uint32_t cnt = s_dropped_count;
      portEXIT_CRITICAL(&s_pending_mux);
      ESP_LOGW(TAG, "debounce overflow: ev=%u dropped (total=%lu)",
               ev_id, (unsigned long)cnt);
    }
  }
}

void flush_pending_alerts(uint32_t now_ms) {
  const Config& cfg = app::get_config();
  uint32_t debounce_ms = (uint32_t)cfg.notify_debounce_s * 1000u;
  if (debounce_ms == 0) return;  // nothing pending when debounce is off

  for (size_t i = 0; i < K_PENDING_CAP; ++i) {
    SafetyState::EventEntry entry_copy{};
    bool should_fire = false;

    portENTER_CRITICAL(&s_pending_mux);
    if (s_pending[i].active &&
        (now_ms - s_pending[i].start_ms) >= debounce_ms) {
      entry_copy   = s_pending[i].entry;
      should_fire  = true;
      s_pending[i].active = false;  // remove from table after firing
    }
    portEXIT_CRITICAL(&s_pending_mux);

    if (should_fire) {
      fire_alert_and_notify(entry_copy, now_ms);
    }
  }
}

}  // namespace notify

// ── notify::test — single-provider test send ─────────────────────────────────

static portMUX_TYPE s_test_mux = portMUX_INITIALIZER_UNLOCKED;
static notify::TestResult s_test_result = {};

static void set_test_result(notify::TestStatus st, const char* msg) {
  portENTER_CRITICAL(&s_test_mux);
  s_test_result.status = st;
  snprintf(s_test_result.message, sizeof(s_test_result.message), "%s", msg ? msg : "");
  portEXIT_CRITICAL(&s_test_mux);
}

struct TestTaskArgs {
  const notify::INotifyProvider* provider;
  Config cfg;   // merged form+saved values; ready to pass to provider->send()
};

static void test_task(void* arg) {
  auto* a = static_cast<TestTaskArgs*>(arg);

  notify::NotifyMessage msg = {};
  msg.severity = notify::Severity::Info;
  msg.title    = "TopBand BMS Gateway test";
  msg.body     = "Notification test from Settings. Your gateway can reach this service.";

  char err[128] = {};
  sources::ble_scanner::pause_scan();
  bool ok = a->provider->send(msg, a->cfg, err, sizeof(err));
  sources::ble_scanner::resume_scan();

  if (ok) {
    set_test_result(notify::TestStatus::Ok,
                    "Test notification sent — check your Telegram chat.");
    notify::mark_telegram_verified();
  } else {
    set_test_result(notify::TestStatus::Failed, err[0] ? err : "Test failed (unknown error)");
  }

  // Release TLS slot before exiting.
  if (s_tls_sem) xSemaphoreGive(s_tls_sem);
  free(a);
  vTaskDelete(nullptr);
}

namespace notify {

bool test(const char* provider_id,
          const Config& form_cfg,
          const Config& saved_cfg) {
  portENTER_CRITICAL(&s_test_mux);
  bool busy = (s_test_result.status == TestStatus::Running);
  portEXIT_CRITICAL(&s_test_mux);
  if (busy) return false;

  const INotifyProvider* provider = find_provider(provider_id);
  if (!provider) {
    set_test_result(TestStatus::Failed, "Unknown provider");
    return true;
  }

  // Serialize: acquire the TLS semaphore before spawning the test task.
  if (!s_tls_sem || xSemaphoreTake(s_tls_sem, 0) != pdTRUE) {
    set_test_result(TestStatus::Failed,
                    "Gateway is currently sending an alert — wait a moment and try again");
    return true;
  }

  // Build a merged config: start from form values, fall back to saved secrets
  // when the form left sensitive fields blank (leave-blank-to-keep pattern).
  Config merged = form_cfg;
  if (merged.notify_telegram_token[0] == '\0') {
    memcpy(merged.notify_telegram_token,
           saved_cfg.notify_telegram_token,
           sizeof(merged.notify_telegram_token));
  }
  if (merged.notify_telegram_chat_id[0] == '\0') {
    memcpy(merged.notify_telegram_chat_id,
           saved_cfg.notify_telegram_chat_id,
           sizeof(merged.notify_telegram_chat_id));
  }

  auto* a = static_cast<TestTaskArgs*>(malloc(sizeof(TestTaskArgs)));
  if (!a) {
    xSemaphoreGive(s_tls_sem);
    set_test_result(TestStatus::Failed, "OOM");
    return true;
  }
  a->provider = provider;
  a->cfg      = merged;

  set_test_result(TestStatus::Running, "Sending test notification\xe2\x80\xa6");

  // 16 KB stack — same rationale as send_task above.
  BaseType_t ok = xTaskCreate(test_task, "notify_test", 16384, a, 2, nullptr);
  if (ok != pdPASS) {
    xSemaphoreGive(s_tls_sem);
    set_test_result(TestStatus::Failed, "Task create failed");
    free(a);
  }
  return true;
}

TestResult test_result() {
  portENTER_CRITICAL(&s_test_mux);
  TestResult snap = s_test_result;
  portEXIT_CRITICAL(&s_test_mux);
  return snap;
}

// ── Verified state ────────────────────────────────────────────────────────────

void mark_telegram_verified() {
  uint32_t ts = net::ntp::now_unix_s();
  portENTER_CRITICAL(&s_status_mux);
  s_verified   = true;
  s_last_ok_ts = ts;
  portEXIT_CRITICAL(&s_status_mux);

  // Persist to NVS (one write per explicit test success — not on every alert send).
  Config cfg = app::get_config();
  cfg.notify_telegram_verified   = true;
  cfg.notify_telegram_last_ok_ts = ts;
  if (!app::update_and_save_config(cfg)) {
    ESP_LOGW(TAG, "mark_telegram_verified: NVS persist failed");
  }
}

uint32_t dropped_count() {
  portENTER_CRITICAL(&s_pending_mux);
  uint32_t c = s_dropped_count;
  portEXIT_CRITICAL(&s_pending_mux);
  return c;
}

bool is_degraded() {
  return s_degraded;
}

bool is_tls_busy() {
  if (!s_tls_sem) return false;
  // Non-destructive check: try to take with zero timeout.
  // If we get it → it was free → give it back immediately → not busy.
  // If we can't get it → something else holds it → busy.
  if (xSemaphoreTake(s_tls_sem, 0) == pdTRUE) {
    xSemaphoreGive(s_tls_sem);
    return false;
  }
  return true;
}

void reset_telegram_verified() {
  portENTER_CRITICAL(&s_status_mux);
  bool was_verified = s_verified;
  s_verified   = false;
  s_last_ok_ts = 0;
  portEXIT_CRITICAL(&s_status_mux);

  if (was_verified) {
    Config cfg = app::get_config();
    cfg.notify_telegram_verified   = false;
    cfg.notify_telegram_last_ok_ts = 0;
    app::update_and_save_config(cfg);
  }
}

TelegramStatus telegram_status() {
  const Config& cfg = app::get_config();
  TelegramStatus s{};
  s.token_stored   = cfg.notify_telegram_token[0]   != '\0';
  s.chat_id_stored = cfg.notify_telegram_chat_id[0] != '\0';
  portENTER_CRITICAL(&s_status_mux);
  s.verified   = s_verified;
  s.last_ok_ts = s_last_ok_ts;
  portEXIT_CRITICAL(&s_status_mux);
  return s;
}

}  // namespace notify

// ── Dev TLS burst trigger ─────────────────────────────────────────────────────
// Compiled only when BLE_SPIKE_DEV_BURST=1 (set in platformio.ini for spike builds).
//
// Purpose: reproduce the V3.0 DRAM-fragmentation failure mode — not total heap
// exhaustion, but the largest *contiguous* block dropping below what a 16 KB TLS
// task stack requires.  Fires N sequential notify::send() calls via the REAL TLS
// path so the DRAM watermark in the Diag panel captures the true worst-case dip.
//
// Safety properties:
// - Only started via explicit POST /api/diag/tls-burst (auth-protected, dev endpoint).
// - Runs in its own low-priority task; cannot preempt or block the ControlTask.
// - Does not bypass the TLS serialization semaphore; each send() waits for the
//   previous one to complete, exactly as production traffic would.
// - send() drops silently when TLS is degraded (crash-loop guard), so burst is
//   automatically inert on a degraded boot.
// - NEVER touches the safety/CAN path. All activity is in the notify layer.
#if BLE_SPIKE_DEV_BURST
#include "esp_heap_caps.h"

static volatile bool s_burst_active = false;
static volatile int  s_burst_fired  = 0;

struct BurstTaskArgs { int n; };

static void burst_task(void* arg) {
  auto* a = static_cast<BurstTaskArgs*>(arg);
  int n = a->n;
  free(a);

  // 4-point per-handshake diagnosis:
  //   [1-before]     : largest free contiguous DRAM block immediately before handshake starts
  //   [2-peak]       : minimum seen while TLS is in progress (mbedTLS stack + heap at worst)
  //   [3-after_free] : block immediately after send_task releases TLS semaphore and exits
  //   [4-settled]    : block after 500 ms settle (gives allocator time to coalesce freed memory)
  //
  // Verdict key:
  //   delta_vs_before near 0 across all 4 handshakes → transient fragmentation only
  //   delta_vs_before monotonically negative          → memory leak (real bug)
  ESP_LOGI(TAG, "[dev-burst] start n=%d -- 4-point diagnosis: before|peak|after_free|settled", n);

  uint32_t baseline = heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL);
  ESP_LOGI(TAG, "[dev-burst] DRAM baseline (idle, before HS 1): %lu B", (unsigned long)baseline);

  for (int i = 0; i < n; i++) {
    // Wait for TLS slot to be free from the previous handshake (max 30 s, 50 ms ticks).
    for (int w = 0; w < 600 && notify::is_tls_busy(); w++) {
      vTaskDelay(pdMS_TO_TICKS(50));
    }

    // [1] Before: sample immediately before firing the handshake.
    uint32_t blk_before = heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL);
    ESP_LOGI(TAG, "[dev-burst] HS %d/%d [1-before]      %lu B", i + 1, n, (unsigned long)blk_before);

    notify::send(notify::Severity::Warning, "BLE-burst-dev",
                 "[dev-burst-test] TLS coexistence diagnosis -- disregard");
    s_burst_fired = s_burst_fired + 1;

    // Yield 50 ms so send_task takes the TLS semaphore before we start polling.
    vTaskDelay(pdMS_TO_TICKS(50));

    // [2] Peak (during): poll while TLS handshake is in progress; record minimum.
    uint32_t blk_peak = UINT32_MAX;
    for (int w = 0; w < 600 && notify::is_tls_busy(); w++) {
      uint32_t blk = heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL);
      if (blk < blk_peak) blk_peak = blk;
      vTaskDelay(pdMS_TO_TICKS(50));
    }
    // Fast-path: handshake already done before first poll tick.
    if (blk_peak == UINT32_MAX) blk_peak = heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL);
    ESP_LOGI(TAG, "[dev-burst] HS %d/%d [2-peak]        %lu B", i + 1, n, (unsigned long)blk_peak);

    // [3] After free: sample immediately after TLS context is destroyed and semaphore released.
    uint32_t blk_after_free = heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL);
    ESP_LOGI(TAG, "[dev-burst] HS %d/%d [3-after_free]  %lu B", i + 1, n, (unsigned long)blk_after_free);

    // [4] Settled: give the allocator 500 ms to coalesce freed blocks.
    vTaskDelay(pdMS_TO_TICKS(500));
    uint32_t blk_settled = heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL);
    int32_t  delta        = (int32_t)blk_settled - (int32_t)blk_before;
    ESP_LOGI(TAG, "[dev-burst] HS %d/%d [4-settled]     %lu B  (delta_vs_before=%+ld)",
             i + 1, n, (unsigned long)blk_settled, (long)delta);
  }

  uint32_t blk_final   = heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL);
  int32_t  delta_total = (int32_t)blk_final - (int32_t)baseline;
  ESP_LOGI(TAG, "[dev-burst] VERDICT: n=%d fired=%d  dram_final=%lu B  delta_vs_baseline=%+ld B",
           n, (int)s_burst_fired, (unsigned long)blk_final, (long)delta_total);
  ESP_LOGI(TAG, "[dev-burst] VERDICT: delta near 0 -> transient fragmentation; negative -> LEAK");

  s_burst_active = false;
  vTaskDelete(nullptr);
}

namespace notify {

bool dev_tls_burst_start(int n) {
  if (n <= 0 || n > 10) return false;
  if (s_burst_active) return false;
  s_burst_active = true;

  auto* a = static_cast<BurstTaskArgs*>(malloc(sizeof(BurstTaskArgs)));
  if (!a) { s_burst_active = false; return false; }
  a->n = n;

  // Priority 1: below HTTP handlers (4) and notify_send tasks (2) so the burst
  // never starves the normal notification path or the web server.
  BaseType_t ok = xTaskCreate(burst_task, "tls_burst_dev", 4096, a, 1, nullptr);
  if (ok != pdPASS) {
    ESP_LOGE(TAG, "[dev-burst] xTaskCreate failed");
    s_burst_active = false;
    free(a);
    return false;
  }
  return true;
}

bool dev_tls_burst_active() { return s_burst_active; }
int  dev_tls_burst_fired()  { return s_burst_fired;  }

}  // namespace notify

#endif  // BLE_SPIKE_DEV_BURST
