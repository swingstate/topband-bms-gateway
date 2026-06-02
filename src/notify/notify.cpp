#include "notify.h"
#include "telegram.h"
#include "app/boot.h"
#include "net/ntp.h"
#include "net/wifi.h"
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

  for (size_t i = 0; i < k_provider_count; ++i) {
    const notify::INotifyProvider* p = k_providers[i];
    if (!p->is_enabled(a->cfg)) continue;

    char err[96] = {};
    bool ok = p->send(a->msg, a->cfg, err, sizeof(err));
    if (!ok) {
      ESP_LOGW(TAG, "notify::send [%s] failed: %s", p->id(), err);
    }
  }

  // Release TLS slot so the next queued send or test can proceed.
  if (s_tls_sem) xSemaphoreGive(s_tls_sem);
  free(a);
  vTaskDelete(nullptr);
}

namespace notify {

void send(Severity severity, const char* title, const char* body) {
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
struct EventDesc {
  const char*    label;      // short message description
  notify::Severity severity;
};

// Max SafetyEvent index (PacksOnlineRecovered = 17).
static constexpr uint8_t K_EV_MAX = 17;

static const EventDesc k_event_desc[K_EVENT_COUNT] = {
  { nullptr, notify::Severity::Info },          //  0: None        — never fires
  { "Pack went offline",      notify::Severity::Warning  },  //  1: BmsWentOffline
  { "Pack came online",       notify::Severity::Info     },  //  2: BmsCameOnline
  { "Over-voltage started",   notify::Severity::Critical },  //  3: PackOvervoltStart
  { "Over-voltage cleared",   notify::Severity::Info     },  //  4: PackOvervoltClear
  { "Cell over-voltage",      notify::Severity::Critical },  //  5: CellOvervoltStart
  { "Cell over-voltage cleared", notify::Severity::Info  },  //  6: CellOvervoltClear
  { "Under-voltage started",  notify::Severity::Critical },  //  7: PackUndervoltStart
  { "Under-voltage cleared",  notify::Severity::Info     },  //  8: PackUndervoltClear
  { "Charge stopped: temperature", notify::Severity::Warning },  //  9: TempChargeStop
  { "Charge resumed: temperature", notify::Severity::Info   },  // 10: TempChargeResume
  { "Discharge stopped: temperature", notify::Severity::Warning }, // 11: TempDischargeStop
  { "Discharge resumed: temperature", notify::Severity::Info    }, // 12: TempDischargeResume
  { "Cell imbalance detected",notify::Severity::Warning  },  // 13: CellImbalanceStart
  { "Cell imbalance cleared", notify::Severity::Info     },  // 14: CellImbalanceClear
  { "BMS reported alarm",     notify::Severity::Critical },  // 15: BmsReportedAlarm
  { "No packs online",        notify::Severity::Critical },  // 16: NoPacksOnline
  { "Packs online recovered", notify::Severity::Info     },  // 17: PacksOnlineRecovered
};

namespace notify {

void on_safety_event(const SafetyState::EventEntry& entry, uint32_t now_ms) {
  if (!s_tls_sem) return;  // not initialized

  uint8_t ev_id = static_cast<uint8_t>(entry.type);
  if (ev_id == 0 || ev_id > K_EV_MAX) return;  // None or out-of-range

  const Config& cfg = app::get_config();
  if (!cfg.notify_telegram_enabled) return;
  if (cfg.notify_telegram_token[0]   == '\0') return;
  if (cfg.notify_telegram_chat_id[0] == '\0') return;

  // Check event-type enable bitmask.
  if (!(cfg.notify_alert_flags & (1u << ev_id))) return;

  // Enforce safe floors (never below 60 s regardless of config value).
  uint32_t poll_ms = (cfg.notify_poll_interval_s >= 60u)
                     ? (uint32_t)cfg.notify_poll_interval_s * 1000u
                     : 60000u;
  uint32_t cool_ms = (cfg.notify_cooldown_s >= 60u)
                     ? (uint32_t)cfg.notify_cooldown_s * 1000u
                     : 60000u;

  // Rate-limit check under critical section to avoid TOCTOU races.
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

  // Build sender name: config field if non-empty, else device hostname.
  char sender[48];
  if (cfg.notify_sender_name[0] != '\0') {
    snprintf(sender, sizeof(sender), "%s", cfg.notify_sender_name);
  } else {
    net::wifi::get_hostname(sender, sizeof(sender));
  }

  // Build title and body.
  const EventDesc& desc = k_event_desc[ev_id];
  char title[80];
  char body[160];

  if (entry.bms_id != 0xFF) {
    // Per-pack event.
    snprintf(title, sizeof(title), "%s", sender);
    snprintf(body,  sizeof(body),  "Pack %u: %s", (unsigned)(entry.bms_id + 1), desc.label);
  } else {
    // System-wide event.
    snprintf(title, sizeof(title), "%s", sender);
    snprintf(body,  sizeof(body),  "%s", desc.label);
  }

  ESP_LOGI(TAG, "alert notify: event=%u pack=%u: %s", ev_id, entry.bms_id, body);
  send(desc.severity, title, body);
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
  bool ok = a->provider->send(msg, a->cfg, err, sizeof(err));

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
