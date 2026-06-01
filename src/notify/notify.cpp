#include "notify.h"
#include "telegram.h"
#include "app/boot.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include <cstring>

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

  free(a);
  vTaskDelete(nullptr);
}

namespace notify {

void send(Severity severity, const char* title, const char* body) {
  auto* a = static_cast<SendTaskArgs*>(malloc(sizeof(SendTaskArgs)));
  if (!a) {
    ESP_LOGE(TAG, "send: OOM allocating task args");
    return;
  }

  a->msg.severity = severity;
  snprintf(a->title_buf, sizeof(a->title_buf), "%s", title ? title : "");
  snprintf(a->body_buf,  sizeof(a->body_buf),  "%s", body  ? body  : "");
  a->msg.title = a->title_buf;
  a->msg.body  = a->body_buf;

  // Get a snapshot of the current live config.
  a->cfg = app::get_config();

  // 12 KB stack: mbedTLS TLS 1.2 handshake alone consumes 4-6 KB of stack
  // (cipher suite negotiation, cert verify, key exchange).  Add https_post()
  // frame (~600 B), task overhead, and FreeRTOS interrupt-save context under
  // MQTT load, and 6 KB reliably overflows → StoreProhibited panic.
  BaseType_t ok = xTaskCreate(send_task, "notify_send", 12288, a, 2, nullptr);
  if (ok != pdPASS) {
    ESP_LOGE(TAG, "send: xTaskCreate failed");
    free(a);
  }
}

// ── notify::test — single-provider test send ─────────────────────────────────

static portMUX_TYPE s_test_mux = portMUX_INITIALIZER_UNLOCKED;
static TestResult   s_test_result = {};

static void set_test_result(TestStatus st, const char* msg) {
  portENTER_CRITICAL(&s_test_mux);
  s_test_result.status = st;
  snprintf(s_test_result.message, sizeof(s_test_result.message), "%s", msg ? msg : "");
  portEXIT_CRITICAL(&s_test_mux);
}

struct TestTaskArgs {
  const INotifyProvider* provider;
  Config cfg;   // merged form+saved values; ready to pass to provider->send()
};

static void test_task(void* arg) {
  auto* a = static_cast<TestTaskArgs*>(arg);

  NotifyMessage msg = {};
  msg.severity = Severity::Info;
  msg.title    = "TopBand BMS Gateway — test";
  msg.body     = "Notification test from Settings. Your gateway can reach this service.";

  char err[128] = {};
  bool ok = a->provider->send(msg, a->cfg, err, sizeof(err));

  if (ok) {
    set_test_result(TestStatus::Ok,
                    "Test notification sent — check your Telegram chat.");
  } else {
    set_test_result(TestStatus::Failed, err[0] ? err : "Test failed (unknown error)");
  }

  free(a);
  vTaskDelete(nullptr);
}

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

  // Build a merged config: start from form values, fall back to saved secrets
  // when the form left sensitive fields blank (leave-blank-to-keep pattern).
  Config merged = form_cfg;
  if (merged.notify_telegram_token[0] == '\0') {
    // Token was not entered in the form — use the saved one.
    memcpy(merged.notify_telegram_token,
           saved_cfg.notify_telegram_token,
           sizeof(merged.notify_telegram_token));
  }

  auto* a = static_cast<TestTaskArgs*>(malloc(sizeof(TestTaskArgs)));
  if (!a) {
    set_test_result(TestStatus::Failed, "OOM");
    return true;
  }
  a->provider = provider;
  a->cfg      = merged;

  set_test_result(TestStatus::Running, "Sending test notification…");

  // 12 KB stack for the same TLS reason as send_task above.
  BaseType_t ok = xTaskCreate(test_task, "notify_test", 12288, a, 2, nullptr);
  if (ok != pdPASS) {
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

}  // namespace notify
