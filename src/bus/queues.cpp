#include "queues.h"
#include "esp_log.h"
#include "esp_heap_caps.h"

static const char* TAG = "queues";

// StaticQueue_t metadata must be in internal RAM (FreeRTOS requirement).
// The actual storage buffers are allocated from PSRAM to save internal heap.
static StaticQueue_t s_mqtt_pub_sq;
static StaticQueue_t s_log_sq;
static StaticQueue_t s_alert_sq;

// ── Global queue handles ──────────────────────────────────────────────────────
QueueHandle_t q_log            = nullptr;
QueueHandle_t q_alert          = nullptr;
QueueHandle_t q_mqtt_publish   = nullptr;
QueueHandle_t q_history_sample = nullptr;
QueueHandle_t q_safety_event   = nullptr;
QueueHandle_t q_ota            = nullptr;

// Helper: allocate queue storage from PSRAM with internal-RAM fallback.
static uint8_t* alloc_queue_storage(size_t bytes) {
  uint8_t* p = static_cast<uint8_t*>(
      heap_caps_malloc(bytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
  if (!p) p = static_cast<uint8_t*>(malloc(bytes));
  return p;
}

namespace bus {

bool createQueues() {
  // q_mqtt_publish: 32 × ~1032 bytes = ~33 KB — PSRAM to protect internal heap
  // from exhaustion when HA discovery floods 24 entity payloads on connect.
  uint8_t* mqtt_stor = alloc_queue_storage(32u * sizeof(MqttPublishRequest));
  q_mqtt_publish = mqtt_stor
      ? xQueueCreateStatic(32, sizeof(MqttPublishRequest), mqtt_stor, &s_mqtt_pub_sq)
      : xQueueCreate(32, sizeof(MqttPublishRequest));

  // q_log: 32 × 112 bytes = ~3.5 KB — PSRAM.
  uint8_t* log_stor = alloc_queue_storage(32u * sizeof(LogLine));
  q_log = log_stor
      ? xQueueCreateStatic(32, sizeof(LogLine), log_stor, &s_log_sq)
      : xQueueCreate(32, sizeof(LogLine));

  // q_alert: 16 × 132 bytes = ~2.1 KB — PSRAM.
  uint8_t* alert_stor = alloc_queue_storage(16u * sizeof(AlertEntry));
  q_alert = alert_stor
      ? xQueueCreateStatic(16, sizeof(AlertEntry), alert_stor, &s_alert_sq)
      : xQueueCreate(16, sizeof(AlertEntry));

  q_history_sample = xQueueCreate(8,   sizeof(HistorySample));
  q_safety_event   = xQueueCreate(16,  sizeof(SafetyEvent));
  q_ota            = xQueueCreate(4,   sizeof(OtaCommand));

  const bool ok = q_log && q_alert && q_mqtt_publish &&
                  q_history_sample && q_safety_event && q_ota;

  if (!ok) {
    ESP_LOGE(TAG, "Queue allocation failed — not enough heap");
  }
  return ok;
}

}  // namespace bus
