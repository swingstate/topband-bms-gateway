#include "queues.h"
#include "esp_log.h"

static const char* TAG = "queues";

// ── Global queue handles ──────────────────────────────────────────────────────
QueueHandle_t q_log            = nullptr;
QueueHandle_t q_alert          = nullptr;
QueueHandle_t q_mqtt_publish   = nullptr;
QueueHandle_t q_history_sample = nullptr;
QueueHandle_t q_safety_event   = nullptr;
QueueHandle_t q_ota            = nullptr;

namespace bus {

bool createQueues() {
  // Depths from architecture §5.8.
  q_log            = xQueueCreate(32,  sizeof(LogLine));
  q_alert          = xQueueCreate(16,  sizeof(AlertEntry));
  q_mqtt_publish   = xQueueCreate(32,  sizeof(MqttPublishRequest));
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
