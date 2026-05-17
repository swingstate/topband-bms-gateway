#pragma once
#include <cstdint>
#ifndef NATIVE_BUILD
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#else
// Stub type for host (Catch2) builds — queues are never created in NATIVE_BUILD.
typedef void* QueueHandle_t;
#endif

// ── Payload struct definitions ────────────────────────────────────────────────
// All payloads are fixed-size (no pointers) so they can live in xQueue entries.

// Ring-buffer log line (q_log, depth 32). Written from any task; consumed by
// HousekeepingTask which appends to the in-RAM log ring.
struct LogLine {
  uint32_t ts_ms;
  uint8_t  level;      // 0=debug 1=info 2=warn 3=err
  char     tag[16];
  char     msg[92];    // 112 bytes total
};

// Persistent alert entry (q_alert, depth 16). Exactly the on-disk struct from
// architecture §5.6. Consumed by HousekeepingTask → alerts_store.
struct AlertEntry {
  uint32_t ts_epoch;
  uint32_t uptime_s;
  uint8_t  severity;   // 0=info 1=warn 2=err 3=crit
  uint8_t  category;   // 0=safety 1=net 2=hw 3=ota 4=user
  uint16_t flags;
  char     message[120];
};
static_assert(sizeof(AlertEntry) == 132, "AlertEntry size must be 132 bytes (on-disk format)");

// Pre-serialized MQTT publish request (q_mqtt_publish, depth 32).
// Payload is JSON, serialized at the producer. See architecture §5.8.
struct MqttPublishRequest {
  enum class Topic : uint8_t { Data, Status, Alarm, Diag, Cells, Discovery };
  Topic    topic;
  uint8_t  pack_id;       // 0xFF = non-Cells topics
  bool     retained;
  uint16_t payload_len;
  char     payload[1024]; // pre-serialized JSON
};

// History accumulator sample (q_history_sample, depth 8).
// Placeholder sized for Phase C when BmsSystemSnapshot is defined.
struct HistorySample {
  uint32_t ts_ms;
  float    power_w;
  float    voltage_v;
  float    soc_pct;
  float    temp_c;
};

// Safety state-transition events (q_safety_event, depth 16).
// Matches the enum in SafetyState (architecture §5.4). Standalone here for
// Phase A; Phase D will unify with safety/events.h.
enum class SafetyEvent : uint8_t {
  None = 0,
  BmsWentOffline,
  BmsCameOnline,
  PackOvervoltStart,   PackOvervoltClear,
  CellOvervoltStart,   CellOvervoltClear,
  PackUndervoltStart,  PackUndervoltClear,
  TempChargeStop,      TempChargeResume,
  TempDischargeStop,   TempDischargeResume,
  CellImbalanceStart,  CellImbalanceClear,
  BmsReportedAlarm,
  NoPacksOnline,       PacksOnlineRecovered,
};

// OTA command (q_ota, depth 4). HttpTask posts here; HousekeepingTask acts.
enum class OtaCommand : uint8_t {
  StartDownload = 0,
  Abort         = 1,
  MarkValid     = 2,
};

// ── Queue handles ─────────────────────────────────────────────────────────────
// Extern globals so any module can enqueue without including FreeRTOS internals.
extern QueueHandle_t q_log;
extern QueueHandle_t q_alert;
extern QueueHandle_t q_mqtt_publish;
extern QueueHandle_t q_history_sample;
extern QueueHandle_t q_safety_event;
extern QueueHandle_t q_ota;

namespace bus {
  // Creates all six queues. Returns false on any allocation failure.
  // Must be called once from app::run_boot() after NVS init.
  bool createQueues();
}
