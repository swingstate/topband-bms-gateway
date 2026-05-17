#include "handlers_diag.h"
#include "bms/poller.h"
#include "can/tx.h"
#include "bus/snapshot_bus.h"
#include "mqtt/publisher.h"
#include "net/ntp.h"
#include "storage/lfs_store.h"
#include "storage/energy_store.h"
#include "storage/history_store.h"
#include "storage/boot_reasons.h"
#include "storage/alerts_store.h"
#include "diag/log_ring.h"
#include "app/version.h"
#include "app/boot.h"
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "esp_system.h"
#include "esp_timer.h"
#include <cstring>
#include <cstdio>

static const char* TAG = "web_diag";

// ── HStream (shared pattern from handlers_history.cpp) ───────────────────────

struct HStream {
  httpd_req_t* req;
  char         buf[2048];
  size_t       pos;
  esp_err_t    err;
};

static void hs_flush(HStream& s) {
  if (s.pos > 0 && s.err == ESP_OK)
    s.err = httpd_resp_send_chunk(s.req, s.buf, (ssize_t)s.pos);
  s.pos = 0;
}
static void hs_raw(HStream& s, const char* data, size_t n) {
  while (n > 0 && s.err == ESP_OK) {
    size_t space = sizeof(s.buf) - s.pos;
    size_t copy  = n < space ? n : space;
    memcpy(s.buf + s.pos, data, copy);
    s.pos += copy; n -= copy; data += copy;
    if (s.pos == sizeof(s.buf)) hs_flush(s);
  }
}
static void hs_str(HStream& s, const char* str) { hs_raw(s, str, strlen(str)); }
static void hs_uint(HStream& s, uint64_t v) {
  char t[24]; snprintf(t, sizeof(t), "%llu", (unsigned long long)v); hs_str(s, t);
}
static void hs_f2(HStream& s, float v) {
  char t[16]; snprintf(t, sizeof(t), "%.2f", v); hs_str(s, t);
}
static void hs_bool(HStream& s, bool v) { hs_str(s, v ? "true" : "false"); }
static void hs_json_str(HStream& s, const char* v) {
  hs_str(s, "\"");
  // Escape control characters and quotes.
  for (const char* p = v; *p; p++) {
    if (*p == '"')       hs_str(s, "\\\"");
    else if (*p == '\\') hs_str(s, "\\\\");
    else if (*p == '\n') hs_str(s, "\\n");
    else if (*p == '\r') hs_str(s, "\\r");
    else if (*p == '\t') hs_str(s, "\\t");
    else                 hs_raw(s, p, 1);
  }
  hs_str(s, "\"");
}

// ── Log ring section ──────────────────────────────────────────────────────────
// log_ring::snapshot returns newline-delimited text; we JSON-encode each line.

static constexpr size_t LOG_SNAPSHOT_SIZE = 200 * 121 + 1;  // 200 lines × 121 B

static void hs_log_ring(HStream& s) {
  // Heap-allocate the snapshot buffer (24 KB is too large for the stack).
  char* snap = static_cast<char*>(malloc(LOG_SNAPSHOT_SIZE));
  if (!snap) {
    hs_str(s, "\"log_ring\":[]");
    return;
  }
  size_t written = diag::log_ring::snapshot(snap, LOG_SNAPSHOT_SIZE);

  hs_str(s, "\"log_ring\":[");
  bool first = true;
  char* line = snap;
  char* end  = snap + written;
  while (line < end) {
    char* nl = static_cast<char*>(memchr(line, '\n', (size_t)(end - line)));
    size_t len = nl ? (size_t)(nl - line) : (size_t)(end - line);
    if (len > 0) {
      if (!first) hs_str(s, ",");
      first = false;
      // Temporarily null-terminate for hs_json_str.
      char saved = line[len];
      line[len] = '\0';
      hs_json_str(s, line);
      line[len] = saved;
    }
    line += len + 1;  // skip past '\n'
  }
  free(snap);
  hs_str(s, "]");
}

// ── Handler ───────────────────────────────────────────────────────────────────

namespace web {

esp_err_t handle_diag(httpd_req_t* req) {
  bms::poller::PollerStats ps{};
  bms::poller::get_stats(ps);

  can::tx::CanStats cs{};
  can::tx::get_stats(cs);

  httpd_resp_set_type(req, "application/json");
  httpd_resp_set_hdr(req, "Cache-Control", "no-cache");

  HStream s = { req, {}, 0, ESP_OK };

  uint32_t uptime_s = (uint32_t)(esp_timer_get_time() / 1000000LL);
  uint32_t heap_free = esp_get_free_heap_size();
  uint32_t heap_min  = esp_get_minimum_free_heap_size();
  uint32_t psram_free = heap_caps_get_free_size(MALLOC_CAP_SPIRAM);

  hs_str(s, "{");

  // ── system ────────────────────────────────────────────────────────────────
  hs_str(s, "\"system\":{");
  hs_str(s, "\"fw\":"); hs_json_str(s, FW_VERSION);
  hs_str(s, ",\"uptime_s\":"); hs_uint(s, uptime_s);
  hs_str(s, ",\"free_heap\":"); hs_uint(s, heap_free);
  hs_str(s, ",\"min_heap\":"); hs_uint(s, heap_min);
  hs_str(s, ",\"free_psram\":"); hs_uint(s, psram_free);

  const char* reset_reason = "UNKNOWN";
  switch (esp_reset_reason()) {
    case ESP_RST_POWERON:   reset_reason = "POWERON_RESET"; break;
    case ESP_RST_SW:        reset_reason = "SOFTWARE_RESET"; break;
    case ESP_RST_PANIC:     reset_reason = "PANIC"; break;
    case ESP_RST_INT_WDT:   reset_reason = "INT_WDT"; break;
    case ESP_RST_TASK_WDT:  reset_reason = "TASK_WDT"; break;
    case ESP_RST_WDT:       reset_reason = "WDT"; break;
    case ESP_RST_DEEPSLEEP: reset_reason = "DEEPSLEEP"; break;
    case ESP_RST_BROWNOUT:  reset_reason = "BROWNOUT"; break;
    default: break;
  }
  hs_str(s, ",\"reset_reason\":"); hs_json_str(s, reset_reason);
  hs_str(s, ",\"build\":"); hs_json_str(s, BUILD_DATE " " BUILD_TIME);
  hs_str(s, ",\"version\":"); hs_json_str(s, FW_VERSION);
  hs_str(s, "}");

  // ── poller ────────────────────────────────────────────────────────────────
  hs_str(s, ",\"poller\":{");
  hs_str(s, "\"cycles_completed\":"); hs_uint(s, ps.cycles_completed);
  hs_str(s, ",\"cycle_avg_ms\":"); hs_uint(s, ps.cycle_avg_ms);
  hs_str(s, ",\"cycle_max_ms\":"); hs_uint(s, ps.cycle_max_ms);
  hs_str(s, ",\"rs485_polls\":"); hs_uint(s, ps.analog_polls_attempted);
  hs_str(s, ",\"rs485_ok\":"); hs_uint(s, ps.analog_polls_ok);
  hs_str(s, ",\"rs485_timeouts\":"); hs_uint(s, ps.analog_polls_timeout);
  hs_str(s, ",\"rs485_parse_err\":"); hs_uint(s, ps.analog_polls_parse_err);
  hs_str(s, ",\"alarm_polls_ok\":"); hs_uint(s, ps.alarm_polls_ok);
  hs_str(s, ",\"alarm_polls_err\":"); hs_uint(s, ps.alarm_polls_err);
  hs_str(s, "}");

  // ── can ───────────────────────────────────────────────────────────────────
  hs_str(s, ",\"can\":{");
  hs_str(s, "\"tx_ok\":"); hs_uint(s, cs.tx_ok);
  hs_str(s, ",\"tx_fail\":"); hs_uint(s, cs.tx_fail);
  hs_str(s, ",\"tx_fail_streak_max\":"); hs_uint(s, cs.tx_fail_streak_max);
  hs_str(s, ",\"heartbeats\":"); hs_uint(s, cs.heartbeats);
  hs_str(s, ",\"express_sends\":"); hs_uint(s, cs.express_sends);
  hs_str(s, ",\"bus_off_count\":"); hs_uint(s, cs.bus_off_count);
  hs_str(s, ",\"driver_restarts\":"); hs_uint(s, cs.driver_restart_count);
  hs_str(s, "}");

  // ── snapshot_bus ──────────────────────────────────────────────────────────
  hs_str(s, ",\"snapshot_bus\":{");
  hs_str(s, "\"publishes\":"); hs_uint(s, bus::snapshot_bus::total_publishes());
  hs_str(s, ",\"reads\":"); hs_uint(s, bus::snapshot_bus::total_reads());
  hs_str(s, ",\"retries\":"); hs_uint(s, bus::snapshot_bus::total_read_retries());
  hs_str(s, "}");

  // ── mqtt ──────────────────────────────────────────────────────────────────
  const char* mqtt_state_str = "disabled";
  switch (mqtt::publisher::get_state()) {
    case mqtt::publisher::State::Disconnected: mqtt_state_str = "disconnected"; break;
    case mqtt::publisher::State::Connecting:   mqtt_state_str = "connecting";   break;
    case mqtt::publisher::State::Connected:    mqtt_state_str = "connected";    break;
    case mqtt::publisher::State::Failed:       mqtt_state_str = "failed";       break;
    default: break;
  }
  hs_str(s, ",\"mqtt\":{");
  hs_str(s, "\"enabled\":"); hs_bool(s, app::get_config().mqtt_enabled);
  hs_str(s, ",\"state\":"); hs_json_str(s, mqtt_state_str);
  hs_str(s, ",\"publish_ok\":"); hs_uint(s, mqtt::publisher::get_publish_ok());
  hs_str(s, ",\"publish_fail\":"); hs_uint(s, mqtt::publisher::get_publish_fail());
  hs_str(s, ",\"publish_drops\":"); hs_uint(s, mqtt::publisher::get_publish_drops());
  hs_str(s, "}");

  // ── ntp ───────────────────────────────────────────────────────────────────
  hs_str(s, ",\"ntp\":{");
  hs_str(s, "\"synced\":"); hs_bool(s, net::ntp::is_synced());
  hs_str(s, ",\"last_sync_ts\":"); hs_uint(s, net::ntp::now_unix_s());
  hs_str(s, ",\"server\":"); hs_json_str(s, app::get_config().ntp_server);
  hs_str(s, "}");

  // ── history ───────────────────────────────────────────────────────────────
  hs_str(s, ",\"history\":{");
  hs_str(s, "\"fine_samples\":"); hs_uint(s, storage::history_store::fine_count());
  hs_str(s, ",\"coarse_samples\":"); hs_uint(s, storage::history_store::coarse_count());
  hs_str(s, "}");

  // ── energy ────────────────────────────────────────────────────────────────
  hs_str(s, ",\"energy\":{");
  hs_str(s, "\"today_in_kwh\":"); hs_f2(s, storage::energy_store::today_in_kwh());
  hs_str(s, ",\"today_out_kwh\":"); hs_f2(s, storage::energy_store::today_out_kwh());
  hs_str(s, ",\"total_in_kwh\":"); hs_f2(s, storage::energy_store::total_in_kwh());
  hs_str(s, ",\"total_out_kwh\":"); hs_f2(s, storage::energy_store::total_out_kwh());
  hs_str(s, "}");

  // ── littlefs ──────────────────────────────────────────────────────────────
  hs_str(s, ",\"littlefs\":{");
  hs_str(s, "\"total_b\":"); hs_uint(s, storage::lfs::total_bytes());
  hs_str(s, ",\"used_b\":"); hs_uint(s,
      storage::lfs::total_bytes() - storage::lfs::free_bytes());
  hs_str(s, ",\"free_b\":"); hs_uint(s, storage::lfs::free_bytes());
  hs_str(s, "}");

  // ── alerts summary ────────────────────────────────────────────────────────
  hs_str(s, ",\"alerts_count\":"); hs_uint(s, storage::alerts_store::stored_count());

  // ── log_ring ──────────────────────────────────────────────────────────────
  hs_str(s, ",");
  hs_log_ring(s);

  hs_str(s, "}");
  hs_flush(s);
  if (s.err == ESP_OK) httpd_resp_send_chunk(req, nullptr, 0);
  else ESP_LOGD(TAG, "client disconnected during /api/diag");
  return s.err;
}

}  // namespace web
