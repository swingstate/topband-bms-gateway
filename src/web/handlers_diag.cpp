#include "handlers_diag.h"
#include "handlers_live.h"
#include "bms/poller.h"
#include "can/tx.h"
#include "bus/snapshot_bus.h"
#include "mqtt/publisher.h"
#include "net/ntp.h"
#include "net/wifi.h"
#include "storage/lfs_store.h"
#include "storage/energy_store.h"
#include "storage/history_store.h"
#include "storage/boot_reasons.h"
#include "storage/alerts_store.h"
#include "diag/log_ring.h"
#include "diag/coredump_probe.h"
#include "app/version.h"
#include "app/boot.h"
#include "app/housekeeping.h"
#include "notify/notify.h"
#include "sources/registry.h"
#include "sources/ble_scanner.h"
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "esp_core_dump.h"
#include "esp_partition.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <cstring>
#include <cstdio>
#include <cstdlib>

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
  // Heap-allocate the snapshot buffer (24 KB). PSRAM preferred to avoid
  // fragmenting the internal heap with this large one-shot allocation.
  char* snap = static_cast<char*>(
      heap_caps_malloc(LOG_SNAPSHOT_SIZE, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
  if (!snap) snap = static_cast<char*>(malloc(LOG_SNAPSHOT_SIZE));
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

  uint32_t uptime_s         = (uint32_t)(esp_timer_get_time() / 1000000LL);
  uint32_t heap_free        = esp_get_free_heap_size();
  uint32_t dram_free        = heap_caps_get_free_size(MALLOC_CAP_INTERNAL);
  uint32_t dram_min         = heap_caps_get_minimum_free_size(MALLOC_CAP_INTERNAL);
  uint32_t dram_largest     = heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL);
  uint32_t psram_free       = heap_caps_get_free_size(MALLOC_CAP_SPIRAM);
  uint32_t psram_largest    = heap_caps_get_largest_free_block(MALLOC_CAP_SPIRAM);
  float    cpu_temp_c       = app::housekeeping::get_cpu_temp_c();

  hs_str(s, "{");

  // ── system ────────────────────────────────────────────────────────────────
  hs_str(s, "\"system\":{");
  hs_str(s, "\"fw\":"); hs_json_str(s, FW_VERSION_FULL);
  hs_str(s, ",\"uptime_s\":"); hs_uint(s, uptime_s);
  hs_str(s, ",\"free_heap\":"); hs_uint(s, heap_free);
  // DRAM (internal SRAM) — primary fragmentation victim. Separate from PSRAM
  // so a large free PSRAM no longer hides DRAM exhaustion.
  // Per docs/diag-mqtt-crash-review.md Finding 10.
  hs_str(s, ",\"dram_free\":"); hs_uint(s, dram_free);
  hs_str(s, ",\"dram_min\":"); hs_uint(s, dram_min);
  hs_str(s, ",\"dram_largest_block\":"); hs_uint(s, dram_largest);
  hs_str(s, ",\"psram_free\":"); hs_uint(s, psram_free);
  hs_str(s, ",\"psram_largest_block\":"); hs_uint(s, psram_largest);

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
  // ESP32-S3 internal die temperature (-128.0f sentinel = unavailable).
  if (cpu_temp_c > -100.0f) {
    char t[16]; snprintf(t, sizeof(t), "%.1f", cpu_temp_c);
    hs_str(s, ",\"cpu_temp_c\":"); hs_str(s, t);
  } else {
    hs_str(s, ",\"cpu_temp_c\":null");
  }
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
  hs_str(s, ",\"sysparam_polls_ok\":"); hs_uint(s, ps.sysparam_polls_ok);
  hs_str(s, ",\"sysparam_polls_err\":"); hs_uint(s, ps.sysparam_polls_err);
  hs_str(s, ",\"wrong_addr\":"); hs_uint(s, ps.wrong_addr);
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
  hs_str(s, ",\"publish_max_ms\":"); hs_uint(s, mqtt::publisher::get_publish_max_ms());
  // Effective base topic (configured prefix + MAC suffix) — needed when
  // debugging HA discovery; previously only visible in the boot log.
  {
    char base[80] = {};
    mqtt::publisher::get_effective_base(base, sizeof(base));
    hs_str(s, ",\"effective_base\":"); hs_json_str(s, base);
  }
  hs_str(s, "}");

  // ── ntp ───────────────────────────────────────────────────────────────────
  // now_ts_s is the CURRENT device time (review F4: the old field name
  // last_sync_ts was a lie — the module does not track sync time).
  hs_str(s, ",\"ntp\":{");
  hs_str(s, "\"synced\":"); hs_bool(s, net::ntp::is_synced());
  hs_str(s, ",\"now_ts_s\":"); hs_uint(s, net::ntp::now_unix_s());
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

  // ── coredump ──────────────────────────────────────────────────────────────
  // Cached boot-time probe. NEVER parse the dump per request: the summary
  // parse reads flash, and a stale BIN-era dump under the ELF config cost
  // ~33 s per /api/diag request, starving the single-threaded HTTP server
  // (preview.3 field regression).
  {
    const diag::coredump::ProbeResult& cd = diag::coredump::probe();
    hs_str(s, ",\"coredump\":{");
    hs_str(s, "\"present\":"); hs_bool(s, cd.present);
    if (cd.has_summary) {
      hs_str(s, ",\"crashing_task\":");     hs_json_str(s, cd.task);
      hs_str(s, ",\"build_sha256\":");      hs_json_str(s, cd.sha256);
      hs_str(s, ",\"exc_pc\":");            hs_json_str(s, cd.pc_hex);
      hs_str(s, ",\"core_dump_version\":"); hs_uint(s, cd.version);
    }
    hs_str(s, "}");
  }

  // ── ble_status — BLE scanner, MPPT, and Shunt status ────────────────────
  {
    bool ble_on = sources::ble_scanner::is_active();

    hs_str(s, ",\"ble_status\":{");
    hs_str(s, "\"ble_active\":"); hs_bool(s, ble_on);
    hs_str(s, ",\"stack\":"); hs_json_str(s, sources::ble_scanner::stack_name());
    hs_str(s, ",\"wifi_disconnects\":"); hs_uint(s, net::wifi::get_disconnect_count());
    { std::string bssid = net::wifi::get_bssid();
      hs_str(s, ",\"wifi_bssid\":"); hs_json_str(s, bssid.empty() ? "" : bssid.c_str()); }
    hs_str(s, ",\"wifi_rssi\":"); { char t[8]; snprintf(t,sizeof(t),"%d",(int)net::wifi::get_rssi()); hs_str(s,t); }
    hs_str(s, ",\"wifi_bssid_pin_active\":"); hs_bool(s, net::wifi::is_bssid_pin_active());
    // Connection identity — a diagnosing user should not need the router UI
    // to see which network/IP the gateway is on (review Part 3.2).
    { std::string ssid = net::wifi::get_ssid();
      hs_str(s, ",\"wifi_ssid\":"); hs_json_str(s, ssid.empty() ? "" : ssid.c_str()); }
    { char ip[24] = {}; net::wifi::get_ip(ip, sizeof(ip));
      hs_str(s, ",\"wifi_ip\":"); hs_json_str(s, ip); }
    { char hn[48] = {}; net::wifi::get_hostname(hn, sizeof(hn));
      hs_str(s, ",\"wifi_hostname\":"); hs_json_str(s, hn); }
    hs_str(s, ",\"wifi_connected_for_s\":"); hs_uint(s, net::wifi::connected_for_s());
    hs_str(s, ",\"handler_last_ms\":"); hs_uint(s, web::live_handler_last_ms());
    hs_str(s, ",\"handler_max_ms\":"); hs_uint(s, web::live_handler_max_ms());
    hs_str(s, ",\"ble_gap_events\":"); hs_uint(s, sources::ble_scanner::gap_event_count());
    hs_str(s, ",\"ble_victron_advs\":"); hs_uint(s, sources::ble_scanner::victron_adv_count());
    hs_str(s, ",\"ble_mppt_advs\":"); hs_uint(s, sources::ble_scanner::mppt_adv_count());

    // BMS total current (for side-by-side comparison with shunt current).
    {
      BmsSystemSnapshot snap{};
      float bms_total_current = 0.0f;
      if (bus::snapshot_bus::read(snap)) {
        for (uint8_t i = 0; i < snap.pack_count_configured && i < 16; ++i) {
          if (snap.pack[i].online) bms_total_current += snap.pack[i].pack_current;
        }
      }
      hs_str(s, ",\"bms_current_a\":");
      { char t[16]; snprintf(t, sizeof(t), "%.3f", bms_total_current); hs_str(s, t); }
    }

    // MPPT BLE decode state.
    {
      sources::MpptSource* mppt = sources::mppt_source();
      bool mppt_enabled = mppt && mppt->enabled();
      hs_str(s, ",\"mppt\":{");
      hs_str(s, "\"enabled\":"); hs_bool(s, mppt_enabled);
      if (mppt_enabled) {
        sources::MpptSource::DiagSnap d = mppt->diag_snap();
        uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000LL);
        int32_t age_s = d.seen ? (int32_t)((now_ms - d.last_seen_ms) / 1000u) : -1;
        hs_str(s, ",\"seen\":"); hs_bool(s, d.seen);
        hs_str(s, ",\"last_seen_s\":"); { char t[12]; snprintf(t,sizeof(t),"%ld",(long)age_s); hs_str(s,t); }
        // null when the charger sent a Victron not-available sentinel (0x7FFF/0xFFFF)
        hs_str(s, ",\"pv_power_w\":");
        if (d.pv_power_valid)    { char t[16]; snprintf(t,sizeof(t),"%.1f",d.pv_power_w);    hs_str(s,t); }
        else hs_str(s, "null");
        hs_str(s, ",\"pv_voltage_v\":");
        if (d.pv_v_valid)        { char t[16]; snprintf(t,sizeof(t),"%.2f",d.pv_voltage_v);  hs_str(s,t); }
        else hs_str(s, "null");
        hs_str(s, ",\"pv_current_a\":");
        if (d.pv_i_valid)        { char t[16]; snprintf(t,sizeof(t),"%.2f",d.pv_current_a);  hs_str(s,t); }
        else hs_str(s, "null");
        hs_str(s, ",\"yield_today_wh\":");
        if (d.yield_valid)       { char t[16]; snprintf(t,sizeof(t),"%.0f",d.yield_today_wh); hs_str(s,t); }
        else hs_str(s, "null");
        hs_str(s, ",\"charge_state\":"); { char t[8]; snprintf(t,sizeof(t),"%d",(int)d.charge_state); hs_str(s,t); }
        hs_str(s, ",\"batt_voltage_v\":");
        if (d.batt_v_valid)      { char t[16]; snprintf(t,sizeof(t),"%.2f",d.batt_voltage_v); hs_str(s,t); }
        else hs_str(s, "null");
        hs_str(s, ",\"batt_current_a\":");
        if (d.batt_i_valid)      { char t[16]; snprintf(t,sizeof(t),"%.2f",d.batt_current_a); hs_str(s,t); }
        else hs_str(s, "null");
      }
      hs_str(s, "}");
    }

    // Shunt BLE decode state.
    {
      sources::ShuntSource* shunt = sources::shunt_source();
      bool shunt_enabled = shunt && shunt->enabled();
      hs_str(s, ",\"shunt\":{");
      hs_str(s, "\"enabled\":"); hs_bool(s, shunt_enabled);
      if (shunt_enabled) {
        sources::ShuntSource::DiagSnap d = shunt->diag_snap();
        uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000LL);
        int32_t age_s = d.seen ? (int32_t)((now_ms - d.last_seen_ms) / 1000u) : -1;
        hs_str(s, ",\"seen\":"); hs_bool(s, d.seen);
        hs_str(s, ",\"last_seen_s\":"); { char t[12]; snprintf(t,sizeof(t),"%ld",(long)age_s); hs_str(s,t); }
        hs_str(s, ",\"current_a\":");
        { char t[16]; snprintf(t,sizeof(t),"%.3f",d.current_a); hs_str(s,t); }
        hs_str(s, ",\"voltage_v\":");
        { char t[16]; snprintf(t,sizeof(t),"%.2f",d.voltage_v); hs_str(s,t); }
        hs_str(s, ",\"soc_pct\":");
        { char t[16]; snprintf(t,sizeof(t),"%.1f",d.soc_pct);   hs_str(s,t); }
        hs_str(s, ",\"soc_valid\":"); hs_bool(s, d.soc_valid);
        // Read-only diagnostic reference: shunt's own Coulomb counter. Not fused
        // into any dashboard/MQTT/CAN value — cross-check only.
        hs_str(s, ",\"consumed_ah_valid\":"); hs_bool(s, d.consumed_ah_valid);
        hs_str(s, ",\"consumed_ah\":");
        { char t[16]; snprintf(t,sizeof(t),"%.1f",d.consumed_ah); hs_str(s,t); }
      }
      hs_str(s, "}");
    }

    // Per-advertisement filter funnel: configured_mac, stage counters, and
    // a ring of the last ≤8 Victron advertisements with per-stage pass/fail flags.
    {
      sources::ble_scanner::AdvDebugState dbg{};
      sources::ble_scanner::get_adv_debug(dbg);

      hs_str(s, ",\"ble_debug\":{");
      hs_str(s, "\"configured_mac\":"); hs_json_str(s, dbg.configured_mac);
      hs_str(s, ",\"mppt_mac_valid\":"); hs_bool(s, dbg.mppt_mac_valid);
      hs_str(s, ",\"mppt_key_valid\":"); hs_bool(s, dbg.mppt_key_valid);
      hs_str(s, ",\"victron_total\":"); hs_uint(s, dbg.victron_total);
      hs_str(s, ",\"mppt_type_match\":"); hs_uint(s, dbg.mppt_type_match);
      hs_str(s, ",\"mppt_mac_match\":"); hs_uint(s, dbg.mppt_mac_match);
      hs_str(s, ",\"mppt_decrypt_ok\":"); hs_uint(s, dbg.mppt_decrypt_ok);
      hs_str(s, ",\"configured_shunt_mac\":"); hs_json_str(s, dbg.configured_shunt_mac);
      hs_str(s, ",\"shunt_mac_valid\":"); hs_bool(s, dbg.shunt_mac_valid);
      hs_str(s, ",\"shunt_key_valid\":"); hs_bool(s, dbg.shunt_key_valid);
      hs_str(s, ",\"shunt_type_match\":"); hs_uint(s, dbg.shunt_type_match);
      hs_str(s, ",\"shunt_mac_match\":"); hs_uint(s, dbg.shunt_mac_match);
      hs_str(s, ",\"shunt_decrypt_ok\":"); hs_uint(s, dbg.shunt_decrypt_ok);
      hs_str(s, ",\"shunt_last_mfg_len\":"); hs_uint(s, dbg.shunt_last_mfg_len);
      hs_str(s, ",\"shunt_last_new_fmt\":"); hs_bool(s, dbg.shunt_last_new_fmt);
      hs_str(s, ",\"adv_ring\":[");
      for (uint8_t i = 0; i < dbg.count; i++) {
        const sources::ble_scanner::AdvDebugEntry& e = dbg.entries[i];
        if (i > 0) hs_str(s, ",");
        char type_hex[7];
        snprintf(type_hex, sizeof(type_hex), "0x%02X", e.record_type);
        hs_str(s, "{\"mac\":"); hs_json_str(s, e.mac_str);
        hs_str(s, ",\"rssi\":"); { char t[8]; snprintf(t, sizeof(t), "%d", (int)e.rssi); hs_str(s, t); }
        hs_str(s, ",\"type\":"); hs_json_str(s, type_hex);
        hs_str(s, ",\"mac_match\":"); hs_bool(s, e.mac_match);
        hs_str(s, ",\"type_ok\":"); hs_bool(s, e.record_type_ok);
        hs_str(s, ",\"decrypt_ok\":"); hs_bool(s, e.decrypt_ok);
        hs_str(s, "}");
      }
      hs_str(s, "]}");
    }

    hs_str(s, "}");
  }

  // ── tasks (FreeRTOS stack HWM) ────────────────────────────────────────────
  // Requires CONFIG_FREERTOS_USE_TRACE_FACILITY=y and
  // CONFIG_FREERTOS_VTASKLIST_INCLUDE_COREID=y (sdkconfig.esp32s3).
  // Stack HWM in bytes (usStackHighWaterMark is in words; ×sizeof(StackType_t)=4).
  {
    static constexpr UBaseType_t MAX_TASKS = 24;
    TaskStatus_t task_buf[MAX_TASKS];
    UBaseType_t n = uxTaskGetSystemState(task_buf, MAX_TASKS, nullptr);
    hs_str(s, ",\"tasks\":[");
    for (UBaseType_t i = 0; i < n; i++) {
      if (i > 0) hs_str(s, ",");
      hs_str(s, "{\"name\":");
      hs_json_str(s, task_buf[i].pcTaskName);
      hs_str(s, ",\"stack_hwm\":");
      // usStackHighWaterMark is in words (StackType_t = 4 B on Xtensa); convert to bytes.
      // This is a since-boot low-water mark — sample AFTER full load for meaningful numbers.
      // NOTE: configured stack size (stack_cfg) is NOT available from the FreeRTOS SMP
      // TaskStatus_t (pxEndOfStack absent when configRECORD_STACK_HIGH_ADDRESS defaults to 0
      // in FreeRTOS-Kernel-SMP/FreeRTOS.h). Reclaimable = cfg_from_source - (cfg - hwm).
      // See sdkconfig.defaults and task creation sites for per-task configured sizes.
      hs_uint(s, (uint32_t)task_buf[i].usStackHighWaterMark * sizeof(StackType_t));
      // xCoreID requires configTASKLIST_INCLUDE_COREID=1 (FREERTOS_VTASKLIST_INCLUDE_COREID).
      // Guard so the handler compiles even if that option is off; UI shows "any" for -1.
      hs_str(s, ",\"core\":");
      {
        char cbuf[12];
#if configTASKLIST_INCLUDE_COREID
        int c = (task_buf[i].xCoreID == tskNO_AFFINITY) ? -1 : (int)task_buf[i].xCoreID;
#else
        int c = -1;
#endif
        snprintf(cbuf, sizeof(cbuf), "%d", c);
        hs_str(s, cbuf);
      }
      hs_str(s, ",\"prio\":");
      hs_uint(s, (uint32_t)task_buf[i].uxCurrentPriority);
      hs_str(s, "}");
    }
    hs_str(s, "]");
  }

  // ── log_ring ──────────────────────────────────────────────────────────────
  hs_str(s, ",");
  hs_log_ring(s);

  hs_str(s, "}");
  hs_flush(s);
  if (s.err == ESP_OK) httpd_resp_send_chunk(req, nullptr, 0);
  else ESP_LOGD(TAG, "client disconnected during /api/diag");
  return s.err;
}

esp_err_t handle_diag_coredump(httpd_req_t* req) {
  // Verify coredump CRC before serving — returns ESP_OK only if a valid dump
  // exists in the dedicated flash partition.
  // Per docs/diag-mqtt-crash-review.md Finding 9.
  if (esp_core_dump_image_check() != ESP_OK) {
    httpd_resp_set_status(req, "404 Not Found");
    httpd_resp_set_type(req, "text/plain");
    const char* msg =
        "No valid coredump in flash (device has not panicked since boot, "
        "or the previous dump was cleared).";
    return httpd_resp_send(req, msg, HTTPD_RESP_USE_STRLEN);
  }

  const esp_partition_t* part = esp_partition_find_first(
      ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_DATA_COREDUMP, nullptr);
  if (!part) {
    httpd_resp_set_status(req, "500 Internal Server Error");
    httpd_resp_set_type(req, "text/plain");
    return httpd_resp_sendstr(req, "Coredump partition not found in partition table");
  }

  httpd_resp_set_type(req, "application/octet-stream");
  httpd_resp_set_hdr(req, "Content-Disposition",
                     "attachment; filename=\"coredump.bin\"");

  // Stream in 512-byte chunks; static buffer avoids stack pressure.
  static uint8_t s_chunk[512];
  size_t offset = 0;
  while (offset < part->size) {
    size_t to_read = sizeof(s_chunk);
    if (offset + to_read > part->size) to_read = part->size - offset;
    esp_err_t err = esp_partition_read(part, offset, s_chunk, to_read);
    if (err != ESP_OK) {
      ESP_LOGW(TAG, "Partition read error at offset %zu: %s",
               offset, esp_err_to_name(err));
      break;
    }
    if (httpd_resp_send_chunk(req, (const char*)s_chunk, (ssize_t)to_read) != ESP_OK) {
      ESP_LOGD(TAG, "client disconnected during coredump stream");
      return ESP_OK;
    }
    offset += to_read;
  }
  httpd_resp_send_chunk(req, nullptr, 0);
  return ESP_OK;
}

// ── Dev TLS burst handler — POST /api/diag/tls-burst?n=<1..10> ───────────────
// Only compiled and registered when BLE_SPIKE_DEV_BURST=1.
// Starts the burst coordinator task in notify.cpp; returns immediately.
// Returns 409 if a burst is already in flight.
#if BLE_SPIKE_DEV_BURST
esp_err_t handle_diag_tls_burst(httpd_req_t* req) {
  httpd_resp_set_type(req, "application/json");

  if (notify::dev_tls_burst_active()) {
    httpd_resp_set_status(req, "409 Conflict");
    return httpd_resp_sendstr(req, "{\"error\":\"burst already active\"}");
  }

  // Parse n= query param; default 4.
  char qs[32] = {};
  int n = 4;
  if (httpd_req_get_url_query_str(req, qs, sizeof(qs)) == ESP_OK) {
    char n_str[8] = {};
    if (httpd_query_key_value(qs, "n", n_str, sizeof(n_str)) == ESP_OK) {
      int parsed = atoi(n_str);
      if (parsed >= 1 && parsed <= 10) n = parsed;
    }
  }

  if (!notify::dev_tls_burst_start(n)) {
    httpd_resp_set_status(req, "500 Internal Server Error");
    return httpd_resp_sendstr(req, "{\"error\":\"burst start failed\"}");
  }

  char resp[64];
  snprintf(resp, sizeof(resp), "{\"ok\":true,\"n\":%d}", n);
  return httpd_resp_sendstr(req, resp);
}
#endif  // BLE_SPIKE_DEV_BURST

}  // namespace web
