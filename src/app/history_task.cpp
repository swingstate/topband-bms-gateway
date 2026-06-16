#include "history_task.h"
#include "storage/history_store.h"
#include "storage/energy_store.h"
#include "bus/snapshot_bus.h"
#include "bus/types.h"
#include "net/ntp.h"
#include "esp_log.h"
#include "esp_attr.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <climits>
#include <cstring>
#include <cstdint>
#include <algorithm>
#include "bms_snapshot.h"

static const char* TAG = "hist_task";

// ── Constants ─────────────────────────────────────────────────────────────────
// Fine sample cadence.
static constexpr uint32_t FINE_INTERVAL_MS  = HISTORY_FINE_RESOLUTION_S * 1000u; // 10 s
// Number of fine samples per coarse sample (5 min / 10 s = 30).
static constexpr uint32_t FINE_PER_COARSE   =
    HISTORY_COARSE_RESOLUTION_S / HISTORY_FINE_RESOLUTION_S; // 30

// ── Module-level statics (PSRAM BSS) — avoids large stack frames (H1 lesson) ──
// EXT_RAM_BSS_ATTR: ~8 KB moved from DRAM BSS to PSRAM. CPU-only read from
// snapshot_bus::read(); never DMA. Must NOT be a task local (H1 lesson).
static EXT_RAM_BSS_ATTR BmsSystemSnapshot s_snap;

// Fine-sample accumulator for downsampling to coarse (FINE_PER_COARSE = 30 slots).
// Each slot includes drift_mv; make_coarse_point averages it from these directly.
static HistoryFinePoint s_fine_window[FINE_PER_COARSE] = {};
static uint32_t s_fine_in_window = 0;   // how many fine points since last coarse
static uint32_t s_fine_total     = 0;   // total fine points appended ever

// ── Build a HistoryFinePoint from the current snapshot ───────────────────────
// drift_mv is computed here and stored directly in the returned struct so it
// is persisted to LittleFS alongside the other metrics (format v2).
static HistoryFinePoint make_fine_point(const BmsSystemSnapshot& snap) {
  HistoryFinePoint pt = {};
  float total_power_w = 0.0f;
  float sum_voltage   = 0.0f;
  float sum_temp      = 0.0f;
  float sum_soc       = 0.0f;
  float sys_cell_max  = 0.0f;
  float sys_cell_min  = 9.9f;
  uint8_t online      = 0;

  for (uint8_t i = 0; i < snap.pack_count_configured && i < 16; i++) {
    const BmsPackSnapshot& p = snap.pack[i];
    if (!p.online) continue;
    total_power_w += p.pack_current * p.pack_voltage;
    sum_voltage   += p.pack_voltage;
    sum_soc       += p.soc;
    sum_temp      += p.temp_avg_c;
    if (p.cell_count > 0) {
      if (p.cell_max_v > sys_cell_max) sys_cell_max = p.cell_max_v;
      if (p.cell_min_v < sys_cell_min) sys_cell_min = p.cell_min_v;
    }
    online++;
  }

  if (online > 0) {
    float inv = 1.0f / (float)online;
    pt.power_w      = (int16_t)(total_power_w + 0.5f);
    pt.voltage_x100 = (int16_t)((sum_voltage * inv) * 100.0f + 0.5f);
    pt.soc_x10      = (int16_t)((sum_soc * inv) * 10.0f + 0.5f);
    pt.temp_x10     = (int16_t)((sum_temp * inv) * 10.0f + 0.5f);
    pt.drift_mv     = (sys_cell_max > sys_cell_min)
                      ? (uint16_t)((sys_cell_max - sys_cell_min) * 1000.0f + 0.5f)
                      : 0;
  }

  uint32_t now = net::ntp::now_unix_s();
  pt.flags = (now != 0) ? 0x0001 : 0x0000;
  pt.t_offset_s = 0;  // filled in by caller
  return pt;
}

// ── Downsample FINE_PER_COARSE fine points into one coarse point ──────────────
// Drift is averaged from pts[i].drift_mv — no companion ring needed (format v2).
static HistoryCoarsePoint make_coarse_point(const HistoryFinePoint* pts, uint32_t n,
                                             uint32_t abs_epoch_s) {
  HistoryCoarsePoint cp = {};
  if (n == 0) return cp;

  int32_t sum_p = 0, min_p = INT16_MAX, max_p = INT16_MIN;
  int32_t sum_v = 0, min_v = INT16_MAX, max_v = INT16_MIN;
  int32_t sum_s = 0;
  int32_t sum_t = 0, min_t = INT16_MAX, max_t = INT16_MIN;
  uint32_t drift_sum = 0;
  uint32_t drift_valid = 0;
  uint32_t valid = 0;

  for (uint32_t i = 0; i < n; i++) {
    const HistoryFinePoint& p = pts[i];
    sum_p += p.power_w;
    min_p = std::min(min_p, (int32_t)p.power_w);
    max_p = std::max(max_p, (int32_t)p.power_w);
    sum_v += p.voltage_x100;
    min_v = std::min(min_v, (int32_t)p.voltage_x100);
    max_v = std::max(max_v, (int32_t)p.voltage_x100);
    sum_s += p.soc_x10;
    sum_t += p.temp_x10;
    min_t = std::min(min_t, (int32_t)p.temp_x10);
    max_t = std::max(max_t, (int32_t)p.temp_x10);
    if (p.drift_mv > 0) { drift_sum += p.drift_mv; drift_valid++; }
    valid++;
  }

  if (valid > 0) {
    float inv = 1.0f / (float)valid;
    cp.power_avg = (int16_t)((float)sum_p * inv);
    cp.power_min = (int16_t)min_p;
    cp.power_max = (int16_t)max_p;
    cp.volt_avg  = (int16_t)((float)sum_v * inv);
    cp.volt_min  = (int16_t)min_v;
    cp.volt_max  = (int16_t)max_v;
    cp.soc_avg   = (int16_t)((float)sum_s * inv);
    cp.temp_avg  = (int16_t)((float)sum_t * inv);
    cp.temp_min  = (int16_t)min_t;
    cp.temp_max  = (int16_t)max_t;
  }
  if (drift_valid > 0)
    cp.drift_mv = (uint16_t)(drift_sum / drift_valid);

  cp.t_epoch = abs_epoch_s;
  return cp;
}

// ── Task body ─────────────────────────────────────────────────────────────────
static void history_task_entry(void* /*arg*/) {
  ESP_LOGI(TAG, "started — fine interval %u s, coarse every %u fine samples",
           (unsigned)HISTORY_FINE_RESOLUTION_S, (unsigned)FINE_PER_COARSE);

  TickType_t tick = xTaskGetTickCount();

  for (;;) {
    vTaskDelayUntil(&tick, pdMS_TO_TICKS(FINE_INTERVAL_MS));

    // ── Read snapshot (safe: s_snap is BSS, not stack) ───────────────────────
    bool has_snap = bus::snapshot_bus::read(s_snap);
    if (!has_snap || s_snap.pack_count_online == 0) {
      // No data yet — still count the tick for coarse-cadence tracking.
      s_fine_in_window++;
      if (s_fine_in_window >= FINE_PER_COARSE) s_fine_in_window = 0;
      continue;
    }

    // ── Build fine point (drift_mv embedded in struct) ───────────────────────
    HistoryFinePoint fp = make_fine_point(s_snap);

    // Compute t_offset_s: offset from epoch_base stored in fine ring header.
    uint32_t now_s = net::ntp::now_unix_s();
    uint32_t base  = storage::history_store::fine_epoch_base();
    if (base == 0 && now_s != 0) {
      // First NTP-valid sample: anchor the ring's epoch_base.
      storage::history_store::set_fine_epoch_base(now_s);
      base = now_s;
    }
    fp.t_offset_s = (base != 0 && now_s >= base)
                    ? (uint16_t)std::min<uint32_t>(now_s - base, 0xFFFFu)
                    : 0;

    storage::history_store::append_fine(fp);
    s_fine_total++;

    // Store in rolling window for downsampling.
    s_fine_window[s_fine_in_window % FINE_PER_COARSE] = fp;
    s_fine_in_window++;

    ESP_LOGD(TAG, "fine sample #%u: power=%d V=%d SOC=%d T=%d",
             (unsigned)s_fine_total,
             (int)fp.power_w, (int)fp.voltage_x100,
             (int)fp.soc_x10, (int)fp.temp_x10);

    // ── Every FINE_PER_COARSE fine samples → produce coarse + flush ───────────
    if (s_fine_in_window >= FINE_PER_COARSE) {
      s_fine_in_window = 0;

      // Anchor time: use the end of the 5-min window (now_s), rounded down to 5min.
      uint32_t coarse_ts = (now_s != 0) ? (now_s / HISTORY_COARSE_RESOLUTION_S)
                                            * HISTORY_COARSE_RESOLUTION_S
                                        : 0;
      HistoryCoarsePoint cp = make_coarse_point(s_fine_window, FINE_PER_COARSE, coarse_ts);
      storage::history_store::append_coarse(cp);
      storage::history_store::flush();

      // Also persist energy counters every 5 min.
      storage::energy_store::persist();

      ESP_LOGI(TAG, "coarse sample: power_avg=%d soc_avg=%d ts=%u — persisted to LFS",
               (int)cp.power_avg, (int)cp.soc_avg, (unsigned)coarse_ts);
    }
  }
}

namespace app::history_task {

bool start() {
  static TaskHandle_t handle = nullptr;
  BaseType_t r = xTaskCreatePinnedToCore(
      history_task_entry,
      "hist",
      /*stack_depth*/ 4096,
      nullptr,
      /*priority*/ 1,
      &handle,
      /*core_id*/ 1
  );
  if (r != pdPASS) {
    ESP_LOGE(TAG, "task creation failed (%d)", (int)r);
    return false;
  }
  ESP_LOGI(TAG, "scheduled first sample in %u s", (unsigned)HISTORY_FINE_RESOLUTION_S);
  return true;
}

}  // namespace app::history_task
