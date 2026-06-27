#include "handlers_drift.h"
#include "app/drift_ring.h"
#include "bus/snapshot_bus.h"
#include "net/ntp.h"
#include "esp_log.h"
#include "esp_attr.h"
#include <cstdio>
#include <cstring>

static const char* TAG = "web_drift";

// PSRAM BSS snapshot buffer — avoids a ~5.6 KB stack frame (H1 lesson).
// HttpTask is single-threaded so a static buffer is safe.
static EXT_RAM_BSS_ATTR BmsSystemSnapshot s_drift_snap;

// ── Streaming JSON helpers (same pattern as handlers_history.cpp) ─────────────
struct DStream {
  httpd_req_t* req;
  char         buf[2048];
  size_t       pos;
  esp_err_t    err;
};

static void ds_flush(DStream& s) {
  if (s.pos > 0 && s.err == ESP_OK)
    s.err = httpd_resp_send_chunk(s.req, s.buf, (ssize_t)s.pos);
  s.pos = 0;
}

static void ds_raw(DStream& s, const char* d, size_t n) {
  while (n > 0 && s.err == ESP_OK) {
    size_t sp = sizeof(s.buf) - s.pos;
    size_t cp = n < sp ? n : sp;
    memcpy(s.buf + s.pos, d, cp);
    s.pos += cp; n -= cp; d += cp;
    if (s.pos == sizeof(s.buf)) ds_flush(s);
  }
}
static void ds_str(DStream& s, const char* str) { ds_raw(s, str, strlen(str)); }
static void ds_uint(DStream& s, uint32_t v) {
  char t[12]; snprintf(t, sizeof(t), "%u", (unsigned)v); ds_str(s, t);
}
static void ds_f1(DStream& s, float v) {
  char t[16]; snprintf(t, sizeof(t), "%.1f", v); ds_str(s, t);
}

namespace web {

esp_err_t handle_drift(httpd_req_t* req) {
  const bool has_snap = bus::snapshot_bus::read(s_drift_snap);
  const uint32_t now_s = net::ntp::now_unix_s();

  httpd_resp_set_type(req, "application/json");
  httpd_resp_set_hdr(req, "Cache-Control", "no-cache");

  DStream s = { req, {}, 0, ESP_OK };

  ds_str(s, "{\"now_ts_s\":"); ds_uint(s, now_s);
  ds_str(s, ",\"packs\":[");

  const uint8_t n_cfg = has_snap ? s_drift_snap.pack_count_configured : 0u;
  bool first_pack = true;

  for (uint8_t pi = 0; pi < n_cfg; pi++) {
    const BmsPackSnapshot& p = s_drift_snap.pack[pi];
    if (!p.online) continue;

    app::drift_ring::PackDrift dr = {};
    const bool has_drift = app::drift_ring::read_pack(pi, dr);

    if (!first_pack) ds_str(s, ",");
    first_pack = false;

    char name[16];
    snprintf(name, sizeof(name), "BMS %u", (unsigned)(pi + 1u));
    ds_str(s, "{\"id\":"); ds_uint(s, pi);
    ds_str(s, ",\"name\":\""); ds_str(s, name); ds_str(s, "\"");
    ds_str(s, ",\"online\":true");
    ds_str(s, ",\"cell_count\":"); ds_uint(s, p.cell_count);

    // Live spread (mV) — always available.
    const uint16_t spread_now =
        (p.cell_count > 0u)
        ? (uint16_t)(p.cell_drift_v * 1000.0f + 0.5f)
        : 0u;
    ds_str(s, ",\"spread_now\":"); ds_uint(s, spread_now);

    // Region-gated spreads (only meaningful when region was visited in window).
    ds_str(s, ",\"has_toc\":"); ds_str(s, (has_drift && dr.has_toc) ? "true" : "false");
    ds_str(s, ",\"has_bod\":"); ds_str(s, (has_drift && dr.has_bod) ? "true" : "false");
    ds_str(s, ",\"toc_spread\":"); ds_uint(s, has_drift ? dr.toc_spread : 0u);
    ds_str(s, ",\"bod_spread\":"); ds_uint(s, has_drift ? dr.bod_spread : 0u);

    // First full / first empty (zero when region not yet visited).
    ds_str(s, ",\"first_full_idx\":"); ds_uint(s, has_drift ? (uint32_t)dr.first_full_idx : 0u);
    ds_str(s, ",\"first_full_mv\":");  ds_uint(s, has_drift ? dr.first_full_mv  : 0u);
    ds_str(s, ",\"first_empty_idx\":"); ds_uint(s, has_drift ? (uint32_t)dr.first_empty_idx : 0u);
    ds_str(s, ",\"first_empty_mv\":");  ds_uint(s, has_drift ? dr.first_empty_mv : 0u);

    // Trend: ToC spread per day, oldest-first (used for drift-rate slope).
    ds_str(s, ",\"n_trend\":"); ds_uint(s, dr.n_days);
    ds_str(s, ",\"trend\":[");
    for (uint8_t d = 0; d < dr.n_days; d++) {
      if (d > 0) ds_str(s, ",");
      ds_uint(s, dr.trend_spread[d]);
    }
    ds_str(s, "]");

    // Drift rate: linear least-squares slope over ToC trend points (mV/day).
    // Positive = spread widening (worsening). Only meaningful with ToC data.
    float drift_rate = 0.0f;
    if (has_drift && dr.has_toc && dr.n_days >= 2u) {
      float sx = 0.0f, sy = 0.0f, sxx = 0.0f, sxy = 0.0f;
      const float n = (float)dr.n_days;
      for (uint8_t d = 0; d < dr.n_days; d++) {
        const float x = (float)d;
        const float y = (float)dr.trend_spread[d];
        sx += x; sy += y; sxx += x * x; sxy += x * y;
      }
      const float denom = n * sxx - sx * sx;
      if (denom != 0.0f) drift_rate = (n * sxy - sx * sy) / denom;
    }
    ds_str(s, ",\"drift_rate\":"); ds_f1(s, drift_rate);
    ds_str(s, ",\"has_history\":"); ds_str(s, has_drift ? "true" : "false");

    // Per-cell data.
    const uint8_t nc = (p.cell_count < app::drift_ring::DRIFT_MAX_CELLS)
                       ? p.cell_count
                       : (uint8_t)app::drift_ring::DRIFT_MAX_CELLS;
    ds_str(s, ",\"cells\":[");
    for (uint8_t ci = 0; ci < nc; ci++) {
      if (ci > 0) ds_str(s, ",");

      const float cv = p.cell_v[ci];
      const uint16_t now_mv = (cv > 0.5f)
                              ? (uint16_t)(cv * 1000.0f + 0.5f)
                              : 0u;
      ds_str(s, "{\"now\":"); ds_uint(s, now_mv);

      if (has_drift && ci < dr.cell_count) {
        const auto& c = dr.cells[ci];
        ds_str(s, ",\"d5min\":"); ds_uint(s, c.d5min);
        ds_str(s, ",\"d5max\":"); ds_uint(s, c.d5max);
        ds_str(s, ",\"evMin\":"); ds_uint(s, c.ev_min);
        ds_str(s, ",\"evMax\":"); ds_uint(s, c.ev_max);
        ds_str(s, ",\"tocMin\":"); ds_uint(s, c.toc_min);
        ds_str(s, ",\"tocMax\":"); ds_uint(s, c.toc_max);
        ds_str(s, ",\"bodMin\":"); ds_uint(s, c.bod_min);
        ds_str(s, ",\"bodMax\":"); ds_uint(s, c.bod_max);
      } else {
        ds_str(s, ",\"d5min\":0,\"d5max\":0,\"evMin\":0,\"evMax\":0");
        ds_str(s, ",\"tocMin\":0,\"tocMax\":0,\"bodMin\":0,\"bodMax\":0");
      }
      ds_str(s, "}");
    }
    ds_str(s, "]}");
  }

  ds_str(s, "]}");
  ds_flush(s);
  if (s.err == ESP_OK) httpd_resp_send_chunk(req, nullptr, 0);
  else ESP_LOGD(TAG, "client disconnected during /api/drift");
  return s.err;
}

}  // namespace web
