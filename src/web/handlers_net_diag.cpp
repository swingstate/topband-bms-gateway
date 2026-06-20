#include "handlers_net_diag.h"
#include "net/net_diag.h"
#include "esp_http_server.h"
#include <cstdio>
#include <cstring>

namespace {

// JSON-escape src into out[max]. Returns number of chars written (excl. null).
static size_t json_esc(const char* src, char* out, size_t max) {
  size_t n = 0;
  for (; *src && n < max - 2; ++src) {
    char c = *src;
    if (c == '"' || c == '\\') {
      if (n + 2 >= max) break;
      out[n++] = '\\';
    }
    out[n++] = c;
  }
  out[n] = '\0';
  return n;
}

static size_t append_stage(char* buf, size_t cap, size_t pos,
                            const char* id, const char* label,
                            const net::diag::Stage& st) {
  char esc[180];
  json_esc(st.detail, esc, sizeof(esc));
  int n = snprintf(buf + pos, cap - pos,
                   "{\"id\":\"%s\",\"label\":\"%s\","
                   "\"run\":%s,\"pass\":%s,\"detail\":\"%s\"}",
                   id, label,
                   st.run  ? "true" : "false",
                   st.pass ? "true" : "false",
                   esc);
  if (n > 0 && pos + (size_t)n < cap) pos += (size_t)n;
  return pos;
}

}  // namespace

namespace web {

esp_err_t handle_net_diag_post(httpd_req_t* req) {
  if (!net::diag::start()) {
    httpd_resp_set_status(req, "409 Conflict");
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_sendstr(req, "{\"error\":\"already running\"}");
  }
  httpd_resp_set_type(req, "application/json");
  return httpd_resp_sendstr(req, "{\"ok\":true,\"status\":\"running\"}");
}

esp_err_t handle_net_diag_get(httpd_req_t* req) {
  net::diag::Report r = net::diag::get_report();

  // Build JSON: ~1100 chars max.
  static constexpr size_t BUF = 1200;
  char buf[BUF];
  size_t pos = 0;

  {
    int n = snprintf(buf, BUF,
                     "{\"running\":%s,\"current_stage\":%d,\"started_at\":%lu,\"stages\":[",
                     r.running ? "true" : "false",
                     (int)r.current_stage,
                     (unsigned long)r.started_at);
    if (n > 0) pos = (size_t)n;
  }

  pos = append_stage(buf, BUF, pos, "wifi", "WiFi / link",          r.wifi);
  if (pos + 1 < BUF) buf[pos++] = ',';
  pos = append_stage(buf, BUF, pos, "dns",  "DNS resolution",        r.dns);
  if (pos + 1 < BUF) buf[pos++] = ',';
  pos = append_stage(buf, BUF, pos, "tcp",  "TCP connect",           r.tcp);
  if (pos + 1 < BUF) buf[pos++] = ',';
  pos = append_stage(buf, BUF, pos, "tls",  "TLS handshake",         r.tls);
  if (pos + 1 < BUF) buf[pos++] = ',';
  pos = append_stage(buf, BUF, pos, "ntp",  "Time / cert sanity",    r.ntp);

  if (pos + 3 < BUF) {
    buf[pos++] = ']';
    buf[pos++] = '}';
    buf[pos]   = '\0';
  }

  httpd_resp_set_type(req, "application/json");
  httpd_resp_set_hdr(req, "Cache-Control", "no-cache");
  return httpd_resp_sendstr(req, buf);
}

}  // namespace web
