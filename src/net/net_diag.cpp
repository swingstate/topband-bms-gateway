#include "net_diag.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "esp_netif.h"
#include "esp_tls.h"
#include "esp_crt_bundle.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lwip/netdb.h"
#include "lwip/sockets.h"
#include "lwip/inet.h"
#include <time.h>
#include <cstdio>
#include <cstring>

static const char* TAG = "net_diag";

// ── Module state ──────────────────────────────────────────────────────────────

static net::diag::Report s_report = {};
static portMUX_TYPE      s_mux    = portMUX_INITIALIZER_UNLOCKED;

// ── Stage helpers ─────────────────────────────────────────────────────────────

static void run_stage_wifi(net::diag::Stage& s) {
  s.run = true;
  s.pass = false;

  wifi_ap_record_t ap = {};
  if (esp_wifi_sta_get_ap_info(&ap) != ESP_OK) {
    snprintf(s.detail, sizeof(s.detail), "Not associated (WiFi not connected)");
    return;
  }

  esp_netif_t* netif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
  esp_netif_ip_info_t ip = {};
  if (netif) esp_netif_get_ip_info(netif, &ip);

  char ip_str[16]  = "?";
  char gw_str[16]  = "?";
  char nm_str[16]  = "?";
  if (netif) {
    inet_ntoa_r(ip.ip,      ip_str, sizeof(ip_str));
    inet_ntoa_r(ip.gw,      gw_str, sizeof(gw_str));
    inet_ntoa_r(ip.netmask, nm_str, sizeof(nm_str));
  }

  snprintf(s.detail, sizeof(s.detail),
           "BSSID: %02X:%02X:%02X:%02X:%02X:%02X  RSSI: %d dBm  "
           "IP: %s  GW: %s  mask: %s",
           ap.bssid[0], ap.bssid[1], ap.bssid[2],
           ap.bssid[3], ap.bssid[4], ap.bssid[5],
           ap.rssi, ip_str, gw_str, nm_str);
  s.pass = true;
}

static void run_stage_dns(net::diag::Stage& s) {
  s.run = true;
  s.pass = false;

  struct addrinfo hints = {};
  hints.ai_family   = AF_INET;
  hints.ai_socktype = SOCK_STREAM;
  struct addrinfo* res = nullptr;

  int rc = getaddrinfo("api.telegram.org", "443", &hints, &res);
  if (rc != 0 || !res) {
    snprintf(s.detail, sizeof(s.detail),
             "getaddrinfo failed (code %d)", rc);
    return;
  }

  char ip_str[16] = {};
  auto* sin = reinterpret_cast<struct sockaddr_in*>(res->ai_addr);
  inet_ntoa_r(sin->sin_addr, ip_str, sizeof(ip_str));
  snprintf(s.detail, sizeof(s.detail),
           "api.telegram.org -> %s", ip_str);
  s.pass = true;

  // Store resolved IP for the TCP stage (passed via a static to avoid alloc).
  // This function is called sequentially before tcp stage, so no race.
  freeaddrinfo(res);
}

// Resolves host again for TCP — simpler than passing the addrinfo across stages.
static bool resolve_for_tcp(const char* host, uint16_t port,
                             struct sockaddr_in* out) {
  struct addrinfo hints = {};
  hints.ai_family   = AF_INET;
  hints.ai_socktype = SOCK_STREAM;
  struct addrinfo* res = nullptr;
  if (getaddrinfo(host, nullptr, &hints, &res) != 0 || !res) return false;
  memcpy(out, res->ai_addr, sizeof(*out));
  out->sin_port = htons(port);
  freeaddrinfo(res);
  return true;
}

static void run_stage_tcp(net::diag::Stage& s) {
  s.run = true;
  s.pass = false;

  struct sockaddr_in addr = {};
  if (!resolve_for_tcp("api.telegram.org", 443, &addr)) {
    snprintf(s.detail, sizeof(s.detail),
             "DNS resolution failed — cannot attempt TCP connect");
    return;
  }

  int fd = socket(AF_INET, SOCK_STREAM, 0);
  if (fd < 0) {
    snprintf(s.detail, sizeof(s.detail), "socket() failed (errno %d)", errno);
    return;
  }

  struct timeval tv = { .tv_sec = 5, .tv_usec = 0 };
  setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
  setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));

  char ip_str[16] = {};
  inet_ntoa_r(addr.sin_addr, ip_str, sizeof(ip_str));

  int rc = connect(fd, reinterpret_cast<struct sockaddr*>(&addr), sizeof(addr));
  close(fd);

  if (rc != 0) {
    snprintf(s.detail, sizeof(s.detail),
             "TCP connect to %s:443 failed (errno %d)", ip_str, errno);
  } else {
    snprintf(s.detail, sizeof(s.detail),
             "TCP connect to %s:443 OK", ip_str);
    s.pass = true;
  }
}

static void run_stage_tls(net::diag::Stage& s) {
  s.run = true;
  s.pass = false;

  esp_tls_cfg_t cfg = {};
  cfg.crt_bundle_attach = esp_crt_bundle_attach;
  cfg.timeout_ms        = 8000;

  uint32_t dram_before = heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL);
  ESP_LOGI(TAG, "[self-test] TLS stage: DRAM largest before = %lu B",
           (unsigned long)dram_before);

  esp_tls_t* tls = esp_tls_init();
  if (!tls) {
    snprintf(s.detail, sizeof(s.detail), "esp_tls_init OOM");
    return;
  }

  int rc = esp_tls_conn_http_new_sync("https://api.telegram.org/", &cfg, tls);

  int tls_code = 0, tls_flags = 0;
  if (rc != 1) {
    esp_tls_error_handle_t err_h = nullptr;
    if (esp_tls_get_error_handle(tls, &err_h) == ESP_OK && err_h) {
      esp_tls_get_and_clear_last_error(err_h, &tls_code, &tls_flags);
    }
    esp_tls_conn_destroy(tls);

    uint32_t dram_after = heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL);
    ESP_LOGW(TAG, "[self-test] TLS failed: rc=%d code=-0x%04X flags=0x%04X "
                  "DRAM_largest after=%lu B",
             rc, (unsigned)(-tls_code), (unsigned)tls_flags,
             (unsigned long)dram_after);

    if (tls_code != 0) {
      if (tls_flags != 0) {
        snprintf(s.detail, sizeof(s.detail),
                 "Handshake failed: mbedTLS -0x%04X (cert flags 0x%04X)",
                 (unsigned)(-tls_code), (unsigned)tls_flags);
      } else {
        snprintf(s.detail, sizeof(s.detail),
                 "Handshake failed: mbedTLS -0x%04X",
                 (unsigned)(-tls_code));
      }
    } else {
      snprintf(s.detail, sizeof(s.detail),
               "Socket timeout — no mbedTLS error code (rc=%d)", rc);
    }
  } else {
    esp_tls_conn_destroy(tls);
    snprintf(s.detail, sizeof(s.detail), "TLS handshake OK");
    s.pass = true;
    ESP_LOGI(TAG, "[self-test] TLS stage: OK");
  }
}

static void run_stage_ntp(net::diag::Stage& s) {
  s.run = true;
  s.pass = false;

  time_t now = time(nullptr);
  struct tm t = {};
  gmtime_r(&now, &t);
  int year = t.tm_year + 1900;

  if (year < 2024) {
    snprintf(s.detail, sizeof(s.detail),
             "Clock not synced (year=%d) — cert validation requires correct time",
             year);
  } else {
    snprintf(s.detail, sizeof(s.detail),
             "UTC %04d-%02d-%02d %02d:%02d:%02d",
             year, t.tm_mon + 1, t.tm_mday,
             t.tm_hour, t.tm_min, t.tm_sec);
    s.pass = true;
  }
}

// ── Background task ────────────────────────────────────────────────────────────

static void diag_task(void*) {
  ESP_LOGI(TAG, "[self-test] starting 5-stage network diagnostic");

  // Initialise report
  net::diag::Report r = {};
  r.running    = true;
  r.started_at = (uint32_t)(esp_timer_get_time() / 1000000LL);

  portENTER_CRITICAL(&s_mux);
  s_report = r;
  portEXIT_CRITICAL(&s_mux);

  run_stage_wifi(r.wifi);
  ESP_LOGI(TAG, "[self-test] wifi: %s — %s",
           r.wifi.pass ? "PASS" : "FAIL", r.wifi.detail);

  run_stage_dns(r.dns);
  ESP_LOGI(TAG, "[self-test] dns: %s — %s",
           r.dns.pass ? "PASS" : "FAIL", r.dns.detail);

  run_stage_tcp(r.tcp);
  ESP_LOGI(TAG, "[self-test] tcp: %s — %s",
           r.tcp.pass ? "PASS" : "FAIL", r.tcp.detail);

  run_stage_tls(r.tls);
  ESP_LOGI(TAG, "[self-test] tls: %s — %s",
           r.tls.pass ? "PASS" : "FAIL", r.tls.detail);

  run_stage_ntp(r.ntp);
  ESP_LOGI(TAG, "[self-test] ntp: %s — %s",
           r.ntp.pass ? "PASS" : "FAIL", r.ntp.detail);

  r.running = false;

  portENTER_CRITICAL(&s_mux);
  s_report = r;
  portEXIT_CRITICAL(&s_mux);

  ESP_LOGI(TAG, "[self-test] complete: wifi=%d dns=%d tcp=%d tls=%d ntp=%d",
           r.wifi.pass, r.dns.pass, r.tcp.pass, r.tls.pass, r.ntp.pass);

  vTaskDelete(nullptr);
}

// ── Public API ─────────────────────────────────────────────────────────────────

namespace net::diag {

bool start() {
  portENTER_CRITICAL(&s_mux);
  bool already = s_report.running;
  portEXIT_CRITICAL(&s_mux);
  if (already) return false;

  // Mark running immediately so a second POST in flight doesn't race.
  portENTER_CRITICAL(&s_mux);
  s_report = {};
  s_report.running = true;
  portEXIT_CRITICAL(&s_mux);

  // Stack 6 KB: TLS handshake + lwIP DNS resolver each need ~2 KB.
  BaseType_t ok = xTaskCreate(diag_task, "net_diag", 6144, nullptr, 2, nullptr);
  if (ok != pdPASS) {
    portENTER_CRITICAL(&s_mux);
    s_report.running = false;
    portEXIT_CRITICAL(&s_mux);
    ESP_LOGE(TAG, "xTaskCreate failed");
    return false;
  }
  return true;
}

bool is_running() {
  portENTER_CRITICAL(&s_mux);
  bool r = s_report.running;
  portEXIT_CRITICAL(&s_mux);
  return r;
}

Report get_report() {
  portENTER_CRITICAL(&s_mux);
  Report r = s_report;
  portEXIT_CRITICAL(&s_mux);
  return r;
}

}  // namespace net::diag
