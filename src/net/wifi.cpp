#include "wifi.h"
#include "app/self_test.h"
#include "diag/alerts.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "esp_mac.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/task.h"
#include <cstring>
#include <cstdio>
#include <algorithm>

static const char* TAG = "wifi";

static EventGroupHandle_t g_events   = nullptr;
static constexpr EventBits_t BIT_CONNECTED = BIT0;
static constexpr EventBits_t BIT_FAILED    = BIT1;
static constexpr EventBits_t BIT_SCAN_DONE = BIT2;

static net::wifi::Mode g_mode        = net::wifi::Mode::Off;
static bool            g_connected   = false;
static int             g_retry       = 0;
static constexpr int   MAX_RETRY     = 5;
static int64_t         g_connect_us  = 0;          // esp_timer_get_time() at connect
static char            g_hostname[32] = {};

static esp_netif_t* g_sta_netif = nullptr;
static esp_netif_t* g_ap_netif  = nullptr;

// ── Event handler ─────────────────────────────────────────────────────────────

static void on_wifi_event(void* arg, esp_event_base_t base,
                          int32_t id, void* data) {
  if (base == WIFI_EVENT && id == WIFI_EVENT_STA_START) {
    esp_wifi_connect();
  } else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
    bool was_connected = g_connected;
    g_connected = false;
    if (g_mode == net::wifi::Mode::StaConnecting && g_retry < MAX_RETRY) {
      g_retry++;
      ESP_LOGW(TAG, "STA disconnected — retry %d/%d", g_retry, MAX_RETRY);
      esp_wifi_connect();
    } else {
      ESP_LOGE(TAG, "STA connect failed after %d retries", g_retry);
      if (g_events) xEventGroupSetBits(g_events, BIT_FAILED);
    }
    if (was_connected) {
      diag::alerts::emit(diag::alerts::Severity::Warn, "wifi", "disconnected");
    }
  } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
    g_connected  = true;
    g_retry      = 0;
    g_connect_us = esp_timer_get_time();
    g_mode       = net::wifi::Mode::StaConnected;
    ip_event_got_ip_t* ev = (ip_event_got_ip_t*)data;
    ESP_LOGI(TAG, "STA connected — IP " IPSTR, IP2STR(&ev->ip_info.ip));
    char ssid[33] = {};
    wifi_config_t wc = {};
    if (esp_wifi_get_config(WIFI_IF_STA, &wc) == ESP_OK) {
      snprintf(ssid, sizeof(ssid), "%s", (char*)wc.sta.ssid);
    }
    char ip_str[16] = {};
    snprintf(ip_str, sizeof(ip_str), IPSTR, IP2STR(&ev->ip_info.ip));
    diag::alerts::emit(diag::alerts::Severity::Info, "wifi",
                       "connected to %s, IP=%s", ssid, ip_str);
    app::self_test::mark_passed(app::self_test::WIFI_CONNECTED);
    if (g_events) xEventGroupSetBits(g_events, BIT_CONNECTED);
  } else if (base == WIFI_EVENT && id == WIFI_EVENT_AP_STACONNECTED) {
    auto* ev = (wifi_event_ap_staconnected_t*)data;
    ESP_LOGI(TAG, "AP: client " MACSTR " joined (AID=%d)",
             MAC2STR(ev->mac), ev->aid);
  } else if (base == WIFI_EVENT && id == WIFI_EVENT_SCAN_DONE) {
    if (g_events) xEventGroupSetBits(g_events, BIT_SCAN_DONE);
  }
}

// ── Init ──────────────────────────────────────────────────────────────────────

namespace net::wifi {

bool init() {
  esp_err_t ret = esp_netif_init();
  if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
    ESP_LOGE(TAG, "esp_netif_init: %s", esp_err_to_name(ret));
    return false;
  }
  ret = esp_event_loop_create_default();
  if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
    ESP_LOGE(TAG, "esp_event_loop_create_default: %s", esp_err_to_name(ret));
    return false;
  }

  g_events = xEventGroupCreate();

  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  ret = esp_wifi_init(&cfg);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "esp_wifi_init: %s", esp_err_to_name(ret));
    return false;
  }

  esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                      on_wifi_event, NULL, NULL);
  esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP,
                                      on_wifi_event, NULL, NULL);
  return true;
}

// ── STA ───────────────────────────────────────────────────────────────────────

bool start_sta(uint32_t timeout_ms) {
  if (!g_events) return false;
  xEventGroupClearBits(g_events, BIT_CONNECTED | BIT_FAILED);
  g_retry = 0;

  if (!g_sta_netif) {
    g_sta_netif = esp_netif_create_default_wifi_sta();
    uint8_t mac[6] = {};
    esp_wifi_get_mac(WIFI_IF_STA, mac);
    snprintf(g_hostname, sizeof(g_hostname), "topband-bms-%02x%02x", mac[4], mac[5]);
    esp_netif_set_hostname(g_sta_netif, g_hostname);
  }

  wifi_config_t sta_cfg = {};
  esp_wifi_get_config(WIFI_IF_STA, &sta_cfg);
  if (sta_cfg.sta.ssid[0] == '\0') {
    ESP_LOGW(TAG, "start_sta: no credentials stored");
    return false;
  }

  ESP_LOGI(TAG, "STA connecting to \"%s\"…", (char*)sta_cfg.sta.ssid);
  g_mode = Mode::StaConnecting;
  esp_wifi_set_mode(WIFI_MODE_STA);
  esp_wifi_set_config(WIFI_IF_STA, &sta_cfg);
  esp_wifi_start();

  EventBits_t bits = xEventGroupWaitBits(
      g_events, BIT_CONNECTED | BIT_FAILED,
      pdFALSE, pdFALSE, pdMS_TO_TICKS(timeout_ms));

  if (bits & BIT_CONNECTED) return true;

  g_mode = Mode::StaFailed;
  ESP_LOGW(TAG, "start_sta: timed out or failed");
  esp_wifi_stop();
  return false;
}

// ── AP ────────────────────────────────────────────────────────────────────────

std::string start_ap() {
  g_mode      = Mode::ApActive;
  g_connected = false;

  // Build SSID "TopBand-Setup-XXXX" from last 2 bytes of AP MAC.
  uint8_t mac[6] = {};
  esp_wifi_get_mac(WIFI_IF_AP, mac);
  char ssid[32] = {};
  snprintf(ssid, sizeof(ssid), "TopBand-Setup-%02X%02X", mac[4], mac[5]);

  if (!g_sta_netif) {
    g_sta_netif = esp_netif_create_default_wifi_sta();
    uint8_t mac[6] = {};
    esp_wifi_get_mac(WIFI_IF_STA, mac);
    snprintf(g_hostname, sizeof(g_hostname), "topband-bms-%02x%02x", mac[4], mac[5]);
    esp_netif_set_hostname(g_sta_netif, g_hostname);
  }
  if (!g_ap_netif)  g_ap_netif  = esp_netif_create_default_wifi_ap();

  wifi_config_t ap_cfg = {};
  memcpy(ap_cfg.ap.ssid, ssid, strlen(ssid));
  ap_cfg.ap.ssid_len       = (uint8_t)strlen(ssid);
  ap_cfg.ap.channel        = 1;
  ap_cfg.ap.authmode       = WIFI_AUTH_OPEN;
  ap_cfg.ap.max_connection = 4;

  // APSTA mode: STA interface is alive but idle, enabling channel scanning.
  esp_wifi_set_mode(WIFI_MODE_APSTA);
  esp_wifi_set_config(WIFI_IF_AP, &ap_cfg);
  esp_wifi_start();

  ESP_LOGI(TAG, "AP started — SSID=%s, IP=192.168.4.1", ssid);
  return std::string(ssid);
}

void stop_ap() {
  // Switch from APSTA → STA to shut the AP interface down.
  esp_wifi_set_mode(WIFI_MODE_STA);
  ESP_LOGI(TAG, "AP stopped");
}

// ── Credentials ───────────────────────────────────────────────────────────────

bool save_creds(const char* ssid, const char* pass) {
  wifi_config_t cfg = {};
  snprintf((char*)cfg.sta.ssid,     sizeof(cfg.sta.ssid),     "%s", ssid);
  snprintf((char*)cfg.sta.password, sizeof(cfg.sta.password), "%s", pass);
  // Allow WPA2/WPA3 and open networks.
  cfg.sta.threshold.authmode = pass[0] ? WIFI_AUTH_WPA_PSK : WIFI_AUTH_OPEN;
  cfg.sta.pmf_cfg.capable    = true;

  esp_err_t ret = esp_wifi_set_config(WIFI_IF_STA, &cfg);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "save_creds: %s", esp_err_to_name(ret));
    return false;
  }
  ESP_LOGI(TAG, "WiFi credentials saved — SSID=\"%s\"", ssid);
  return true;
}

bool load_ssid(char* buf, size_t len) {
  wifi_config_t cfg = {};
  if (esp_wifi_get_config(WIFI_IF_STA, &cfg) != ESP_OK) return false;
  if (cfg.sta.ssid[0] == '\0') return false;
  snprintf(buf, len, "%s", (char*)cfg.sta.ssid);
  return true;
}

// ── Async connect (captive portal) ────────────────────────────────────────────

struct ConnectArgs {
  char     ssid[33];
  char     pass[65];
  uint32_t timeout_ms;
};

static void connect_task(void* arg) {
  ConnectArgs* a = (ConnectArgs*)arg;

  ESP_LOGI(TAG, "connect_task: attempting \"%s\"", a->ssid);
  g_mode = Mode::StaConnecting;

  xEventGroupClearBits(g_events, BIT_CONNECTED | BIT_FAILED);
  g_retry = 0;

  wifi_config_t cfg = {};
  // wifi_config_t.sta.ssid is uint8_t[32] and .password is uint8_t[64];
  // use bounded memcpy to stay inside each buffer without triggering
  // -Werror=format-truncation from snprintf size mismatch.
  {
    size_t n = strlen(a->ssid);
    if (n >= sizeof(cfg.sta.ssid)) n = sizeof(cfg.sta.ssid) - 1;
    memcpy(cfg.sta.ssid, a->ssid, n);
  }
  {
    size_t n = strlen(a->pass);
    if (n >= sizeof(cfg.sta.password)) n = sizeof(cfg.sta.password) - 1;
    memcpy(cfg.sta.password, a->pass, n);
  }
  cfg.sta.threshold.authmode = a->pass[0] ? WIFI_AUTH_WPA_PSK : WIFI_AUTH_OPEN;
  cfg.sta.pmf_cfg.capable    = true;
  esp_wifi_set_config(WIFI_IF_STA, &cfg);

  // In APSTA mode the STA interface is already started; just connect.
  esp_wifi_connect();

  EventBits_t bits = xEventGroupWaitBits(
      g_events, BIT_CONNECTED | BIT_FAILED,
      pdFALSE, pdFALSE, pdMS_TO_TICKS(a->timeout_ms));

  delete a;

  if (bits & BIT_CONNECTED) {
    ESP_LOGI(TAG, "connect_task: connected — rebooting in 3 s");
    vTaskDelay(pdMS_TO_TICKS(3000));
    esp_restart();
  } else {
    g_mode = Mode::StaFailed;
    ESP_LOGW(TAG, "connect_task: failed — returning to AP");
    esp_wifi_disconnect();
  }
  vTaskDelete(nullptr);
}

void start_connection_async(const char* ssid, const char* pass,
                            uint32_t timeout_ms) {
  ConnectArgs* a = new ConnectArgs{};
  snprintf(a->ssid, sizeof(a->ssid), "%s", ssid);
  snprintf(a->pass, sizeof(a->pass), "%s", pass);
  a->timeout_ms = timeout_ms;

  BaseType_t ret = xTaskCreate(connect_task, "wifi_conn", 4096,
                               a, 4, nullptr);
  if (ret != pdPASS) {
    ESP_LOGE(TAG, "start_connection_async: xTaskCreate failed");
    delete a;
    g_mode = Mode::StaFailed;
  }
}

// ── Scan ──────────────────────────────────────────────────────────────────────

std::vector<ScanResult> scan(uint32_t timeout_ms) {
  xEventGroupClearBits(g_events, BIT_SCAN_DONE);

  wifi_scan_config_t cfg = {};
  cfg.scan_type              = WIFI_SCAN_TYPE_ACTIVE;
  cfg.scan_time.active.min   = 100;
  cfg.scan_time.active.max   = 300;

  esp_err_t err = esp_wifi_scan_start(&cfg, false);  // non-blocking
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "scan_start: %s", esp_err_to_name(err));
    return {};
  }

  EventBits_t bits = xEventGroupWaitBits(
      g_events, BIT_SCAN_DONE, pdTRUE, pdFALSE, pdMS_TO_TICKS(timeout_ms));
  if (!(bits & BIT_SCAN_DONE)) {
    ESP_LOGW(TAG, "scan: timed out");
    esp_wifi_scan_stop();
    return {};
  }

  uint16_t count = 0;
  esp_wifi_scan_get_ap_num(&count);
  if (count == 0) return {};

  // Cap at 20 results to limit stack/heap usage.
  if (count > 20) count = 20;
  wifi_ap_record_t records[20] = {};
  esp_wifi_scan_get_ap_records(&count, records);

  std::vector<ScanResult> out;
  out.reserve(count);
  for (uint16_t i = 0; i < count; i++) {
    // Skip entries with empty SSID (hidden networks).
    if (records[i].ssid[0] == '\0') continue;
    ScanResult r;
    r.ssid   = std::string((char*)records[i].ssid);
    r.rssi   = records[i].rssi;
    r.secure = (records[i].authmode != WIFI_AUTH_OPEN);
    out.push_back(std::move(r));
  }
  // Sort by signal strength, strongest first.
  std::sort(out.begin(), out.end(),
            [](const ScanResult& a, const ScanResult& b){ return a.rssi > b.rssi; });
  return out;
}

// ── Status ────────────────────────────────────────────────────────────────────

Mode get_state()     { return g_mode; }
bool is_connected()  { return g_connected; }
bool is_ap_mode()    { return g_mode == Mode::ApActive; }

void get_ip(char* buf, size_t len) {
  if (!g_connected || !g_sta_netif) {
    snprintf(buf, len, "0.0.0.0");
    return;
  }
  esp_netif_ip_info_t info = {};
  esp_netif_get_ip_info(g_sta_netif, &info);
  snprintf(buf, len, IPSTR, IP2STR(&info.ip));
}

std::string get_local_ip() {
  char buf[24] = {};
  get_ip(buf, sizeof(buf));
  return std::string(buf);
}

esp_netif_t* get_ap_netif() { return g_ap_netif; }

std::string get_ssid() {
  if (!g_connected) return {};
  wifi_ap_record_t info = {};
  if (esp_wifi_sta_get_ap_info(&info) != ESP_OK) return {};
  return std::string((char*)info.ssid);
}

int8_t get_rssi() {
  if (!g_connected) return 0;
  wifi_ap_record_t info = {};
  if (esp_wifi_sta_get_ap_info(&info) != ESP_OK) return 0;
  return info.rssi;
}

void get_hostname(char* buf, size_t len) {
  snprintf(buf, len, "%s", g_hostname);
}

IpInfo get_ip_info() {
  IpInfo out = {};
  snprintf(out.ip,      sizeof(out.ip),      "0.0.0.0");
  snprintf(out.gw,      sizeof(out.gw),      "0.0.0.0");
  snprintf(out.netmask, sizeof(out.netmask), "0.0.0.0");
  snprintf(out.dns,     sizeof(out.dns),     "0.0.0.0");

  if (!g_connected || !g_sta_netif) return out;

  esp_netif_ip_info_t ip_info = {};
  if (esp_netif_get_ip_info(g_sta_netif, &ip_info) == ESP_OK) {
    snprintf(out.ip,      sizeof(out.ip),      IPSTR, IP2STR(&ip_info.ip));
    snprintf(out.gw,      sizeof(out.gw),      IPSTR, IP2STR(&ip_info.gw));
    snprintf(out.netmask, sizeof(out.netmask), IPSTR, IP2STR(&ip_info.netmask));
  }

  esp_netif_dns_info_t dns_info = {};
  if (esp_netif_get_dns_info(g_sta_netif, ESP_NETIF_DNS_MAIN, &dns_info) == ESP_OK) {
    if (dns_info.ip.type == ESP_IPADDR_TYPE_V4) {
      snprintf(out.dns, sizeof(out.dns), IPSTR, IP2STR(&dns_info.ip.u_addr.ip4));
    }
  }

  return out;
}

uint32_t connected_for_s() {
  if (!g_connected || g_connect_us == 0) return 0;
  int64_t now = esp_timer_get_time();
  int64_t diff = (now - g_connect_us) / 1000000LL;
  return (diff > 0) ? (uint32_t)diff : 0;
}

}  // namespace net::wifi
