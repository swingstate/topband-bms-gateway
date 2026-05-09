#include "wifi.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "esp_mac.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include <cstring>
#include <cstdio>

static const char* TAG = "wifi";

static EventGroupHandle_t g_wifi_events = nullptr;
static constexpr EventBits_t CONNECTED_BIT = BIT0;
static constexpr EventBits_t FAILED_BIT    = BIT1;

static bool g_ap_mode  = false;
static bool g_connected = false;
static int  g_retry_count = 0;
static constexpr int MAX_RETRY = 5;

static esp_netif_t* g_sta_netif = nullptr;
static esp_netif_t* g_ap_netif  = nullptr;

// ── Event handler ─────────────────────────────────────────────────────────────

static void on_wifi_event(void* arg, esp_event_base_t event_base,
                          int32_t event_id, void* event_data) {
  if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
    esp_wifi_connect();
  } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
    g_connected = false;
    if (g_retry_count < MAX_RETRY) {
      g_retry_count++;
      ESP_LOGW(TAG, "STA disconnected — retry %d/%d", g_retry_count, MAX_RETRY);
      esp_wifi_connect();
    } else {
      ESP_LOGE(TAG, "STA connect failed after %d retries", MAX_RETRY);
      if (g_wifi_events) xEventGroupSetBits(g_wifi_events, FAILED_BIT);
    }
  } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
    g_connected   = true;
    g_retry_count = 0;
    ip_event_got_ip_t* event = (ip_event_got_ip_t*)event_data;
    ESP_LOGI(TAG, "STA connected — IP " IPSTR, IP2STR(&event->ip_info.ip));
    if (g_wifi_events) xEventGroupSetBits(g_wifi_events, CONNECTED_BIT);
  } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_AP_STACONNECTED) {
    wifi_event_ap_staconnected_t* e = (wifi_event_ap_staconnected_t*)event_data;
    ESP_LOGI(TAG, "AP: station " MACSTR " joined, AID=%d",
             MAC2STR(e->mac), e->aid);
  }
}

// ── Public API ────────────────────────────────────────────────────────────────

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

  g_wifi_events = xEventGroupCreate();

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

bool start_sta(uint32_t timeout_ms) {
  if (!g_wifi_events) return false;
  xEventGroupClearBits(g_wifi_events, CONNECTED_BIT | FAILED_BIT);
  g_retry_count = 0;
  g_ap_mode     = false;

  if (!g_sta_netif) {
    g_sta_netif = esp_netif_create_default_wifi_sta();
  }

  // Read stored credentials.
  wifi_config_t sta_cfg = {};
  esp_wifi_get_config(WIFI_IF_STA, &sta_cfg);
  if (sta_cfg.sta.ssid[0] == '\0') {
    ESP_LOGW(TAG, "No WiFi credentials stored — cannot start STA");
    return false;
  }

  ESP_LOGI(TAG, "STA connecting to \"%s\"…", (char*)sta_cfg.sta.ssid);
  esp_wifi_set_mode(WIFI_MODE_STA);
  esp_wifi_set_config(WIFI_IF_STA, &sta_cfg);
  esp_wifi_start();

  EventBits_t bits = xEventGroupWaitBits(
      g_wifi_events, CONNECTED_BIT | FAILED_BIT,
      pdFALSE, pdFALSE, pdMS_TO_TICKS(timeout_ms));

  if (bits & CONNECTED_BIT) return true;

  ESP_LOGW(TAG, "STA connection failed — starting AP fallback");
  esp_wifi_stop();
  return false;
}

void start_ap() {
  g_ap_mode   = true;
  g_connected = false;

  // Build SSID "TopBand-Setup-XXXX" using last 2 bytes of MAC.
  uint8_t mac[6] = {};
  esp_wifi_get_mac(WIFI_IF_AP, mac);
  char ssid[32];
  snprintf(ssid, sizeof(ssid), "TopBand-Setup-%02X%02X", mac[4], mac[5]);

  if (!g_ap_netif) {
    g_ap_netif = esp_netif_create_default_wifi_ap();
  }

  wifi_config_t ap_cfg = {};
  memcpy(ap_cfg.ap.ssid, ssid, strlen(ssid));
  ap_cfg.ap.ssid_len       = (uint8_t)strlen(ssid);
  ap_cfg.ap.channel        = 1;
  ap_cfg.ap.authmode       = WIFI_AUTH_OPEN;
  ap_cfg.ap.max_connection = 4;

  esp_wifi_set_mode(WIFI_MODE_AP);
  esp_wifi_set_config(WIFI_IF_AP, &ap_cfg);
  esp_wifi_start();

  ESP_LOGI(TAG, "AP started: SSID=%s, IP=192.168.4.1", ssid);
}

bool save_creds(const char* ssid, const char* pass) {
  wifi_config_t cfg = {};
  snprintf((char*)cfg.sta.ssid,     sizeof(cfg.sta.ssid),     "%s", ssid);
  snprintf((char*)cfg.sta.password, sizeof(cfg.sta.password), "%s", pass);
  cfg.sta.threshold.authmode = WIFI_AUTH_WPA_PSK;

  esp_err_t ret = esp_wifi_set_config(WIFI_IF_STA, &cfg);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "save_creds: esp_wifi_set_config: %s", esp_err_to_name(ret));
    return false;
  }
  ESP_LOGI(TAG, "WiFi credentials saved for SSID \"%s\"", ssid);
  return true;
}

bool load_ssid(char* buf, size_t len) {
  wifi_config_t cfg = {};
  if (esp_wifi_get_config(WIFI_IF_STA, &cfg) != ESP_OK) return false;
  if (cfg.sta.ssid[0] == '\0') return false;
  snprintf(buf, len, "%s", (char*)cfg.sta.ssid);
  return true;
}

bool is_connected() {
  return g_connected;
}

void get_ip(char* buf, size_t len) {
  if (!g_connected || !g_sta_netif) {
    snprintf(buf, len, "0.0.0.0");
    return;
  }
  esp_netif_ip_info_t info = {};
  esp_netif_get_ip_info(g_sta_netif, &info);
  snprintf(buf, len, IPSTR, IP2STR(&info.ip));
}

bool is_ap_mode() {
  return g_ap_mode;
}

}  // namespace net::wifi
