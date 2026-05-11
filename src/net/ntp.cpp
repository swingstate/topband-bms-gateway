#include "ntp.h"
#include "storage/config.h"
#include "esp_sntp.h"
#include "esp_log.h"
#include <cstring>
#include <cstdio>
#include <ctime>

static const char* TAG = "ntp";

static volatile bool s_synced = false;

static void sntp_sync_callback(struct timeval* /*tv*/) {
  s_synced = true;
  time_t now = time(nullptr);
  struct tm local_tm = {};
  localtime_r(&now, &local_tm);
  char buf[32];
  strftime(buf, sizeof(buf), "%Y-%m-%d %H:%M:%S", &local_tm);
  ESP_LOGI(TAG, "sync complete, time=%s (local)", buf);
}

// Build POSIX TZ string from a simple signed-hour offset.
// e.g. offset +1 → "UTC-1" (POSIX inverts the sign).
static void apply_tz(int8_t offset_h) {
  char tz_str[16];
  if (offset_h == 0) {
    snprintf(tz_str, sizeof(tz_str), "UTC0");
  } else if (offset_h > 0) {
    snprintf(tz_str, sizeof(tz_str), "UTC-%d", (int)offset_h);
  } else {
    snprintf(tz_str, sizeof(tz_str), "UTC+%d", (int)(-offset_h));
  }
  setenv("TZ", tz_str, 1);
  tzset();
  ESP_LOGI(TAG, "timezone set: TZ=%s (offset %+d h)", tz_str, (int)offset_h);
}

namespace net::ntp {

bool start(const Config& cfg) {
  apply_tz(cfg.timezone_offset_h);

  esp_sntp_setoperatingmode(ESP_SNTP_OPMODE_POLL);
  const char* srv = (cfg.ntp_server[0] != '\0') ? cfg.ntp_server : "pool.ntp.org";
  esp_sntp_setservername(0, srv);
  sntp_set_time_sync_notification_cb(sntp_sync_callback);
  esp_sntp_set_sync_mode(SNTP_SYNC_MODE_IMMED);
  esp_sntp_init();
  ESP_LOGI(TAG, "SNTP started, server=%s", srv);
  return true;
}

void stop() {
  esp_sntp_stop();
  s_synced = false;
}

bool is_synced() {
  return s_synced;
}

uint32_t now_unix_s() {
  if (!s_synced) return 0;
  return (uint32_t)time(nullptr);
}

void reconfigure(const Config& cfg) {
  esp_sntp_stop();
  s_synced = false;
  start(cfg);
}

}  // namespace net::ntp
