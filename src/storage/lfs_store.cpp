#include "lfs_store.h"
#include "esp_littlefs.h"
#include "esp_log.h"
#include <cstring>
#include <cerrno>
#include <sys/stat.h>
#include <dirent.h>

static const char* TAG = "lfs";

// LittleFS is registered under this VFS path prefix.
static constexpr const char* LFS_BASE = "/lfs";

static bool g_mounted = false;

namespace storage::lfs {

bool init() {
  esp_vfs_littlefs_conf_t conf = {
    .base_path              = LFS_BASE,
    .partition_label        = "littlefs",
    .partition              = NULL,
    .format_if_mount_failed = true,
    .read_only              = false,
    .dont_mount             = false,
    .grow_on_mount          = false,
  };

  esp_err_t ret = esp_vfs_littlefs_register(&conf);
  if (ret == ESP_OK) {
    g_mounted = true;
    size_t total = 0, used = 0;
    esp_littlefs_info("littlefs", &total, &used);
    ESP_LOGI(TAG, "LittleFS mounted: total=%u used=%u free=%u B",
             (unsigned)total, (unsigned)used, (unsigned)(total - used));
    // Ensure base directories exist.
    mkdir_p("/lfs/ui");
    mkdir_p("/lfs/meta");
    return true;
  }
  if (ret == ESP_ERR_NOT_FOUND) {
    ESP_LOGE(TAG, "LittleFS partition not found in partition table");
  } else {
    ESP_LOGE(TAG, "LittleFS mount/format failed: %s — running in degraded mode",
             esp_err_to_name(ret));
  }
  return false;
}

bool exists(const char* path) {
  struct stat st;
  return (stat(path, &st) == 0);
}

size_t read_file(const char* path, char* buf, size_t buf_size) {
  if (buf_size == 0) return 0;
  FILE* f = fopen(path, "rb");
  if (!f) return 0;
  size_t n = fread(buf, 1, buf_size - 1, f);
  fclose(f);
  buf[n] = '\0';
  return n;
}

bool write_file_atomic(const char* path, const uint8_t* data, size_t len) {
  // Build tmp path: <path>.tmp
  char tmp[256];
  snprintf(tmp, sizeof(tmp), "%s.tmp", path);

  FILE* f = fopen(tmp, "wb");
  if (!f) {
    ESP_LOGE(TAG, "write_file_atomic: fopen(%s) failed: %d", tmp, errno);
    return false;
  }
  size_t written = fwrite(data, 1, len, f);
  fclose(f);

  if (written != len) {
    ESP_LOGE(TAG, "write_file_atomic: wrote %u of %u bytes to %s",
             (unsigned)written, (unsigned)len, tmp);
    remove(tmp);
    return false;
  }

  // Remove existing target first: esp-littlefs rename-over-existing triggers
  // an internal lfs_remove, which can overflow a shallow stack. By removing
  // explicitly the rename becomes a simple directory-entry move.
  remove(path);  // expected ENOENT on first extraction — ignore

  if (rename(tmp, path) != 0) {
    ESP_LOGE(TAG, "write_file_atomic: rename(%s → %s) failed: %d", tmp, path, errno);
    remove(tmp);
    return false;
  }
  return true;
}

bool mkdir_p(const char* path) {
  // Walk the path and create each component.
  char buf[256];
  snprintf(buf, sizeof(buf), "%s", path);
  size_t len = strlen(buf);
  if (len > 0 && buf[len - 1] == '/') buf[len - 1] = '\0';

  for (char* p = buf + 1; *p; p++) {
    if (*p == '/') {
      *p = '\0';
      if (mkdir(buf, 0755) != 0 && errno != EEXIST) {
        ESP_LOGE(TAG, "mkdir_p: mkdir(%s) failed: %d", buf, errno);
        return false;
      }
      *p = '/';
    }
  }
  if (mkdir(buf, 0755) != 0 && errno != EEXIST) {
    ESP_LOGE(TAG, "mkdir_p: mkdir(%s) failed: %d", buf, errno);
    return false;
  }
  return true;
}

bool remove_file(const char* path) {
  if (remove(path) == 0) return true;
  if (errno == ENOENT) return false;
  ESP_LOGW(TAG, "remove_file(%s) failed: %d", path, errno);
  return false;
}

size_t total_bytes() {
  if (!g_mounted) return 0;
  size_t total = 0, used = 0;
  esp_littlefs_info("littlefs", &total, &used);
  return total;
}

size_t free_bytes() {
  if (!g_mounted) return 0;
  size_t total = 0, used = 0;
  esp_littlefs_info("littlefs", &total, &used);
  return total > used ? total - used : 0;
}

}  // namespace storage::lfs
