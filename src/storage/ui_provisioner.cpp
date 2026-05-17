#include "ui_provisioner.h"
#include "lfs_store.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_heap_caps.h"
#include "../third_party/microtar/microtar.h"
#include <cstring>
#include <cerrno>

// Symbols injected by CMake EMBED_TXTFILES for web/littlefs_ui.tar
extern const uint8_t _binary_littlefs_ui_tar_start[];
extern const uint8_t _binary_littlefs_ui_tar_end[];

static const char* TAG = "ui_prov";

// Path where the version string is stored so we can detect staleness.
static constexpr const char* VERSION_FILE = "/lfs/meta/ui_version.txt";

namespace storage::ui_provisioner {

// Extract in-memory tar into LittleFS. Returns number of files extracted.
static int extract_tar(const uint8_t* tar_data, size_t tar_len) {
  mtar_t tar;
  if (mtar_open_mem(&tar, tar_data, tar_len) != MTAR_ESUCCESS) {
    ESP_LOGE(TAG, "mtar_open_mem failed");
    return 0;
  }

  mtar_header_t h;
  int file_count = 0;
  size_t total_bytes = 0;

  while (mtar_read_header(&tar, &h) == MTAR_ESUCCESS) {
    if (h.type == MTAR_TDIR) {
      char dir_path[256];
      snprintf(dir_path, sizeof(dir_path), "/lfs/ui/%s", h.name);
      size_t len = strlen(dir_path);
      if (len > 0 && dir_path[len - 1] == '/') dir_path[len - 1] = '\0';
      storage::lfs::mkdir_p(dir_path);
    } else if (h.type == MTAR_TREG && h.size > 0) {
      // Prefer PSRAM for large files — DRAM is tight on this firmware.
      uint8_t* file_buf = (uint8_t*)heap_caps_malloc(
          h.size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
      if (!file_buf) file_buf = (uint8_t*)malloc(h.size);  // DRAM fallback
      if (!file_buf) {
        ESP_LOGE(TAG, "OOM reading %s (%u B)", h.name, h.size);
      } else {
        if (mtar_read_data(&tar, file_buf, h.size) == MTAR_ESUCCESS) {
          char dest[256];
          snprintf(dest, sizeof(dest), "/lfs/ui/%s", h.name);

          // Create parent directory if needed.
          char parent[256];
          snprintf(parent, sizeof(parent), "%s", dest);
          char* slash = strrchr(parent, '/');
          if (slash && slash != parent) {
            *slash = '\0';
            storage::lfs::mkdir_p(parent);
          }

          if (storage::lfs::write_file_atomic(dest, file_buf, h.size)) {
            file_count++;
            total_bytes += h.size;
            ESP_LOGD(TAG, "  extracted %s (%u B)", dest, h.size);
          } else {
            ESP_LOGE(TAG, "  write failed: %s", dest);
          }
        }
        free(file_buf);
      }
    }

    // mtar_next() re-reads the current entry header from the post-read position,
    // which fails for non-zero-size entries (it reads data as a header). Seek
    // directly to the next 512-byte-aligned entry using the already-parsed size.
    unsigned next = tar.last_header + 512u + ((h.size + 511u) & ~511u);
    if (mtar_seek(&tar, next) != MTAR_ESUCCESS) break;
  }

  mtar_close(&tar);
  ESP_LOGI(TAG, "Extracted %d files, %u B total", file_count, (unsigned)total_bytes);
  return file_count;
}

void provision_ui_if_needed() {
  char stored_version[32] = {};
  storage::lfs::read_file(VERSION_FILE, stored_version, sizeof(stored_version));

  if (strcmp(stored_version, UI_VERSION) == 0) {
    ESP_LOGI(TAG, "UI version %s already extracted — skipping", UI_VERSION);
    return;
  }

  const uint8_t* tar_start = _binary_littlefs_ui_tar_start;
  const uint8_t* tar_end   = _binary_littlefs_ui_tar_end;
  size_t tar_len = (size_t)(tar_end - tar_start);

  if (tar_len == 0) {
    ESP_LOGE(TAG, "Embedded UI tarball is empty — UI provisioning skipped");
    return;
  }

  ESP_LOGI(TAG, "Extracting UI bundle %s (tar_len=%u B)…", UI_VERSION, (unsigned)tar_len);
  int64_t t0 = esp_timer_get_time();

  storage::lfs::mkdir_p("/lfs/ui");
  int n = extract_tar(tar_start, tar_len);

  if (n > 0) {
    storage::lfs::write_file_atomic(
        VERSION_FILE, (const uint8_t*)UI_VERSION, strlen(UI_VERSION));

    int64_t elapsed_ms = (esp_timer_get_time() - t0) / 1000;
    ESP_LOGI(TAG, "UI provisioning done: %d files in %lld ms", n, elapsed_ms);
  } else {
    ESP_LOGE(TAG, "UI provisioning extracted 0 files — check tarball");
  }
}

}  // namespace storage::ui_provisioner
