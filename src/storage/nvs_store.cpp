#include "nvs_store.h"
#include "config.h"
#include "esp_log.h"
#include "nvs.h"
#include "nvs_flash.h"

static const char* TAG       = "nvs_store";
static const char* NVS_NS    = "gateway";
static const char* KEY_BLOB  = "cfg_v1";
static const char* KEY_CRC   = "cfg_v1_crc";

// ── RAII handle ───────────────────────────────────────────────────────────────
namespace {

class NvsHandle {
public:
  NvsHandle(const char* ns, nvs_open_mode_t mode)
    : err_(nvs_open(ns, mode, &h_)) {}
  ~NvsHandle() { if (err_ == ESP_OK) nvs_close(h_); }

  bool ok()             const { return err_ == ESP_OK; }
  nvs_handle_t get()    const { return h_; }
  esp_err_t    error()  const { return err_; }

  NvsHandle(const NvsHandle&)            = delete;
  NvsHandle& operator=(const NvsHandle&) = delete;

private:
  nvs_handle_t h_   = 0;
  esp_err_t    err_ = ESP_ERR_INVALID_STATE;
};

}  // namespace

// ── loadConfig ────────────────────────────────────────────────────────────────

namespace storage {

bool loadConfig(Config& out) {
  NvsHandle h(NVS_NS, NVS_READONLY);
  if (!h.ok()) {
    ESP_LOGW(TAG, "NVS open failed (%s) — using defaults", esp_err_to_name(h.error()));
    out = DEFAULT_CONFIG;
    return false;
  }

  // Read blob
  uint8_t buf[sizeof(Config)];
  size_t len = sizeof(buf);
  esp_err_t err = nvs_get_blob(h.get(), KEY_BLOB, buf, &len);
  if (err != ESP_OK) {
    ESP_LOGW(TAG, "cfg_v1 not found (%s) — using defaults", esp_err_to_name(err));
    out = DEFAULT_CONFIG;
    return false;
  }

  // Read stored CRC
  uint32_t stored_crc = 0;
  err = nvs_get_u32(h.get(), KEY_CRC, &stored_crc);
  if (err != ESP_OK) {
    ESP_LOGW(TAG, "cfg_v1_crc not found (%s) — using defaults", esp_err_to_name(err));
    out = DEFAULT_CONFIG;
    return false;
  }

  // Verify CRC
  uint32_t computed = storage::crc32(buf, len);
  if (computed != stored_crc) {
    ESP_LOGE(TAG, "NVS CRC mismatch: stored=0x%08" PRIx32 " computed=0x%08" PRIx32
             " — using defaults", stored_crc, computed);
    out = DEFAULT_CONFIG;
    return false;
  }

  // Deserialize (checks schema_version)
  if (!storage::deserialize(buf, len, out)) {
    ESP_LOGE(TAG, "Deserialize failed (schema version mismatch?) — using defaults");
    out = DEFAULT_CONFIG;
    return false;
  }

  return true;
}

// ── saveConfig ────────────────────────────────────────────────────────────────

bool saveConfig(const Config& cfg) {
  // 1. Serialize
  uint8_t buf[sizeof(Config)];
  size_t len = 0;
  if (!storage::serialize(cfg, buf, sizeof(buf), len)) {
    ESP_LOGE(TAG, "Serialize failed");
    return false;
  }

  // 2. Compute CRC
  uint32_t crc = storage::crc32(buf, len);

  // 3. Write blob + CRC + commit
  {
    NvsHandle h(NVS_NS, NVS_READWRITE);
    if (!h.ok()) {
      ESP_LOGE(TAG, "NVS open failed for write (%s)", esp_err_to_name(h.error()));
      return false;
    }
    if (nvs_set_blob(h.get(), KEY_BLOB, buf, len) != ESP_OK) {
      ESP_LOGE(TAG, "nvs_set_blob failed");
      return false;
    }
    if (nvs_set_u32(h.get(), KEY_CRC, crc) != ESP_OK) {
      ESP_LOGE(TAG, "nvs_set_u32 (crc) failed");
      return false;
    }
    if (nvs_commit(h.get()) != ESP_OK) {
      ESP_LOGE(TAG, "nvs_commit failed");
      return false;
    }
  }

  // 4. Read back and re-verify CRC.
  // On mismatch, leave the NVS entry as-is (it may still be valid from before)
  // and return false so the caller knows the write cannot be confirmed.
  {
    NvsHandle h(NVS_NS, NVS_READONLY);
    if (!h.ok()) {
      ESP_LOGE(TAG, "NVS readback open failed");
      return false;
    }
    uint8_t rbuf[sizeof(Config)];
    size_t rlen = sizeof(rbuf);
    if (nvs_get_blob(h.get(), KEY_BLOB, rbuf, &rlen) != ESP_OK) {
      ESP_LOGE(TAG, "NVS readback blob failed");
      return false;
    }
    uint32_t rcrc = 0;
    if (nvs_get_u32(h.get(), KEY_CRC, &rcrc) != ESP_OK) {
      ESP_LOGE(TAG, "NVS readback CRC key failed");
      return false;
    }
    uint32_t rcomputed = storage::crc32(rbuf, rlen);
    if (rcomputed != rcrc) {
      // Keep old cfg_v1 as-is per architecture §8.6 transactional rule.
      ESP_LOGE(TAG, "NVS write-readback CRC mismatch — old config preserved");
      return false;
    }
  }

  ESP_LOGI(TAG, "Config saved to NVS (%u bytes, CRC 0x%08" PRIx32 ")",
           (unsigned)len, crc);
  return true;
}

}  // namespace storage
