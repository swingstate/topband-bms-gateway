#include "nvs_store.h"
#include "config.h"
#include "esp_log.h"
#include "nvs.h"
#include "nvs_flash.h"

static const char* TAG       = "nvs_store";
static const char* NVS_NS    = "gateway";
static const char* KEY_BLOB  = "cfg_v1";
static const char* KEY_CRC   = "cfg_v1_crc";

// Upper bound for reading a stored cfg_v1 blob. Schema versions do not only
// grow: v11 is 4 B SMALLER than v10 (a mid-struct field removal closed an
// alignment gap — see config.h's migration history comment), so a blob
// written by older firmware can be larger than sizeof(Config) on the
// currently running firmware. Must stay >= the largest Config_vN ever
// serialized (v10 = 884 B is the largest to date), with headroom for future
// schema growth.
static constexpr size_t MAX_CONFIG_BLOB_SIZE = 1024;
static_assert(MAX_CONFIG_BLOB_SIZE >= sizeof(Config),
    "MAX_CONFIG_BLOB_SIZE must be >= sizeof(Config)");

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

  // Probe the stored blob's actual size first — do not assume it fits in
  // sizeof(Config) (see MAX_CONFIG_BLOB_SIZE comment above). Sizing the read
  // buffer off the CURRENT (running-firmware) struct size silently drops a
  // larger stale blob: nvs_get_blob() rejects the undersized buffer with an
  // error, loadConfig() falls through to "not found", and the entire config
  // — MQTT, BLE/MPPT, everything — resets to DEFAULT_CONFIG on first boot
  // after an upgrade that shrinks the struct.
  size_t len = 0;
  esp_err_t err = nvs_get_blob(h.get(), KEY_BLOB, nullptr, &len);
  if (err != ESP_OK) {
    ESP_LOGW(TAG, "cfg_v1 not found (%s) — using defaults", esp_err_to_name(err));
    out = DEFAULT_CONFIG;
    return false;
  }
  if (len > MAX_CONFIG_BLOB_SIZE) {
    ESP_LOGE(TAG, "cfg_v1 blob too large (%u B, max %u) — using defaults",
             (unsigned)len, (unsigned)MAX_CONFIG_BLOB_SIZE);
    out = DEFAULT_CONFIG;
    return false;
  }

  uint8_t buf[MAX_CONFIG_BLOB_SIZE];
  err = nvs_get_blob(h.get(), KEY_BLOB, buf, &len);
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

  // Peek at schema version before deserialization so we can detect migration.
  uint16_t schema_in_blob = 0;
  if (len >= 2) memcpy(&schema_in_blob, buf, sizeof(schema_in_blob));

  // Deserialize (handles v1→v2 migration inside; unknown versions return false).
  if (!storage::deserialize(buf, len, out)) {
    ESP_LOGE(TAG, "Deserialize failed (unknown schema v%u?) — using defaults",
             (unsigned)schema_in_blob);
    out = DEFAULT_CONFIG;
    return false;
  }

  // If a schema migration ran, persist the upgraded blob immediately so
  // subsequent boots load v2 directly without re-running the migration path.
  if (schema_in_blob != CURRENT_SCHEMA_VERSION) {
    ESP_LOGI(TAG, "Config migrated v%u → v%u — persisting to NVS",
             (unsigned)schema_in_blob, (unsigned)CURRENT_SCHEMA_VERSION);
    if (!saveConfig(out)) {
      // Non-fatal: the device will re-migrate on the next boot (safe and correct).
      ESP_LOGW(TAG, "Migration persist failed — will re-migrate on next boot");
    }
  }

  // Per-pack level insertion migration: old PerCell was value 3, now value 4.
  // Any config saved before this change with mqtt_level==3 meant "Per-cell";
  // upgrade it in-place and persist so subsequent boots need no migration.
  if (static_cast<uint8_t>(out.mqtt_level) == 3) {
    out.mqtt_level = Config::MqttLevel::PerCell;
    ESP_LOGI(TAG, "mqtt_level migrated: 3 (old PerCell) -> 4 (new PerCell)");
    saveConfig(out);  // best-effort; failure is non-critical, migrates again next boot
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
