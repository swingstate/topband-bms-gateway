#include "nvs.h"
#include "fake_nvs_test_api.h"

#include <cstring>
#include <map>
#include <string>
#include <vector>

// Fake single-namespace NVS backing store. nvs_store.cpp only ever opens one
// namespace ("gateway") and touches two keys ("cfg_v1", "cfg_v1_crc"), so the
// namespace argument is ignored throughout — good enough for these tests.
namespace {

struct FakeNvs {
  std::map<std::string, std::vector<uint8_t>> blobs;
  std::map<std::string, uint32_t> u32s;
};

FakeNvs& store() {
  static FakeNvs s;
  return s;
}

}  // namespace

const char* esp_err_to_name(esp_err_t err) {
  return err == ESP_OK ? "ESP_OK" : "ESP_FAIL";
}

esp_err_t nvs_open(const char* /*name*/, nvs_open_mode_t /*open_mode*/, nvs_handle_t* out_handle) {
  *out_handle = 1;
  return ESP_OK;
}

void nvs_close(nvs_handle_t /*handle*/) {}

esp_err_t nvs_get_blob(nvs_handle_t /*handle*/, const char* key, void* out_value, size_t* length) {
  auto it = store().blobs.find(key);
  if (it == store().blobs.end()) return ESP_ERR_NVS_NOT_FOUND;

  if (out_value == nullptr) {
    *length = it->second.size();
    return ESP_OK;
  }
  if (*length < it->second.size()) {
    *length = it->second.size();
    return ESP_ERR_NVS_INVALID_LENGTH;
  }
  std::memcpy(out_value, it->second.data(), it->second.size());
  *length = it->second.size();
  return ESP_OK;
}

esp_err_t nvs_get_u32(nvs_handle_t /*handle*/, const char* key, uint32_t* out_value) {
  auto it = store().u32s.find(key);
  if (it == store().u32s.end()) return ESP_ERR_NVS_NOT_FOUND;
  *out_value = it->second;
  return ESP_OK;
}

esp_err_t nvs_set_blob(nvs_handle_t /*handle*/, const char* key, const void* value, size_t length) {
  auto& v = store().blobs[key];
  const auto* p = static_cast<const uint8_t*>(value);
  v.assign(p, p + length);
  return ESP_OK;
}

esp_err_t nvs_set_u32(nvs_handle_t /*handle*/, const char* key, uint32_t value) {
  store().u32s[key] = value;
  return ESP_OK;
}

esp_err_t nvs_commit(nvs_handle_t /*handle*/) { return ESP_OK; }

// ── Test-only seeding API ───────────────────────────────────────────────────

void fake_nvs_reset() {
  store().blobs.clear();
  store().u32s.clear();
}

void fake_nvs_seed_blob(const char* key, const void* data, size_t len) {
  auto& v = store().blobs[key];
  const auto* p = static_cast<const uint8_t*>(data);
  v.assign(p, p + len);
}

void fake_nvs_seed_u32(const char* key, uint32_t value) {
  store().u32s[key] = value;
}
