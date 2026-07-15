#pragma once
#include <cstddef>
#include <cstdint>

// Minimal host stand-in for the ESP-IDF NVS API surface used by
// src/storage/nvs_store.cpp. Backed by an in-memory fake store
// (fake_nvs.cpp) so loadConfig()/saveConfig() can be exercised on host —
// including the exact "stored blob larger than the read buffer" scenario
// that caused the V3.2 config-migration data-loss regression (a stale v10
// blob is 884 B; the buffer used to be sized off the running firmware's
// sizeof(Config), which is 880 B for v11). See test_nvs_store.cpp.
//
// Only the handful of calls nvs_store.cpp actually makes are modeled; this
// is not a general NVS emulation.

using esp_err_t = int;
constexpr esp_err_t ESP_OK                     = 0;
constexpr esp_err_t ESP_FAIL                   = -1;
constexpr esp_err_t ESP_ERR_NVS_NOT_FOUND      = 0x1102;
constexpr esp_err_t ESP_ERR_NVS_INVALID_LENGTH = 0x1104;
constexpr esp_err_t ESP_ERR_INVALID_STATE      = 0x103;

const char* esp_err_to_name(esp_err_t err);

using nvs_handle_t = uint32_t;

enum nvs_open_mode_t { NVS_READONLY = 0, NVS_READWRITE = 1 };

esp_err_t nvs_open(const char* name, nvs_open_mode_t open_mode, nvs_handle_t* out_handle);
void      nvs_close(nvs_handle_t handle);

// Mirrors real nvs_get_blob() length-probing semantics: if out_value is
// nullptr, *length is set to the stored size and ESP_OK is returned (when
// found). If out_value is non-null but *length is smaller than the stored
// size, returns ESP_ERR_NVS_INVALID_LENGTH and sets *length to the required
// size WITHOUT copying any data — this is the real-hardware behavior the
// V3.2 regression's undersized fixed buffer collided with.
esp_err_t nvs_get_blob(nvs_handle_t handle, const char* key, void* out_value, size_t* length);
esp_err_t nvs_get_u32(nvs_handle_t handle, const char* key, uint32_t* out_value);
esp_err_t nvs_set_blob(nvs_handle_t handle, const char* key, const void* value, size_t length);
esp_err_t nvs_set_u32(nvs_handle_t handle, const char* key, uint32_t value);
esp_err_t nvs_commit(nvs_handle_t handle);
