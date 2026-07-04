#pragma once
#include <cstddef>
#include <cstdint>

// Test-only control surface for the fake in-memory NVS backing store behind
// nvs.h/fake_nvs.cpp. Not part of the real ESP-IDF API — only used by
// test_nvs_store.cpp to seed/reset state between cases.

void fake_nvs_reset();
void fake_nvs_seed_blob(const char* key, const void* data, size_t len);
void fake_nvs_seed_u32(const char* key, uint32_t value);
