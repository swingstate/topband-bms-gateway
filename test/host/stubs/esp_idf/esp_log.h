#pragma once
// Minimal host-build stand-in for ESP-IDF's esp_log.h. nvs_store.cpp only
// ever uses these as fire-and-forget diagnostic logging; no-ops are
// sufficient for host tests. <cinttypes> is included because nvs_store.cpp
// uses PRIx32 inside ESP_LOGE format strings — the token still needs to
// expand to something even though the macro discards the whole call.
#include <cinttypes>
#include <cstring>

#define ESP_LOGE(tag, fmt, ...) ((void)0)
#define ESP_LOGW(tag, fmt, ...) ((void)0)
#define ESP_LOGI(tag, fmt, ...) ((void)0)
