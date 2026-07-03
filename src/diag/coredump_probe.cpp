#include "diag/coredump_probe.h"
#include "esp_core_dump.h"
#include "esp_log.h"
#include "sdkconfig.h"
#include <cstdio>
#include <cstring>

static const char* TAG = "coredump";

namespace diag::coredump {

static ProbeResult s_result = {};
static bool        s_probed = false;

const ProbeResult& probe() {
  if (s_probed) return s_result;
  s_probed = true;

  if (esp_core_dump_image_check() != ESP_OK) {
    return s_result;  // no dump — the common case
  }
  s_result.present = true;

#if defined(CONFIG_ESP_COREDUMP_ENABLE_TO_FLASH) && defined(CONFIG_ESP_COREDUMP_DATA_FORMAT_ELF)
  esp_core_dump_summary_t sum = {};
  const bool parsed = (esp_core_dump_get_summary(&sum) == ESP_OK);

  // Stale-format guard: a BIN-era dump passes the image check but yields a
  // failed or empty summary (no task name, PC 0). Keeping it would re-pay
  // the multi-second garbage parse on every boot and raise a false
  // "previous panic" alert, so erase it. A genuine ELF panic dump always
  // carries the crashing task name.
  const bool usable = parsed && !(sum.exc_task[0] == '\0' && sum.exc_pc == 0);
  if (!usable) {
    ESP_LOGW(TAG, "unusable coredump in flash (foreign format or corrupt) — erasing");
    if (esp_core_dump_image_erase() != ESP_OK) {
      ESP_LOGW(TAG, "coredump erase failed");
    }
    s_result.present = false;
    return s_result;
  }

  s_result.has_summary = true;
  snprintf(s_result.task, sizeof(s_result.task), "%s", sum.exc_task);
  snprintf(s_result.sha256, sizeof(s_result.sha256), "%s",
           reinterpret_cast<const char*>(sum.app_elf_sha256));
  snprintf(s_result.pc_hex, sizeof(s_result.pc_hex), "0x%08lx",
           (unsigned long)sum.exc_pc);
  s_result.version = sum.core_dump_version;
#endif

  return s_result;
}

}  // namespace diag::coredump
