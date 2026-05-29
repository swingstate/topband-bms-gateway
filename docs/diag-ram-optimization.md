# RAM Optimization Analysis

**Branch:** `iter/charts-drift-tz`  
**Date:** 2026-05-29  
**Status:** Report only — no code changes applied. Optimization applies in a follow-up iteration.

---

## D1 — Where DRAM goes

DRAM = internal SRAM, `MALLOC_CAP_INTERNAL`. Observed steady-state baseline: ~30 KB free,
largest block 8–22 KB (from `/api/diag` fields `dram_free` / `dram_largest_block`). Total
available DRAM on ESP32-S3 is approximately 384 KB (512 KB total SRAM less IRAM).

### D1.1 Dominant consumers (static analysis, no on-device profiling yet)

| Consumer | Size | Notes |
|---|---|---|
| `history_store.cpp` BSS: `s_fine_buf[720]` + `s_coarse_buf[2016]` | **~57 KB** | No `EXT_RAM_BSS_ATTR` — lands in `.dram0.bss`. Largest single static consumer. |
| `handlers_history.cpp` BSS: duplicate read buffers | **~57 KB** | Same two arrays, separate TU. Total history BSS ≈ **114 KB** in DRAM. |
| Task stacks (application) | ~36 KB | ControlTask 8 + HttpTask 12 + MqttTask 6 + HistoryTask 4 + HousekeepingTask 6 KB. |
| WiFi / lwIP system stacks | ~25 KB est. | lwIP tcpip_thread 3 KB configured + WiFi driver internal stacks. |
| WiFi RX static buffers | ~23 KB est. | 10 static RX mgmt buffers × ~1.6 KB each (CONFIG_ESP_WIFI_STATIC_RX_BUFFER_NUM=10, CONFIG_ESP_WIFI_RX_MGMT_BUF_NUM_DEF=5). |
| WiFi dynamic TX/RX buffers | pool varies | Depth 32 each (CONFIG_ESP_WIFI_DYNAMIC_TX/RX_BUFFER_NUM=32). Allocated from DRAM when in use. |
| lwIP TCP buffers | ~11 KB est. | SND+WND 5760 B each × active connections. CONFIG_LWIP_TCP_SND_BUF_DEFAULT=5760, _WND=5760. |
| FreeRTOS TCBs | ~3 KB est. | ~20 tasks × ~140 B per TCB; trace facility adds ~8 B per task (160 B total, negligible). |
| MQTT queue storage | **0 DRAM** | `alloc_queue_storage()` uses `MALLOC_CAP_SPIRAM`; 64 × 1160 B = ~74 KB lives in PSRAM. |
| Alert / log queues | **0 DRAM** | Same PSRAM path via `alloc_queue_storage()`. |
| BmsSystemSnapshot (history_task BSS) | ~8 KB | `s_snap` static in history_task.cpp, no PSRAM attr. |

**Summary:** The dominant DRAM consumer is the history ring BSS (~114 KB). This is the primary
target. All queue storage is already in PSRAM (correctly implemented). mbedTLS is compiled in for
WiFi WPA2 crypto only — MQTT is plaintext port 1883, confirmed in `src/mqtt/publisher.cpp`.

### D1.2 Heap / PSRAM observability

The existing `/api/diag` endpoint already emits `dram_free`, `dram_min`, `dram_largest_block`,
`psram_free`, `psram_largest_block` via `heap_caps_get_*`. This is sufficient for regression
monitoring. No additional permanent instrumentation is needed.

To get a full region breakdown on-device, call `heap_caps_print_heap_info(MALLOC_CAP_INTERNAL)`
from a boot-time log (add temporarily in `app/boot.cpp`, remove before commit). Output goes to
serial console.

---

## D2 — Task stack right-sizing candidates

**Actual HWM values require on-device measurement from the Part C Diag table** (now working after
this iteration's fix). The values below are estimates from the architecture doc stack sizes;
replace with real HWM after USB-flash.

| Task | Configured stack | Est. HWM reading | Candidate? | Notes |
|---|---|---|---|---|
| ControlTask | 8 192 B | Read from Diag | Likely not | Deepest call path: RS485 parse + safety + CAN TX. Conservative. |
| HttpTask | 12 288 B | Read from Diag | Maybe | Has 2 KB `HStream` buf in handler frame + 864 B `task_buf[24]` for uxTaskGetSystemState. Check actual depth. |
| MqttTask | 6 144 B | Read from Diag | Maybe | esp_mqtt client has variable call depth; keep ≥ 1 KB headroom. |
| HistoryTask | 4 096 B | Read from Diag | Yes | Simple float accumulation + LittleFS write. If HWM shows > 1.5 KB free, reduce to 3 KB. |
| HousekeepingTask | 6 144 B | Read from Diag | Maybe | NTP query uses lwIP; keep ≥ 1.5 KB headroom for lwIP stack frames. |

**Methodology once Part C table is live:**
- Flag any task with HWM < 512 B free as DANGEROUSLY LOW (do not reduce).
- Reduce stack of any task with > 1.5 KB unused by 1 KB at a time, test for one full poll cycle.
- Suggested safe minimum after any reduction: observed HWM + 1 024 B margin (more for tasks
  touching lwIP, e.g. HousekeepingTask).

---

## D3 — Buffer / config candidates

### D3.1 Static history BSS → PSRAM (highest value)

**What:** Mark `s_fine_buf` and `s_coarse_buf` in `history_store.cpp` and `handlers_history.cpp`
with `EXT_RAM_BSS_ATTR`. This moves ~114 KB from `.dram0.bss` to `.ext_ram.bss` (PSRAM).

**Estimated DRAM saved:** ~114 KB — by far the largest single optimization.

**Risk:** LOW. These arrays are accessed via CPU (fread/fwrite, memcpy), not DMA. PSRAM is
cache-coherent for CPU reads/writes. LittleFS uses CPU-driven SPI flash I/O, not DMA. File I/O
performance may slow slightly (PSRAM latency vs. DRAM) but HistoryTask runs every 10 s —
imperceptible.

**Test after applying:** Reboot, confirm fine/coarse ring loads correctly from LittleFS. Run for
2+ hours and verify history chart shows data. No functional change expected.

**Implementation:** Add `#include "esp_attr.h"` and change:
```cpp
// history_store.cpp and handlers_history.cpp
static EXT_RAM_BSS_ATTR HistoryFinePoint   s_fine_buf[HISTORY_FINE_CAPACITY]   = {};
static EXT_RAM_BSS_ATTR HistoryCoarsePoint s_coarse_buf[HISTORY_COARSE_CAPACITY] = {};
```
Also consider `s_snap` in `history_task.cpp` (~8 KB BmsSystemSnapshot) for the same treatment.

### D3.2 WiFi dynamic TX/RX buffer counts

**What:** Reduce `CONFIG_ESP_WIFI_DYNAMIC_TX_BUFFER_NUM` and `CONFIG_ESP_WIFI_DYNAMIC_RX_BUFFER_NUM`
from 32 to 16 each in `sdkconfig.esp32s3`.

**Estimated DRAM saved:** ~26 KB (16 × ~1.6 KB per buffer × 2 directions, DRAM pool).

**Risk:** MEDIUM. Reducing buffer depth can cause WiFi throughput drops under concurrent traffic.
For this workload (MQTT to local broker + occasional Web UI), 16 buffers per direction is likely
sufficient. Risk is higher if OTA or large HTTP transfers are active simultaneously.

**Test after applying:** Verify WiFi reconnect after power cycle, MQTT publish stream for 30 min,
HTTP file upload (OTA), concurrent Web UI + MQTT load.

### D3.3 lwIP TCP buffer sizes

**What:** Reduce `CONFIG_LWIP_TCP_SND_BUF_DEFAULT` and `CONFIG_LWIP_TCP_WND_DEFAULT` from 5760 B
to 2920 B each.

**Estimated DRAM saved:** ~5.7 KB per active TCP connection × ~2–3 active connections = ~11–17 KB.

**Risk:** LOW-MEDIUM. This is the per-connection window size, not a pool. For small payloads
(MQTT publishes, JSON API responses), 2920 B is adequate. TCP throughput for large OTA transfers
may decrease slightly.

**Test after applying:** Verify MQTT publish latency, HTTP response times, OTA upload speed.

### D3.4 `CONFIG_SPIRAM_MALLOC_ALWAYSINTERNAL` threshold

**What:** Lower the threshold from 16 384 B to 4 096 B. Heap allocations ≥ 4 KB go to PSRAM
instead of ≥ 16 KB.

**Estimated DRAM saved:** Unknown without heap profiling. Depends on allocation size distribution.

**Risk:** MEDIUM. DMA-capable allocations (e.g., SPI/I2S drivers, some lwIP paths) must come from
DRAM. `MALLOC_CAP_DMA` allocations still use DRAM regardless of threshold. However, some driver
code uses plain `malloc()` expecting DMA-capable memory and the SPIRAM redirect could intercept
it. Risk is higher than D3.1–D3.3.

**Test after applying:** Full system test: WiFi connect, MQTT publish, Web UI, CAN TX, OTA upload.

### D3.5 mbedTLS

**Confirm:** MQTT is plaintext port 1883 (no TLS). Confirmed in `src/mqtt/publisher.cpp`. mbedTLS
is compiled in for WiFi WPA2 crypto only (`CONFIG_ESP_WIFI_MBEDTLS_CRYPTO=y`). It cannot be
disabled without breaking WiFi. No mbedTLS optimization is applicable for this workload.

---

## D4 — Prioritized recommendations

Ranked by (DRAM saved ÷ risk). Safe wins first.

| Rank | Change | DRAM saved | Risk | Apply mode |
|---|---|---|---|---|
| **1** | D3.1: Move history BSS to PSRAM (`EXT_RAM_BSS_ATTR`) | ~114 KB | LOW | Batch with D2 |
| **2** | D2: Task stack right-sizing (post-HWM measurement) | 4–12 KB est. | LOW per task | Each task in isolation |
| **3** | D3.3: Reduce lwIP TCP buffer sizes (5760 → 2920 B) | ~11–17 KB | LOW-MEDIUM | In isolation, test thoroughly |
| **4** | D3.2: Reduce WiFi dynamic buffer counts (32 → 16) | ~26 KB | MEDIUM | In isolation, test WiFi reconnect + OTA |
| **5** | D3.4: Lower SPIRAM_MALLOC_ALWAYSINTERNAL (16 KB → 4 KB) | unknown | MEDIUM | Last, only if margin still insufficient |

**Batch vs. isolation guidance:**
- D3.1 (history BSS) + D2 (stack sizing) can be batched — both are pure DRAM reduction with no
  functional behavioral change. If both pass, the combined delta in `/api/diag` confirms the saving.
- D3.3, D3.4, D3.5 must each be applied and verified in isolation: their failure modes involve
  WiFi/lwIP/MQTT instability which can be hard to distinguish if changes are stacked.

**Expected outcome after applying D3.1 + D2:** DRAM free should increase from ~30 KB to ~140–145 KB.
DRAM largest block should stabilize around 80–100 KB. This eliminates the fragmentation risk that
caused the recent DRAM exhaustion panic.

---

*Report written on branch `iter/charts-drift-tz`. No sdkconfig, buffer size, task stack size, or
allocation strategy changed in this commit beyond the Part C trace-facility enable (required for
the Diag table to function). All optimization changes are deferred to a follow-up iteration.*
