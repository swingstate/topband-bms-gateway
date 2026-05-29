# MQTT Crash Investigation — Code Review

> Branch: `review/mqtt-crash`
> Reviewed: 2026-05-28
> Configuration during crash: MQTT enabled, PerCell level, HA discovery enabled, 2 packs online

---

## Summary

### Finding 1: ArduinoJson heap alloc/free on every payload build — sustained DRAM fragmentation [severity: HIGH] [likelihood: HIGH]

- **Where:** `src/mqtt/payloads.cpp:32`, `payloads.cpp:91`, `payloads.cpp:118`; called from `housekeeping.cpp:97`, `housekeeping.cpp:212`, `housekeeping.cpp:292`
- **What:** Every call to `build_data()`, `build_diag()`, and `build_cells()` declares a `JsonDocument doc` as a local variable. ArduinoJson V7's `JsonDocument` uses a dynamic memory pool obtained from `malloc`/`free` at construction/destruction time. These pool sizes vary by document (~512 B for diag, ~800 B for data with 16 packs, ~900 B for cells with 16 cells). They draw from DRAM by default because the `CONFIG_SPIRAM_USE_MALLOC` threshold is 16 KB (from Phase I fix) and these pools are well below that threshold.
  - `build_data()` called every 5 s → 720 cycles/hour, 4320 cycles/6 h
  - `build_cells()` called every 20 s → 180 cycles/hour, 1080 cycles/6 h
  - `build_diag()` called every 30 s → 120 cycles/hour, 720 cycles/6 h
  - HA discovery also allocates ~80 JsonDocument objects at boot (one per entity)
  - Each call is a DRAM malloc/free cycle of a different size (512–900 B depending on pack count and online status)
- **Why it could cause a panic:** After thousands of cycles of irregular-size alloc/free, the DRAM heap becomes fragmented into blocks smaller than any individual request needs. When the esp_mqtt internal task or lwIP stack tries to allocate a TCP segment buffer and the DRAM allocator cannot satisfy it, the IDF panics. This is consistent with the Phase I post-OTA finding: "free_heap_b aggregates across all regions and HIDES per-region exhaustion." The `/api/diag` endpoint exposes `heap_min` but not `MALLOC_CAP_INTERNAL` separately — so the fragmentation is invisible until the crash.
- **Proposed fix:** Pre-allocate a single reusable PSRAM buffer per payload type and serialize directly into it without constructing a `JsonDocument` each time. Alternatively, call `doc.clear()` on a module-level static `JsonDocument` and re-use the same pool. The architecture already established the HStream streaming JSON pattern in Phase H2 for `/api/history`; `build_data` and `build_cells` could use the same approach (format directly into the caller's `char* out` buffer with a manual JSON writer). For the HA discovery calls in `ha_discovery.cpp`, which already have a 640–700 B `char buf[]` on the stack, the `JsonDocument` heap allocation could be replaced by `ArduinoJson::StaticDocument<N>` (stack-allocated pool, no heap) where N matches each entity type's known maximum size.

---

### Finding 2: `ha_discovery::publish_all()` runs synchronously in MqttTask, blocking queue drain for ~1.0–1.5 s [severity: HIGH] [likelihood: HIGH]

- **Where:** `src/mqtt/publisher.cpp:197–199`; `src/mqtt/ha_discovery.cpp:356–393`
- **What:** On MQTT connect, `MqttTask` calls `mqtt::ha_discovery::publish_all()` directly (line 198 in publisher.cpp). `publish_all` makes ~80 direct `esp_mqtt_client_publish()` calls with `vTaskDelay()` between each:
  - 20 system entities × 20 ms delay = 400 ms
  - 11 × 2 packs plain-pack entities × 10 ms = 220 ms
  - 4 × 2 packs cells-JSON entities × 10 ms = 80 ms
  - 15 × 2 packs per-cell voltage entities × 10 ms = 300 ms
  - Total vTaskDelay: ~1,000 ms of 10–20 ms sleeps, plus actual publish time (~5 ms × 93 entities = ~465 ms)
  - Wall-clock window: ~1.1–1.5 s during which MqttTask is inside `publish_all` and NOT draining `q_mqtt_publish`
- **Why it could cause a panic:** During this 1.5 s window, HousekeepingTask (priority 1, lower than MqttTask's priority 3) gets to run only during the `vTaskDelay` gaps. In each gap it produces items and enqueues them. The queue (`q_mqtt_publish`, depth 32) receives new items but MqttTask is not draining it. When the queue fills, `post_mqtt` drops the oldest item and re-enqueues. If the reconnect scenario involves rapid disconnect/reconnect (e.g., broker restart), the queue oscillates between full and dropped states, causing `mqtt_publish_drops` to accumulate rapidly. More critically: the 80 direct `esp_mqtt_client_publish()` calls from `publish_all` bypass the drop-oldest queue protection and compete directly with the MQTT internal task for TCP send buffer space. Under DRAM pressure (Finding 1), a TCP buffer allocation failure here would panic.
- **Proposed fix:** Move HA discovery publishes through `q_mqtt_publish` (enqueue as Discovery topic type) so the existing backpressure and drop-oldest mechanism applies. Alternatively, post a single "send discovery" command to a dedicated command queue that `MqttTask` drains piecemeal across multiple iterations (one entity per 50 ms tick), keeping the queue drain path unblocked.

---

### Finding 3: HousekeepingTask 4096-byte stack with `IndivTopic iv_topics[20]` (1440 B) as a stack local [severity: HIGH] [likelihood: MEDIUM]

- **Where:** `src/app/housekeeping.cpp:133–155`
- **What:** Inside `housekeeping_task_entry`, within the DATA-period conditional block (line 90), the following is declared as a local:
  ```cpp
  struct IndivTopic { const char* suffix; char value[64]; };
  IndivTopic iv_topics[] = { /* 20 entries */ };
  ```
  Each entry is `sizeof(const char*) + 64` = 8 + 64 = 72 bytes × 20 = **1440 bytes**. GCC at `-Os` reserves the full function frame at entry, so this 1440 B is always allocated on the stack regardless of whether the DATA conditional fires. Combined with:
  - Other locals: `last_cells_ms[16]` (64 B), timing vars (28 B), iv_cell_* and iv_rt_* vars (49 B), `val[32]` in the per-pack loop (32 B) → ~173 B additional
  - FreeRTOS interrupt context save frame on ESP32-S3 (XTENSA LX7 saves ~600 B on the task stack when an interrupt fires)
  - Nested call frames: `build_data()` allocates locals for cell aggregation (~100 B), `estimate_min()` (~50 B), `snapshot_bus::read()` (~50 B)
  - `log_hook` fires on every `ESP_LOGI/W/E` call and allocates `char line[256]` on the calling task's stack (see Finding 8)
  - Combined deepest chain: 1440 + 173 + 600 + 200 + 256 = **2669 B** at peak, leaving only ~1427 B headroom in 4096 B
  The Phase H1 finding reported a stack overflow from ~10 KB of locals. After the fix, large structs were promoted to BSS, but `iv_topics[20]` is new code that added 1440 B back to the stack frame without the same fix being applied.
- **Why it could cause a panic:** If a deep call path is taken while the HousekeepingTask stack is near capacity — for example, an alert is emitted (`diag::alerts::emit`, which adds ~200 B of locals + the `log_hook` 256 B) during the per-pack publish loop while `build_data()` is on the stack — the total can exceed 4096 B. Stack overflow corrupts the adjacent FreeRTOS TCB or heap, causing panics in unrelated code (as observed in H1: "assert in lwIP tcpip_thread context but actual cause was elsewhere"). This matches the 6-hour intermittent symptom: overflow requires a specific depth of concurrent call stack that only occurs occasionally.
- **Proposed fix:** Promote `iv_topics[]` to a `static IndivTopic` array at module scope (same pattern used for `s_snap`, `s_safety`, `s_req`). HousekeepingTask is single-instance so no aliasing risk. This reclaims 1440 B from the stack frame. Alternatively, use a tighter `char value[24]` for numeric values and a separate 48 B buffer for `sys_message` only.

---

### Finding 4: `q_mqtt_publish` queue depth 32 vs. burst of 43–59 items per HousekeepingTask cycle [severity: MEDIUM] [likelihood: HIGH]

- **Where:** `src/bus/queues.h:37–45`; `src/app/housekeeping.cpp:89–325`
- **What:** Per 5-second DATA tick with 2 packs at PerCell level, HousekeepingTask posts to the queue:
  - 20 system `IndivTopic` items (from `iv_topics[]` loop, lines 184–193)
  - 22 per-pack `IndivTopic` items (11 topics × 2 packs, lines 226–275)
  - 1 Data JSON item (line 101)
  - On PerCell ticks (every 20 s per pack, via round-robin): 1 Cells JSON + 15 individual cell_v items = 16 more items
  Total: 43 items on regular ticks, 59 items on PerCell ticks.
  Queue depth is 32. `post_mqtt()` drops the **oldest** item and re-enqueues the new one on overflow. So on every DATA+PerCell tick, 27 items are silently dropped. On regular DATA ticks, 11 items are dropped. The dropped items are the oldest system-state values.
- **Why it could cause a panic:** Not a direct crash trigger, but: (a) the repeated `post_mqtt` drop-oldest path executes `xQueueReceive` + `xQueueSend` on every overflow, doubling the FreeRTOS queue operation count and increasing time in `post_mqtt`; (b) `mqtt_publish_drops` increments continuously but is not wired to an alert or watchdog; (c) under DRAM pressure, even `xQueueSend` failing silently (pdFALSE) could leave state inconsistent. The constant drops also mean subscribers receive a stale/incomplete view.
- **Proposed fix:** Increase queue depth to 64 or adopt the round-robin staggered scheduler (see Proposed Restructure) which limits burst to ~5 publishes per tick, eliminating overflow entirely.

---

### Finding 5: `cleanup_stale()` writes NVS from MqttTask context, blocking ~20–40 ms [severity: MEDIUM] [likelihood: HIGH]

- **Where:** `src/mqtt/ha_discovery.cpp:402–443`; called from `src/mqtt/publisher.cpp:197`
- **What:** `cleanup_stale()` is called on the first connection after boot, from inside `mqtt_task_entry`. It calls `nvs_open()`, `nvs_get_str()`, `nvs_set_str()`, `nvs_commit()`, and `nvs_close()`. The `nvs_commit()` call performs a flash erase+write, which blocks the calling task for typically 20–40 ms on ESP32-S3 (measured value depends on NVS partition state). During this window:
  - MqttTask is blocked; it cannot drain `q_mqtt_publish`
  - The MQTT internal task (esp_mqtt, priority 3) may need to send a PINGREQ or process an incoming PINGRESP during this 20–40 ms; since MqttTask holds CPU time via its own blocking call, the MQTT internal task is delayed until MqttTask yields
  - The 30-second keepalive (`mcfg.session.keepalive = 30`) means a 40 ms delay is harmless in normal operation, but this runs at the worst possible moment (immediately after connecting, when the broker most needs to see traffic)
- **Why it could cause a panic:** Indirect: the 20–40 ms NVS stall delays queue drain. If HousekeepingTask fires during this window (it runs every 1 s), the queue fills by ~8 items during the stall. Combined with the already-near-full queue from normal operation (Finding 4), this pushes drop rates higher. Not a direct crash path, but compounds with Findings 1 and 2.
- **Proposed fix:** Move the NVS read-write out of MqttTask entirely. The cleanup guard (checking if we've already run for this firmware version) should run once at boot in `HousekeepingTask` or `app::boot::run()`, before MQTT starts. The result (run or skip) is passed to the publisher via a flag so the event handler can call cleanup immediately on connect without any blocking I/O.

---

### Finding 6: `trigger_ha_discovery()` modifies `s_ha_discovery_done` and `s_cfg` under spinlock, but `MqttTask` reads `s_ha_discovery_done` outside the spinlock in the connect path [severity: MEDIUM] [likelihood: LOW]

- **Where:** `src/mqtt/publisher.cpp:394–405` (`trigger_ha_discovery`); `publisher.cpp:190–199` (MqttTask connect path)
- **What:** `trigger_ha_discovery()` (called from HTTP handler context) takes `portENTER_CRITICAL(&s_mux)` and sets `s_just_connected = true`, `s_cfg.ha_discovery_enabled = true`, `s_ha_discovery_done = false`. However, in `mqtt_task_entry`'s connect path:
  ```cpp
  bool ha_en = app::get_config().ha_discovery_enabled;   // not under s_mux
  if (ha_en && !s_ha_discovery_done) {                   // s_ha_discovery_done not under s_mux
  ```
  `s_ha_discovery_done` is read outside the critical section. On a multicore SMP system (Core 0 and Core 1), if `trigger_ha_discovery()` runs on a Core 0-dispatched HTTP handler at the exact moment `MqttTask` reads `s_ha_discovery_done`, the read may observe stale data without the memory barrier that `portENTER_CRITICAL`/`portEXIT_CRITICAL` provides. This could cause discovery to be suppressed (if the old `true` value is observed after `trigger_ha_discovery()` wrote `false`).
- **Why it could cause a panic:** Low probability crash path; more likely causes incorrect behavior (missed re-send) than a crash. However, if the HTTP handler and MqttTask simultaneously access `s_cfg` without a barrier, a torn read of `s_cfg` (which is ~600 bytes) is theoretically possible.
- **Proposed fix:** Read `s_ha_discovery_done` and `ha_en` under the same `portENTER_CRITICAL` block where `s_just_connected` is read, and snapshot both with the same atomicity guarantee.

---

### Finding 7: `stop()` calls `vTaskDelete(s_task_handle)` then immediately acquires `portENTER_CRITICAL(&s_mux)` — potential spinlock orphan [severity: MEDIUM] [likelihood: LOW]

- **Where:** `src/mqtt/publisher.cpp:319–343` (`stop()`); called from `reconfigure()` line 347
- **What:** `stop()` force-deletes MqttTask with `vTaskDelete(s_task_handle)`. If MqttTask is mid-execution inside `publish_request()` and has already taken `portENTER_CRITICAL(&s_mux)` (lines 118–125) when `vTaskDelete` removes it from the scheduler, that spinlock (`s_mux`) will remain locked forever. `stop()` then immediately calls `portENTER_CRITICAL(&s_mux)` to update `s_state` — this will spin forever (WDT reboot).
  In practice this window is narrow (the critical section in `publish_request` is only a few instruction cycles), and `reconfigure()` is only triggered by user action (HTTP config save). It is not in the normal crash path but is a latent crash that could manifest as a WDT reboot when the user saves settings while the MQTT publish burst is active.
- **Proposed fix:** Before calling `vTaskDelete`, set a module-level `s_stop_requested` flag that `mqtt_task_entry` polls at the top of its loop. `stop()` then waits for the task to self-exit (via `vTaskDelay` until `s_task_handle` becomes NULL). No `portENTER_CRITICAL` is held across the self-deletion.

---

### Finding 8: `log_hook` in `log_ring.cpp` adds 256-byte stack frame on EVERY calling task [severity: LOW] [likelihood: MEDIUM]

- **Where:** `src/diag/log_ring.cpp:36–49`
- **What:** The vprintf hook registered via `esp_log_set_vprintf` is called from any task that emits a log line. The hook allocates:
  ```cpp
  char line[256];   // on caller task's stack
  ```
  plus a `va_list args_copy`. Any task that calls `ESP_LOGI/W/E` has 256+ B transiently added to its stack frame. This fires on HousekeepingTask, MqttTask, HistoryTask, and ControlTask.
- **Why it could cause a panic:** In isolation this is manageable. But combined with Finding 3 (HousekeepingTask near-full stack from `iv_topics[20]`), a log call from within a nested call path in HousekeepingTask adds the final 256 B that pushes over the limit. `alerts::emit()` (called e.g. from the MQTT CONNECTED event handler) internally calls `ESP_LOGW/I` paths, which trigger the hook.
- **Proposed fix:** Reduce `line[256]` to `line[MAX_LINE + 4]` = 124 B (MAX_LINE is 120 in `log_ring.cpp`; longer lines are truncated anyway). Log lines longer than 120 chars are already silently truncated in `append()` via `strncpy(..., MAX_LINE - 1)`. The 256-byte local only preserves characters that are immediately discarded.

---

### Finding 9: Coredump endpoint returning "Not found" — post-mortem evidence is unavailable [severity: HIGH] [likelihood: N/A]

- **Where:** `src/web/handlers_diag.cpp` (not fully reviewed; referenced by prompt)
- **What:** `/api/diag/coredump.bin` returns "Not found". The coredump partition exists (64 KB, per `partitions.csv` architecture §8.2). Either: (a) the partition is not being written to (esp_core_dump not configured in `sdkconfig.defaults`), (b) the crash is a WDT reset (which may not write a coredump depending on IDF config), or (c) the endpoint implementation has a bug in reading the partition. Without the coredump, root cause is inferred from code review rather than confirmed.
- **Why it could cause a panic:** It doesn't — but the absence of coredump data is a compounding factor that prevents definitive diagnosis.
- **Proposed fix:** Verify `sdkconfig.defaults` contains `CONFIG_ESP_COREDUMP_ENABLE=y` and `CONFIG_ESP_COREDUMP_TO_FLASH=y`. Verify the coredump partition in `partitions.csv` matches what `esp_core_dump_image_check()` expects. Add a boot-time check in `boot.cpp` that reads `esp_core_dump_image_check()` and logs "coredump found" or "no coredump" to the alert ring so the next boot's diag page shows whether the previous crash left a dump.

---

## Suspect Ranking

**Most likely to least likely as the primary crash cause:**

1. **Finding 1 — ArduinoJson DRAM heap fragmentation** (HIGH/HIGH): Explains the 6-hour delay before crash. 4320+ alloc/free cycles of irregular DRAM blocks (512–900 B each) over 6 hours accumulate fragmentation. The lwIP or MQTT internal task fails a TCP buffer allocation. IDF panics. Matches: deferred onset, MQTT load makes it worse (more publish paths = more `JsonDocument` churn).

2. **Finding 3 — HousekeepingTask stack overflow from `iv_topics[20]`** (HIGH/MEDIUM): Explains intermittent nature — only triggers when a specific call depth is reached during the DATA publish path while an interrupt or alert log fires. The corrupted TCB/heap causes panic in an unrelated task (as seen in Phase H1 crash manifesting as lwIP assert).

3. **Finding 2 — Discovery blocking queue drain** (HIGH/HIGH): Happens on every MQTT connect or reconnect. Does not crash by itself, but causes queue overflow and compounds DRAM pressure during the 1.5 s blocking window.

4. **Finding 4 — Queue depth vs. burst mismatch** (MEDIUM/HIGH): Causes data loss on every cycle, not a crash trigger in isolation, but contributes to the `post_mqtt` hot path consuming more CPU and FreeRTOS queue operations than designed.

5. **Finding 5 — NVS write in MqttTask** (MEDIUM/HIGH): One-time block (once per firmware version) that delays queue drain. More of a contributing factor than root cause.

6. **Findings 6, 7, 8** (MEDIUM/LOW): Latent bugs unlikely to be the 6-hour crash trigger under normal steady-state operation.

---

## Burst-load Analysis

### What happens during the ~43-publish burst at each 5-second DATA tick (2 packs, PerCell)

**Functions called, in order, within a single `housekeeping_task_entry` loop iteration:**

1. `bus::snapshot_bus::read(s_snap)` — seqlock read of ~8 KB from PSRAM into DRAM `s_snap`
2. `bms::poller::read_safety_state(s_safety)` — spinlock copy of SafetyState (~370 B)
3. `mqtt::payloads::build_data(s_snap, s_safety, ...)` — `JsonDocument` heap alloc (~800 B DRAM), serialize ~800 B JSON into `s_req.payload[1024]`, heap free
4. `post_mqtt(s_req)` × 1 — `xQueueSend` of 1096 B into `q_mqtt_publish`
5. **iv_topics fill loop (20 items):** 20 × `snprintf` into `iv_topics[i].value[64]`, followed by 20 × `post_mqtt(s_req)` (each involves `snprintf(s_req.topic_suffix)` + `memcpy(s_req.payload)` + `xQueueSend`)
6. **PerPack loop (11 × 2 = 22 posts):** 22 × lambda `post_pack()` which formats `val[32]` and calls `post_mqtt`
7. **PerCell round-robin (when due — every 20 s per pack):**
   - `bus::snapshot_bus::read(s_snap)` — second seqlock read this tick
   - `mqtt::payloads::build_cells(s_snap.pack[pi], ...)` — `JsonDocument` heap alloc (~600 B DRAM), serialize, heap free
   - `post_mqtt(s_req)` × 1 — Cells JSON
   - inner cell loop: 15 × `snprintf` + `post_mqtt` for individual cell_v topics

**Resources touched per burst:**
- DRAM heap: 2–3 `JsonDocument` alloc/free cycles (build_data + optionally build_cells)
- `q_mqtt_publish`: 43 to 59 `xQueueSend` calls; queue depth 32 → 11–27 items dropped via `post_mqtt` drop-oldest
- FreeRTOS critical sections: 2 × `portENTER_CRITICAL` (snapshot_bus reads) + up to 59 × 2 (queue send + optional drop-oldest receive)
- Stack: `iv_topics[20]` (1440 B) allocated at function entry regardless of DATA timer

**Worst-case timing if all publishes are slow:**
- MqttTask at priority 3 preempts HousekeepingTask at priority 1 between each `post_mqtt` call
- If MqttTask's `esp_mqtt_client_publish` stalls (broker slow, TCP back-pressure): each call blocks for up to `mcfg.network.reconnect_timeout_ms = 5000` ms before returning -1
- In practice QoS 0 publish is non-blocking (enqueued into MQTT internal buffer); stalls happen in the MQTT internal task's TCP send loop, not in the calling `esp_mqtt_client_publish` call itself
- Realistic worst case: 43 items × 2 ms each = 86 ms for the burst phase; MQTT internal task drains TCP over next 200–500 ms
- During a reconnect (discovery), this stretches to ~1.5 s as analyzed in Finding 2

**MqttTask's queue drain path:**
- `MqttPublishRequest req` declared as a local (1096 B on MqttTask stack = 6144 B)
- `while (xQueueReceive(q_mqtt_publish, &req, 0) == pdTRUE)` drains until empty
- Each `publish_request(req)` calls `esp_mqtt_client_publish()` then takes `portENTER_CRITICAL` briefly for stats
- After drain loop: `vTaskDelay(50 ms)` — 20 cycles/second, each potentially draining up to 32 items
- At 8.6 items/second average production rate and 20 drain opportunities/second, drain rate is sufficient IF QoS 0 publish is fast (< 5 ms each)

---

## Proposed Restructure (Preventive)

### Round-robin staggered MQTT scheduler

**Goal:** limit in-flight queue depth to ≤ 5 publishes per 1-second HousekeepingTask tick, distributed evenly across a rolling window.

**Data structure (module-level in housekeeping.cpp):**

```cpp
// Three rotating cursors — each advances by one slot per tick
static uint8_t s_sys_cursor;    // 0..N_SYS_TOPICS-1 (20 system topics)
static uint8_t s_pack_cursor;   // 0..(N_PACK_TOPICS * bms_count)-1
static uint8_t s_cell_cursor;   // 0..(16 * bms_count)-1 (cell voltage slots)

// Tick counters for cadence enforcement
static uint32_t s_data_json_tick;   // publishes full data JSON every 25 ticks (~25 s)
static uint32_t s_diag_json_tick;   // publishes diag JSON every 30 ticks (~30 s)
```

**Per-tick logic (called every 1 s from HousekeepingTask loop):**

```
On each 1-second tick:

1. Read snapshot once (s_snap, s_safety) — used for all publishes this tick.

2. System-topic group (target: 4 publishes per tick → full rotation every 5 ticks):
   Publish topics[s_sys_cursor % 20] through topics[(s_sys_cursor + 3) % 20]
   Advance s_sys_cursor += 4.
   After 5 ticks, all 20 system topics have been published once.
   Effective cadence: each system topic published every ~5 s.

3. Per-pack group (target: 2–3 publishes per tick per pack):
   For each pack that is online:
     Publish pack_topics[s_pack_cursor % N_PLAIN_PACK]
     Advance s_pack_cursor.
   With 2 packs × 11 topics = 22 items, rotation completes every ~11 ticks (~11 s).
   Effective cadence: each per-pack topic published every ~10–11 s.

4. Per-cell group (target: 2 cell voltage publishes per tick):
   For cells_rr (existing round-robin), advance by 2 cells per tick instead of all 15.
   With 2 packs × 15 cells = 30 cells, rotation completes every ~15 ticks (~15 s).
   Effective cadence: each cell voltage topic published every ~30 s.

5. Data JSON (full snapshot): publish every DATA_PERIOD_MS / tick_period = every 5 ticks.
   1 publish every 5 ticks.

6. Diag JSON (when enabled): publish every 30 ticks.
   1 publish every 30 ticks.

Max publishes per tick: 4 (system) + 3 (pack) + 2 (cell) + 1 (data JSON) = 10 max.
Typical tick: 4 + 2 + 2 + 0 = 8 publishes. No overflow of a 32-deep queue.
```

**Cadences vs. current:**

| Topic group | Current cadence | Staggered cadence | Acceptable? |
|---|---|---|---|
| System IndivTopic | 5 s per topic | 5 s per topic (same) | Yes |
| Per-pack IndivTopic | 5 s per topic | 11 s per topic | Yes (monitoring interval) |
| Per-cell voltage | 20 s per pack | 30 s per cell | Yes (cell voltage stable) |
| Data JSON | 5 s | 5 s (same) | Yes |
| Diag JSON | 30 s | 30 s (same) | Yes |

**HA discovery** would remain on the existing path but routed through `q_mqtt_publish` (one entity per 50 ms MqttTask drain tick), eliminating the 1.5 s blocking window entirely. At 50 ms per entity × 93 entities = 4.65 s total discovery time, completely transparent to queue load.

---

## Recommendations

Ordered by expected impact on crash elimination:

1. **[CRITICAL] Fix ArduinoJson DRAM fragmentation (Finding 1):** Replace `JsonDocument doc` locals in `build_data()`, `build_cells()`, `build_diag()` with a module-level static `StaticDocument<N>` cleared per call, or with a hand-rolled JSON writer targeting the caller's `char* out` buffer directly (the HStream pattern already proven in `/api/history` and `/api/diag`). This eliminates the primary suspected crash mechanism (sustained DRAM fragmentation) and also reduces DRAM usage by the pool sizes (800 B × 3 functions freed after each call instead of fragmented). Expected outcome: eliminates crash at 6-hour horizon.

2. **[CRITICAL] Move `iv_topics[20]` to BSS static (Finding 3):** `static IndivTopic iv_topics[20]` at module level in housekeeping.cpp. Reclaims 1440 B from the HousekeepingTask stack frame. Brings worst-case stack depth from ~2700 B to ~1260 B, providing comfortable 2800 B headroom against the 4096 B budget. Expected outcome: eliminates the stack-overflow crash path that explains H1 findings recurrence.

3. **[HIGH] Implement staggered round-robin publish scheduler (Proposed Restructure):** Limits burst to ≤ 10 publishes per 1-second tick. Eliminates queue overflow (Finding 4) as a steady-state condition. Expected outcome: eliminates continuous `mqtt_publish_drops`, improves subscriber data freshness, reduces CPU time in `post_mqtt` drop-oldest path.

4. **[HIGH] Route HA discovery through `q_mqtt_publish` queue (Finding 2):** Move `publish_all()` to post entities to `q_mqtt_publish` as a `Topic::Discovery` type. MqttTask drains the queue at its normal 50 ms cadence. Eliminates the 1.5 s blocking window that starves the queue drain path during reconnect. Expected outcome: reconnect-triggered discoveries are transparent to system load.

5. **[MEDIUM] Move NVS write in `cleanup_stale()` out of MqttTask (Finding 5):** Execute the stale-key NVS check at boot (before MQTT starts) in `boot.cpp` or `HousekeepingTask` init. Pass a `bool cleanup_needed` flag to the publisher. Expected outcome: removes blocking I/O from the MQTT publish path.

6. **[MEDIUM] Reduce `log_hook` stack buffer from 256 B to 124 B (Finding 8):** `char line[MAX_LINE + 4]` is sufficient since `append()` truncates at MAX_LINE - 1 = 119 chars. Expected outcome: reclaims 132 B from every task's transient stack depth during log calls — particularly meaningful for HousekeepingTask.

7. **[MEDIUM] Fix coredump endpoint (Finding 9):** Verify `sdkconfig.defaults` enables coredump-to-flash. Add boot-time `esp_core_dump_image_check()` and emit an alert if a dump is present. This will provide definitive evidence for root cause in the next crash rather than requiring code-review inference. Expected outcome: definitive crash diagnosis on next occurrence.

8. **[LOW] Fix `stop()` spinlock orphan (Finding 7):** Use a cooperative shutdown flag rather than `vTaskDelete` on MqttTask. Expected outcome: eliminates rare WDT crash on user config save during active publish burst.

9. **[LOW] Fix `s_ha_discovery_done` read outside spinlock (Finding 6):** Snapshot `s_ha_discovery_done` and `ha_en` under the same `portENTER_CRITICAL` block as `s_just_connected`. Expected outcome: eliminates torn-read risk on SMP, ensures consistent discovery-enable behavior.

10. **[DIAGNOSTIC] Wire `MALLOC_CAP_INTERNAL` and `MALLOC_CAP_SPIRAM` free sizes separately into `/api/diag`:** Replace the single `heap_min` field with two values: `dram_free` and `psram_free`. Per Phase I lesson: aggregate heap free is meaningless for diagnosing regional exhaustion. This visibility would have confirmed Finding 1 without code review. Expected outcome: makes DRAM fragmentation observable before the next crash.
