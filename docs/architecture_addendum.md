# Architecture-Doc Nachtrag — alles seit Phase I

> **Zum Einarbeiten in `docs/architecture.md` (lokal, gitignored).**
>
> Zwei Einfügepunkte:
> 1. **Block A** (neue R9-Einträge) → ans **Ende der R9-Sektion**, direkt **vor** `## 10.11 Phase H — completion summary`.
> 2. **Block B** (neue Completion-Summaries + ersetzter Status) → §10.12 bleibt, danach kommen die neuen §10.12b/c/d/e, und **§10.13 wird komplett ersetzt** durch die neue Fassung unten.
>
> Reihenfolge chronologisch: J1+J2 → MQTT-Iterationen → MQTT-Crash → RAM-Opt → Charts → Board-Selector.

---

# BLOCK A — Neue R9-Einträge (ans Ende von R9, vor §10.11)

**Phase J1+J2 finding (RESOLVED): CMakeLists registration needed for new protocol sources.**
- Observed: new `src/can/pylontech.cpp` and `src/can/sma.cpp` compiled in isolation but the build failed to link / didn't pick them up.
- Root cause: new source files must be registered in the component `CMakeLists.txt` SRCS list. Same oversight had occurred earlier in Phase H3a.
- Resolution: added both files to CMakeLists.txt. Build clean.
- Lesson: any new `.cpp` under `src/` must be added to the component CMakeLists SRCS. This is the second time it bit us — when adding source files, check CMakeLists in the same commit.

**Post-Phase-I MQTT iterations (RESOLVED): plain-text topic split + per-pack + per-cell levels.**
- The MQTT schema evolved from JSON-blob topics to individual plain-text retained topics (one value per topic), and the publish-level enum gained a `PerPack` level between `DataSystem` and `PerCell`.
- NVS migration trap: inserting an enum value (`PerPack=3`, shifting `PerCell` 3→4) requires a one-shot migration at config load (stored value 3 → 4). Without it, devices on the old `PerCell` jump to the new `PerPack` after update.
- Per-cell publish path was initially missed when system topics were split — per-pack worked, per-cell topics never published. Diagnosed via `mosquitto_sub`: 11 per-pack topics present, zero `cell_v_NN`. Was a publisher gap, not a discovery bug.
- HA device hierarchy: packs are sub-devices of the gateway via `via_device`. `sw_version` belongs ONLY on the gateway device — packs cannot report their BMS firmware over RS485, so showing the gateway's firmware on a pack would be misleading. Gateway model string is "BMS Gateway", not the chip name.
- Lesson: when splitting an aggregate payload into per-item topics, audit every level (system / per-pack / per-cell) for the publish path AND the HA discovery path separately — they're independent code.

**MQTT-load panic (RESOLVED): the big one — panic every few hours under expanded MQTT load.**
- Observed: device panicked and auto-rebooted roughly every few hours after the MQTT expansion (per-pack + per-cell + HA discovery). No serial capture available. `/api/diag/coredump.bin` initially returned "Not found".
- Investigation method: review-only branch produced `docs/diag-mqtt-crash-review.md` with 9 ranked findings; then a fix branch with 8 commits; root cause finally CONFIRMED by coredump after the coredump endpoint was fixed.
- Coredump verdict: `StoreProhibited` (write to invalid address `0x6072b`) with ALL task TCBs reported corrupt — the signature of memory corruption from an out-of-bounds write, consistent with stack overflow. Decoded with: `espcoredump.py info_corefile -t raw --gdb <toolchain>/xtensa-esp32s3-elf-gdb -c dump.bin firmware.elf` (note `-t raw` for `CONFIG_ESP_COREDUMP_DATA_FORMAT_BIN`; needs the ELF of the exact crashed build for symbols).
- Root causes (multiple, all fixed):
  1. **ArduinoJson DRAM fragmentation:** `build_data/build_cells/build_diag` allocated a fresh `JsonDocument` heap pool (512–900 B DRAM, below the 16 KB SPIRAM threshold) on every publish. Thousands of irregular alloc/free cycles fragmented DRAM until a TCP buffer alloc failed. Fix: module-level static `JsonDocument`, `.clear()` and reuse.
  2. **HousekeepingTask stack overflow:** `IndivTopic iv_topics[20]` (1440 B) was a stack local — reintroducing exactly the Phase H1 stack-overflow class. The H1 BSS-promotion lesson had not been applied to new code. Fix: move to module-level static.
  3. **HA discovery blocked the MQTT task:** `publish_all()` made ~80 direct publishes + vTaskDelays inside MqttTask (~1.5 s), starving the queue drain. Fix: route discovery through `q_mqtt_publish`.
  4. **Bursty publishing:** 43–59 publishes in a 50 ms window vs a 32-deep queue → 11–27 silent drops/cycle. Fix: round-robin staggered scheduler, ≤10 publishes per 1 Hz tick (0 drops over 360k publishes verified).
  5. NVS `cleanup_stale()` ran (blocking) inside MqttTask — moved to boot.
  6. Latent races (torn read of `s_ha_discovery_done`, spinlock orphan in `stop()`'s `vTaskDelete`) — fixed with atomic snapshot + cooperative shutdown.
  7. `log_hook` 256 B stack buffer reduced to 124 B (append truncates at 119 anyway).
  8. Coredump endpoint fixed + boot-time `esp_core_dump_image_check()` emits a CRITICAL alert when a previous-boot dump exists; `/api/diag` now exposes `dram_free`/`psram_free`/`dram_largest_block` separately.
- Verified: 9h 44m+ stable vs previous panic-every-few-hours.
- Lessons:
  1. **Static-locals discipline is not a one-time fix — re-check on every new task-context code.** H1 fixed it; the per-cell iteration silently reintroduced it. Any array/struct >256 B in a task function goes to BSS (single-instance task) or heap/PSRAM. Add to review checklist.
  2. **DRAM fragmentation from repeated irregular sub-16 KB alloc/free is a slow-onset killer.** Anything allocated repeatedly in a hot path must reuse a static pool or live in PSRAM, never per-call malloc/free.
  3. **`largest_block` falling while `free` stays high = fragmentation signature.** Expose per-region free + largest-block in diag; aggregate `free_heap` hides it.
  4. **Stagger periodic multi-item publishing** — never all-at-once. Round-robin ≤N per tick.
  5. **The MQTT task does nothing but drain its queue** — discovery, NVS, reconnect side-work all go elsewhere (queue or boot).
  6. **Coredump-to-flash + boot-time check is infrastructure, not optional** — verify the full path (sdkconfig flags + partition + endpoint + boot alert) BEFORE you need it. Tool wants `-t raw` for BIN format.
  7. **Process: investigate → report → review → fix in separate branches/sessions.** Worked well; the earlier throwaway diagnostic branch rolled back cleanly because it was isolated.

**RAM optimization (RESOLVED): ~127 KB DRAM reclaimed; lwIP buffers are load-bearing.**
- Analysis report `docs/diag-ram-optimization.md` (report-only branch) ranked candidates by saving÷risk.
- **History buffers to PSRAM (~122 KB):** `s_fine_buf`, `s_coarse_buf` (in history_store AND handlers_history) plus `s_snap` in history_task were DRAM BSS. `EXT_RAM_BSS_ATTR` + `CONFIG_SPIRAM_ALLOW_BSS_SEG_EXTERNAL_MEMORY=y` moved them to PSRAM. This was the single biggest DRAM consumer and a hidden contributor to the original crash's DRAM tightness. DRAM free went from ~30 KB to ~173 KB; DRAM min-ever from ~2.6 KB to ~150 KB.
- **ctrl stack trim (~5 KB):** ControlTask 12 KB → 7 KB (observed peak ~3.5 KB + generous margin). Conservative; `main` left alone (small win).
- **lwIP TCP buffers — REVERTED:** reducing `LWIP_TCP_SND_BUF_DEFAULT`/`WND_DEFAULT` 5760→4096 caused panics and broke OTA. The buffers are load-bearing under large transfers (OTA, history JSON). ~7 KB saving was not worth the risk, especially next to the 122 KB win. Left at IDF default.
- Lessons:
  1. **`EXT_RAM_BSS_ATTR` fails SILENTLY without the active flag.** `CONFIG_SPIRAM_ALLOW_BSS_SEG_EXTERNAL_MEMORY` must be in the active sdkconfig, not just `sdkconfig.defaults`. The attribute is ignored with no error if the flag isn't set; the reclaim simply doesn't happen.
  2. **`sdkconfig.defaults` changes do NOT apply to an existing `sdkconfig.esp32s3` automatically.** Must regenerate: `rm sdkconfig.esp32s3 && pio run` (or a clean build). This bit us twice (coredump format, then PSRAM BSS). After any `sdkconfig.defaults` edit, regenerate and `grep` the active sdkconfig to confirm.
  3. **A new linker section (`.ext_ram.bss`) needs a clean build** — `_ext_ram_bss_start/end` undefined-reference errors mean the linker script wasn't regenerated. `rm -rf .pio/build/esp32s3` forces it.
  4. **lwIP TCP buffers are load-bearing; do not reduce below IDF default.** Reduction caused panics + OTA failure. DRAM is better reclaimed from history-in-PSRAM (122 KB) than from network buffers (~7 KB at high risk).

**Charts iteration (RESOLVED): timezone offset, drift metric, task-HWM table.**
- **Chart timezone (~6 h future offset):** `fine_epoch_base()`/`coarse_epoch_base()` were set once (when 0) and could never correct if seeded with a wrong/future time at first sample. The UI built x-positions as `t0_epoch + i*resolution`, so the whole chart shifted. Fix: anchor `t0_epoch` to `now_unix_s() - (count-1)*resolution` at render time, fallback to epoch_base only pre-NTP. Same fix for CSV export. (Note: topbar clock uses browser `Date()`, not server time — that's why topbar was correct while charts were off.)
- **Cell Drift as a chart metric:** added as a 5th selectable metric (Power/SOC/Voltage/Temp/Drift) in both chart tiles. Drift is not in the on-disk sample struct; rather than grow DRAM BSS, drift is held in PSRAM-heap companion arrays (`heap_caps_malloc(MALLOC_CAP_SPIRAM)`), +5.5 KB PSRAM / 0 DRAM. Ephemeral (lost on reboot) — acceptable for a chart. On-disk format unchanged.
- **Task-HWM table (Phase H3a deferral resolved):** the `/api/diag` `tasks[]` array now populates. Required `CONFIG_FREERTOS_USE_TRACE_FACILITY=y` + `CONFIG_FREERTOS_VTASKLIST_INCLUDE_COREID=y` + `CONFIG_FREERTOS_USE_STATS_FORMATTING_FUNCTIONS=y` AND the backend emitting the array (it wasn't). ~160 B DRAM cost. Sorted lowest-HWM-first. This was the H3a "deferred to V3.1" item — now done early because it was needed for RAM diagnosis.
- Lesson: a config flag added only to `sdkconfig.defaults` won't apply without regenerating the active sdkconfig (see RAM lesson 2 — same class).

**Board-Selector + power-cut + schema migration (RESOLVED).**
- **Board selector:** Waveshare ESP32-S3-RS485-CAN (default) + Manual. LilyGo T-CAN485 is NOT compatible (classic ESP32, 4 MB flash, no PSRAM) — V3 requires ESP32-S3 + 16 MB + 8 MB PSRAM. Manual mode covers ESP32-S3 boards with RS485/CAN HATs or transceiver ICs. Pin validation rejects reserved S3 GPIOs `{19,20,26-37,43,44,45,46}`, duplicates, and out-of-range. Per-interface present toggles (rs485_enabled; CAN via existing can_enabled). Changes apply on a controlled reboot. LED pin (GPIO38) is configurable but has NO driver in V3 yet (stored, inert).
- **Power-cut WiFi-wipe (the nasty one):** the 5×-rapid-reset → factory-wipe protection fired after only 1-2 power-cuts. Root cause: `record_this_boot()` recorded `esp_timer_get_time()` at call time (~105 ms) as the boot's "uptime" — there was never a mechanism to record the PREVIOUS boot's run length. So every POWERON/BROWNOUT (and every dev flash) deposited a ~105 ms entry; old entries + a couple power-cuts filled all 5 slots. Compounding: on ESP32-S3, `esp_reset_reason()` returns `ESP_RST_POWERON` for both a real power outage AND a hardware reset-button press — architecturally indistinguishable. Fix: POWERON/BROWNOUT excluded from the rapid-reset tally entirely; a successful run ≥30 s clears the ring; deliberate factory reset is UI-only now.
- **Field-preserving schema migration:** the board-selector bumped config schema v1→v2. Initial migration replaced old blobs with DEFAULT_CONFIG, wiping all user settings. Fixed to field-preserving: start from DEFAULT_CONFIG (valid new-field defaults), then overlay every field that existed in v1. `static_assert(sizeof(Config_v1) == sizeof(Config))` guards it at compile time (the new `rs485_enabled` bool fit into existing alignment padding, so size is unchanged and all fields ≥ offset 16 stay put). All 38 v1 fields carry forward.
- Lessons:
  1. **ESP32-S3 cannot distinguish a power outage from a reset-button press** (both `ESP_RST_POWERON`). Any power-cycle-gesture factory reset is unreliable; factory reset belongs in the UI. A rapid-reset protection must NOT count POWERON/BROWNOUT or it will wipe config on a power outage — fatal for a mains-powered device.
  2. **Schema migration starts from DEFAULT_CONFIG and overlays existing fields; never blindly set DEFAULT_CONFIG.** Otherwise every schema bump wipes user config. Guard total size with `static_assert` when new fields fit existing padding; reuse the "defaults then overlay" pattern for future versions.
  3. **A "rapid reset" counter needs a real previous-boot-uptime source**, not the time-of-call. If that can't survive a power-cut (RTC is wiped), exclude power-cut resets from the count rather than trusting a fake short uptime.

---

# BLOCK B — Neue Completion-Summaries + ersetzter Status

> §10.12 (Phase I) bleibt unverändert. Füge die folgenden Summaries danach ein, dann ersetze §10.13 komplett.

## 10.12b Phase J1+J2 — Pylontech + SMA CAN protocols

Added alternative inverter CAN targets alongside Victron. `src/can/pylontech.{h,cpp}` (6 frames) and `src/can/sma.{h,cpp}` (6 frames). Protocol routing in `tx.cpp` via `cfg.can_protocol` enum, read live per cycle. Settings → Battery radio selector (Disabled / Victron / Pylontech / SMA), integrating the `can_enabled` flag as the "Disabled" option. V2.67 source used as the encoding goldstandard. 176/176 host tests pass.

## 10.12c MQTT schema iterations — plain-text topics, per-pack, per-cell

The MQTT schema moved from JSON-blob topics to individual plain-text retained topics (one value per topic). The publish-level enum gained `PerPack` between `DataSystem` and `PerCell` (NVS migration shifts stored 3→4). Per-pack publishes 11 topics/pack every 5 s; per-cell publishes `cell_v_NN` per cell every 20 s. HA discovery models packs as sub-devices of the gateway (`via_device`); `sw_version` only on the gateway. The round-robin staggered scheduler (see §10.12d) later replaced the burst publishing.

## 10.12d MQTT-load panic — root-caused and fixed

The expanded MQTT load caused a panic every few hours. Root cause confirmed by coredump: memory corruption from a stack overflow (`StoreProhibited`, all TCBs corrupt). Fixed in 8 commits on `fix/mqtt-crash` (ArduinoJson static pools, `iv_topics` to BSS, staggered round-robin scheduler, HA discovery via queue, NVS cleanup to boot, race fixes, log_hook stack reduction, coredump endpoint + per-region heap observability). Verified 9h 44m+ stable. Investigation artifacts: `docs/diag-mqtt-crash-review.md` (9-finding report). The detailed lessons are in §10 R9.

## 10.12e RAM optimization — ~127 KB DRAM reclaimed

History buffers (`s_fine_buf`, `s_coarse_buf`, `s_snap`) moved from DRAM BSS to PSRAM via `EXT_RAM_BSS_ATTR` (~122 KB). ControlTask stack trimmed 12→7 KB (~5 KB). DRAM free went ~30 KB → ~173 KB; min-ever ~2.6 KB → ~150 KB. The lwIP TCP-buffer reduction was attempted and REVERTED (caused panics + OTA failure — buffers are load-bearing). Report: `docs/diag-ram-optimization.md`. Lessons in §10 R9 (silent `EXT_RAM_BSS_ATTR`, sdkconfig regeneration, lwIP buffers load-bearing).

## 10.12f Charts + Board-Selector + power-cut fix

- Chart x-axis timezone fixed (anchor `t0_epoch` to NTP time, not stale `epoch_base`).
- Cell Drift added as a 5th selectable chart metric (PSRAM companion arrays, 0 DRAM).
- Task-HWM table populated (the H3a V3.1 deferral, done early — trace-facility flags + backend array).
- Board selector (Waveshare / Manual, ESP32-S3 only, pin validation, reboot-on-save). LilyGo T-CAN485 excluded (incompatible: classic ESP32 / 4 MB / no PSRAM).
- Power-cut WiFi-wipe fixed (POWERON/BROWNOUT excluded from rapid-reset tally; factory reset UI-only).
- Field-preserving config schema v1→v2 migration (`static_assert` size guard; defaults-then-overlay).

---

## 10.13 V3.0 status (REPLACES the existing §10.13)

All core phases merged on `develop`. V3.0-beta is functionally complete and stable in real-world operation (3 packs, MQTT/HA, OTA, charts).

- **A-D:** Skeleton, BMS protocol, control task, safety
- **E:** Victron CAN broadcast
- **F-G:** Web server, dashboard, captive portal, auth
- **H1-H3c:** MQTT + HA discovery, history + charts + energy + NTP, alerts + diag, BMS detail, network tab + settings overhaul
- **I:** OTA with self-test rollback
- **J1+J2:** Pylontech + SMA CAN protocols
- **Post-J stabilization (merged):** MQTT plain-text/per-pack/per-cell schema; MQTT-load panic fix (8 commits, coredump-confirmed); RAM optimization (~127 KB DRAM reclaimed); chart timezone fix; Cell Drift chart metric; Task-HWM diag table; board selector (Waveshare/Manual); power-cut WiFi-wipe fix; field-preserving schema migration

**Stability milestones:** MQTT-load panic eliminated (was every few hours → 9h 44m+ clean). DRAM headroom 5× improved (~30 KB → ~173 KB free). Power outage no longer wipes WiFi/config. Config survives schema bumps.

**What remains for V3.0.0 release:**
- *(dropped)* ~~V2.67 backup JSON migrator~~ — dropped per owner decision
- Drift chart persistence across reboots (optional; ~5.5 KB flash with format versioning) — under consideration
- Phase J4: brand palette refresh to Indigo/Cream (current Cream/Aubergine is V3.0-dev)
- Phase J5: final documentation pass (api.md, mqtt.md, safety.md to match shipped state); `pio test -e native` Catch2 dependency fix; edge-case sweeps; release tag `v3.0.0` on `main` with GitHub release + SHA-256 in notes
- Optional follow-ups: LED driver (pin is configurable but inert); board selector in captive portal (currently Settings-only); WebSocket (V3.1)

---

*Nachtrag-Ende. Nach Einarbeitung: dieses File verwerfen.*
