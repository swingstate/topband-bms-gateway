# TopBand BMS Gateway Roadmap

**Last updated:** 2026-04-20
**Current release:** V2.66.3 (shipped, soak continuing)

Living document. Each version has scope, breaking-changes, and open questions. Versions flow top-to-bottom in release order. Abhängigkeiten explizit markiert.

---

## Near-term: V2.65.x hotfix stream

OTA-compatible, backend fixes only, no partition changes.

### V2.65.4 — shipped 2026-04-18

Architectural decision: chart history leaves NVS permanently until V3.0. Details in CHANGELOG.md.

**Status:** Validated stable. NVS usage flat at ~305 entries over 10h observation.

### V2.65.5 — retired

Originally scoped as TASK_WDT hotfix. Scope absorbed into V2.66 per user direction.

---

## V2.66 — shipped 2026-04-18

**Status:** Shipped, code review complete (see CODE_REVIEW_V2.66.md). Soak phase continues: watching for TASK_WDT recurrence, hold-last-value effectiveness, and NVS entry growth stabilization.

Shipped scope:
- **W1** TASK_WDT reboots fixed (esp_task_wdt_reset() at critical points in loop, WDT_TIMEOUT 30→60s)
- **H1** BMS current hold-last-value with 120s window, per-BMS counter in /data
- **H2** Session expiry server-side enforcement, 30-day cap, auto-rotation on expiry
- **M4** Configurable spike filter thresholds (Battery tab UI, NVS-persisted)
- **M6** /data rate-limit (8-slot IP token bucket, 4 burst, 2/sec refill)

### Code review summary (2026-04-18)

25 findings across the file. All high/medium items routed into V2.66.1 / V2.67 / V3.0 below. Executive summary and full detail in `CODE_REVIEW_V2.66.md`.

### Soak validation targets

- No TASK_WDT reboots over 48h of mixed Web UI use + normal operation
- `current_holds` counter climbs during low-load evening periods (confirms H1 fires)
- Dashboard current reading no longer flickers between real value and 0.00 A
- Energy counters accumulate correctly during 40-60W discharge periods (was undercounting in V2.65.4)
- Rate-limit counter stays at 0 during normal use (dashboard polls every 2.5s)
- NVS entry count stabilizes around 400 entries or below (rules out structural growth; see M4 finding)

---

## V2.66.1 — hotfix (scope expanded after field observation 2026-04-18 21:54:05)

OTA-compatible. Originally three findings from the review; C3 added after reproducible TASK_WDT reboot on Web UI page refresh. Ships ASAP.

### Scope

- **C3 (top priority)** Fix `/data` streaming TASK_WDT trigger. Add `streamAlive()` helper with `server.client().connected()` check and `esp_task_wdt_reset()`. Insert calls at ~5 strategic points in the streaming block. Handler aborts cleanly on disconnect.
- **C1** Fix `xSemaphoreGive()` without confirmed `Take` on three sites (`sendMqttData` line 2572, `handleData` line 3288, `rs485Task::calculateVictronData` line 4967). Capture `Take` result in local `bool locked`, enter critical section only if true, `Give` only if true. Use `alertDetectTick` at line 2263 as template.
- **C2** Fix H1 hold-last-value cross-core read at lines 1709 and 1712. Capture current and validity under the mutex block at line 1685, use captured locals instead of re-reading `bms[addr]`.
- **H1** Fix M6 rate-limit LRU loop short-circuit at lines 3238-3250. Continue scanning all 8 slots for existing IP match; only fall back to empty/LRU slot if no match found.

### Ship criteria

- C3 validated: refresh + new tab + normal operation 30 min without TASK_WDT
- C1/C2/H1 validated via simulation mode
- No new NVS keys, no breaking changes, no Web UI changes
- OTA push, then 24h soak

### Field observation reference

2026-04-18 21:54:05 boot log: heap 144 KB, NVS 320/630 (stable), H1 firing on BMS0 + BMS1 within 40 s. Boot reason TASK_WDT. Repro: page refresh on dashboard + new tab. C3 finding added to review doc.

---

## V2.66.2 — C3 hardening + diagnostics (shipped 2026-04-19)

OTA-compatible. Scope expanded after reproducible TASK_WDT on V2.66.1 page-refresh (same C3-class trigger, C3 V2.66.1 fix was insufficient) plus unexplained WDT reboot at 6h30m with no log retention.

### Scope

- **C3+ Hardening via global abort flag.** `streamAlive()` sets `g_stream_abort`, every `sendHist()` call in `handleData` checks it after return and exits handler. Resolves the lambda-return-doesn't-propagate issue documented in V2.66.1 delivery notes.
- **E1** Per-BMS abort check inside BMS array loop (gate after each `sendContent(c)`).
- **E2** `Cache-Control: max-age=600` on `/` main UI instead of `no-store`. Browser caches the HTML shell, live data still fetched via `/data`. Reduces gateway load on page-refresh significantly.
- **D1** MQTT Diagnostics Topic `{base}/diag`, 30s cadence, retained for post-mortem. 21 keys total including 4 new C3+ observability counters: `stream_aborts`, `handler_max_ms`, `loop_max_ms`, `wdt_warnings`. Off-by-default.
- **AD-001** Architecture Decision Record added to ROADMAP explaining why V2.66.x C3 bandaids are the correct response and why root-cause fix (Async WebServer) is scoped to V3.0.

### Post-ship diagnostic strategy

Enable diagnostics, configure Mosquitto persistence, let run 24-48h. On next reboot the retained diag-snapshot on broker shows:
- `stream_aborts` climbing → C3+ is firing, page-refresh now handled correctly
- `loop_max_ms` climbing toward 45000 → something in Core 1 is getting slower over time
- `wdt_warnings` non-zero → near-WDT events (use `handler_max_ms` to identify culprit)
- `can_tx_fail_streak` climbing → CAN bus or inverter issue (hypothesis from 8h WDT reboot)
- `nvs_used` near cap (>600) → NVS fragmentation / GC stall
- `rs485_hwm` dropping toward 0 → Stack pressure on Core 0

Without this data, V2.66.x residual reboots stay unsolvable.

---

## V2.66.3 — HA Auto-Discovery for diag topic (shipped 2026-04-20)

OTA-compatible. Complements V2.66.2 D1 by surfacing the 21 diagnostic keys as Home Assistant entities without manual YAML configuration.

### Scope

- **D2** `sendMqttDiagDiscovery()` publishes 21 retained discovery configs to `homeassistant/sensor/{uid}_diag_{key}/config` when `mq_diag` + `ha_en` both active. Entities grouped under existing Topband BMS device. Correct device_class / state_class assignments so HA renders graphs and statistics natively.
- **D2** `removeMqttDiagDiscovery()` cleans up discovery configs on toggle-off.
- **Bugfix** Duplicate JSON key `rl_rejects` in `/data` response removed (pre-existing V2.66 copy-paste artifact).

### Ship criteria

- HA Integration page shows 21 new sensors under Topband BMS device after enable+save
- Boot reason, uptime, heap trends visible in HA history graphs
- Toggle off → sensors disappear within seconds in HA

---

## V2.67 — cleanup and hardening (planned)

OTA-compatible. Batch of medium-severity items from the code review plus any deferred V2.66 scope. Target: 4-6 weeks after V2.66 soak completion.

### Scope

#### Diagnostics (follow-up from V2.66.2)

- **D1 follow-up** Home Assistant auto-discovery for the `{base}/diag` topic. 17 entities registered via retained MQTT config messages. Deferred from V2.66.2 to let the real-world diagnostic stream inform which keys need HA entities vs. remain broker-only.

#### Thread-safety and mutex discipline

- **H3** Move `sendVictronCAN()` inside the `dataMutex` block in `rs485Task`, or snapshot the 8 victronData fields into locals before releasing. Prevents future torn reads into CAN frames.
- **H4** Change `sendMqttData` to match the `handleData` pattern: snapshot `victronData`, `bms[]`, `energy_*`, `sys_error_msg` into locals under the lock, release, then build JSON from locals. Eliminates 10-50 ms mutex hold during JSON assembly.
- **H5** Guard the pre-NTP session re-anchor NVS write with a per-boot flag so concurrent `checkAuth` calls skip the duplicate write.
- **L2** Include `g_runtime_cvl`, `g_runtime_proto_ccl_cap`, `g_runtime_proto_dcl_cap` in the `/data` snapshot block at line 3288.
- **M10** Take `dataMutex` around `writeLogToSD()` field reads, or snapshot into locals under the lock.

#### Security

- **H2** Switch login rate-limiter from global to per-IP. Reuse the M6 8-slot structure or add a dedicated 4-slot table.
- **M7** Fix `login_fails` post-lockout loophole. Either keep the counter monotonic with doubling backoff (60s, 120s, 240s), or apply IP-level bans after N lockouts in a rolling window.

#### Watchdog coverage gaps

- **M1** Add `esp_task_wdt_reset()` after `writeLogToSD()` in the main loop.
- **M2** Add `esp_task_wdt_reset()` after `alertDetectTick()` in the main loop.
- **M3** Add `esp_task_wdt_reset()` after `Update.begin()` in the OTA upload handler.

#### NVS hygiene

- **M4** Increase `saveAlertsToNvs` throttle from 60s to 5-10 minutes. Reduces NVS wear amplification from sustained alert activity.

#### Robustness

- **M9** Raise RS485 RX buffer `reserve()` from 512 to 600 bytes, or add explicit length cap with early break at 580 bytes.
- **L3** Log `mqtt.publish()` failures via `addToLog` for the periodic status/data/alarm topics. Increment a counter exposed in `/data`.

### Ship criteria

- No breaking changes, no NVS schema changes, no UI restructure
- OTA push, then soak validation

### Open questions

- V2.67 could be split into V2.67 (thread-safety) and V2.68 (security + WDT) if scope feels large. Decide once starting implementation.
- M8 (constant-time credential compare) feasible here too if deemed worth the extra risk of auth path changes. Otherwise parking lot.

---

## V3.0 — refactor milestone (breaking, USB reflash required)

**Target:** After V2.66 soak stable AND V2.67 shipped. Groups ALL breaking changes into a single one-time flash-and-reconfigure event to minimize user pain.

### Why a milestone release

Changing partition table requires `esptool.py erase_flash` + USB cable. We want this to happen exactly once for the life of the project, not multiple times. Everything that needs a new partition layout or breaks persistence format goes here.

### Scope

#### Architecture (mandatory)

- **Async WebServer migration (ESPAsyncWebServer).** Replaces the synchronous Arduino `WebServer`. Root-cause fix for the TASK_WDT-on-page-refresh class of bugs that V2.66.1 C3 and V2.66.2 C3-hardening only patched over. Async handlers are event-driven: client disconnect automatically aborts the handler, no manual gate instrumentation needed. Chunked responses work via callback chunks filled on demand. Expected impact: eliminates entire WDT-risk class around HTTP streaming. Effort: ~600 lines rewrite of all handlers; HA/MQTT behavior unchanged from subscriber perspective.
- **Multi-file source split.** Single 5000-line `.ino` → multiple `.cpp`/`.h` files. Target organisation:
  - `main.ino` (setup, loop, glue)
  - `protocol.cpp` — TopBand RS485 parsing
  - `web.cpp` — HTTP handlers
  - `mqtt.cpp` — MQTT + HA discovery
  - `alerts.cpp` — alert ring buffer
  - `nvs.cpp` — settings persistence
  - `can.cpp` — CAN TX logic
- **New partition table.** Adds LittleFS partition (~256-512 KB). NVS stays at 24 KB.
- **NVS schema versioning.** Adds `schema_ver` key. Migration runner on boot.

#### Persistence (via new LittleFS partition)

- **Chart history back on disk.** Replaces V2.65.4 RAM-only fallback. Single `/hist.bin` file, atomic rename pattern. Wear-leveling + CRC built into LittleFS, no more partial-save failures.
- **Web UI files served from LittleFS.** `index.html`, `app.css`, `app.js` as separate files. UI changes no longer require firmware rebuild. Synergizes with Async WebServer (zero-copy static file serving).

#### Code quality (synergies with rewrite)

- **ArduinoJson for response serialization.** Replaces String-concatenation in all handlers. Type-safe, escape-safe, bounded-memory builds. Addresses M6 heap-churn finding at the root instead of snprintf-patching per site. Mandatory for Async handlers anyway (lambda captures require stable buffers).

#### Security (recommended)

- **OTA signature verification.** RSA-signed firmware images. Rejects unsigned uploads.

#### Tech debt from V2.66 code review

- **M5** Gate all 9 unconditional `Serial.println` call sites behind `g_serial_debug`.
- **M6** Replace String concatenation in `sendMqttData` with `snprintf` into pre-allocated char buffer. Pattern already established in `g_can_status` and alert message handling.
- **L1** Replace magic epoch `1672531200` with named constant `NTP_EPOCH_MIN`.
- **S1** Resolve `g_expert_mode` / `g_easy_mode` confusion. Either remove "Deprecated" comment and document actual behavior, or consolidate into a single mode enum with migration.
- **S2** Run clang-format with 2-space canonical across all files as part of the multi-file split.
- **S3** Introduce `addToLogf(const char *fmt, ...)` variant; migrate 83 hot-path call sites from String concatenation.

### Breaking changes (all users affected)

- First install of V3.0 requires USB reflash with `esptool.py erase_flash`
- All settings reset on upgrade (backup settings before flash, restore after)
- Chart history continues to fresh after V3.0 reflash, then persists normally
- OTA from V2.x to V3.0 NOT supported. After V3.0 is installed, OTA works again for V3.x.

### Dependencies

- V2.66 must be stable first (validates V2.65.x architectural decisions)
- V2.67 shipped (otherwise the thread-safety fixes need to be re-done during the split and land unreviewed)
- V2.66 open questions answered (informs abstraction design for modules)

### Effort estimate (revised 2026-04-19)

Original V3.0 scope: ~4 weeks (multi-file split + LittleFS + schema versioning).
With Async WebServer + ArduinoJson added: **~2-3 months**. Rationale: every HTTP handler needs rewrite, and testing coverage must extend to disconnect-mid-stream scenarios across the full surface.

This is the correct investment. V2.66.x C3 class bugs are not fixable architecturally without this step. Each V2.66.x hotfix is a bandaid on an inherent limitation of the synchronous `WebServer` library.

### Open questions (to decide during V3.0 design phase)

- Partition sizes: how much LittleFS? 256 KB, 512 KB, 1 MB?
- Upgrade path: provide pre-built erase+flash script for users?
- SPIFFS (if anyone has it on a LilyGo setup) migration?
- NVS partition bump from 24 KB to 32 KB while we're at it?

---

## V3.1 — Victron MPPT via Bluetooth LE

**Dependencies:** V3.0 shipped (needs multi-file split + LittleFS for BLE state persistence).

### Motivation

User has a Victron 150/35 MPPT charger. Reading its BLE advertising gives solar yield, PV power, charge phase, battery voltage (cross-check with BMS). Makes the gateway a true energy-monitoring device, not just a BMS bridge.

### Scope

- **BLE scanner module** (`ble_victron.cpp`) using ESP-IDF NimBLE
- **Victron Instant Readout decrypt** via AES-CTR with per-device encryption key
- **MPPT data source** — PV voltage, PV current, PV power, battery charge current, charge state, today's yield
- **Settings UI** for BLE device discovery + encryption key entry
- **New dashboard card** for solar / PV live values
- **Fifth chart type** for PV power history (alongside existing Power/Voltage/SOC/Temperature)
- **MQTT + HA discovery** for solar entities

### Memory + flash budget

- BLE stack: ~30 KB heap when active, ~80 KB flash
- UI additions: ~5 KB HTML/CSS/JS
- LittleFS data files: ~1 KB per discovered device (keys + last-known state)

### Open questions

- Single MPPT only in V3.1, or multi-device support from day one?
- Coexistence with WiFi: test under load how much WiFi throughput degrades
- Fallback when BLE device out of range: hide UI or show stale data with warning?

---

## V3.2 — Victron SmartShunt integration (strategic)

**Dependencies:** V3.1 shipped (shares BLE infrastructure).

### Motivation

This is the big one architecturally. Current problem: TopBand BMS only reports current samples every ~90 seconds (confirmed via Frame Spy in V2.65.1 diagnostics). This forces the Hold-Last-Value workaround in V2.66 and causes energy undercounting for low loads.

SmartShunt provides 1-Hz precision current measurements via BLE. If a SmartShunt is present, it becomes the **ground truth** for current/SOC, and the BMS role shrinks to "cell voltages + temps + alarms only".

### Scope

- **SmartShunt BLE module** (extends `ble_victron.cpp` with SmartShunt protocol variant)
- **Data fusion module** (`data_fusion.cpp`): priority-based source selection
  - Current: SmartShunt (if online) > BMS aggregate
  - SOC: SmartShunt (if online) > BMS reported
  - Cell voltages, temps, alarms: BMS (always)
  - Pack voltage: BMS (always)
- **Dashboard UX**: show "Source: SmartShunt" badge next to current reading when active
- **MQTT**: extra fields for shunt-derived values
- **Victron CAN TX**: uses SmartShunt data for Pylontech/Victron current/SOC if available (more accurate inverter-side display)

### Architectural upside

- V2.66's Hold-Last-Value becomes the fallback path for BMS-only users
- Energy integration gets 100% accurate for users with SmartShunt
- SOC display stops bouncing during low-load periods

### Open questions

- Do we notify user if shunt goes offline during runtime? Alert, or silent fallback?
- SmartShunt-derived SOC can disagree with BMS-reported SOC — how to surface the delta in UI without being confusing?

---

## V3.3+ — incremental features

**Not scheduled. These are ideas in the parking lot.**

### Candidates (priority TBD)

- **Auto-recovery from safety lockout.** Hysteresis-based reset for overvoltage/undervoltage/temperature cutoffs. Currently only clears on manual power-cycle.
- **WiFi fallback AP mode.** If primary WiFi drops for >60s, start AP portal for re-config.
- **BMV-712 support.** Similar to SmartShunt but older battery monitor.
- **Multiple SmartShunts.** For systems with separate battery banks.
- **Home Assistant extended discovery.** Per-cell voltages as HA entities (optional, behind toggle).
- **Configurable MQTT publish rate.** Currently hardcoded at 5s.
- **Session max-age configurable in UI.** Currently fixed at 30 days via V2.66 H2.
- **InfluxDB / Prometheus export.** Alternative to MQTT for users with time-series DB.
- **Firmware rollback.** If new OTA fails to boot within 60s, revert to previous app partition.
- **Event log persistence.** Safety events, state transitions, long-lived on LittleFS for forensics.
- **M8 Constant-time credential comparison.** Replace `String ==` with `mbedtls_ct_memcmp`. Low priority on LAN, cryptographic hygiene item.
- **L4 Verify rem_ah offset against Topband spec.** The `p += 3` for a 2-byte field at lines 1005 and 1654 is likely a spec-required reserved byte skip, but has no inline documentation. Confirm against Topband protocol spec and add comment, or fix if wrong.
- **L5 CSRF token on POST handlers.** SameSite=Strict cookie already mitigates most cases. Add double-submit token for belt-and-suspenders defense.

### Retired / rejected ideas

- **NVS partition to 256 KB** — rejected as architectural anti-pattern. LittleFS is the right tool for repeated large writes.
- **BMS firmware update via gateway** — TopBand doesn't expose this interface, out of scope.
- **Cell balancing override commands** — safety liability, not worth the risk.

---

## Open backlog (style / tech debt)

Most items from the original V2.64 code review are now mapped into the V3.0 section above. Remaining unscheduled items:

- Magic numbers duplicated between backend JSON emission and frontend parsing
- Copyright year not in source header
- No NVS schema versioning yet (V3.0 will fix)

These land opportunistically during V3.0 multi-file split, not as a dedicated release.

---

## How this document stays alive

1. When a release ships, move its "Target" status to "Shipped" with date
2. When open questions get answered, move them into the scope as decisions
3. Add new items to V3.3+ parking lot as they come up, let priority sort itself later
4. Every quarter or after major release, review dependencies and reshuffle if needed

**File ownership:** maintainer keeps this in sync with CHANGELOG.md. CHANGELOG is what happened, ROADMAP is what's coming.

---

## Architecture decisions (context for future reviews)

Historical record of architectural choices that shape the roadmap. Each entry is a decision that was not obvious, with the reasoning for why we picked one path over another.

### AD-001 — Arduino WebServer reaches its limit with streaming-heavy UIs (2026-04-19)

**Context:** V2.66 shipped with a streaming `/data` JSON endpoint that emits ~60-80 chunks via `server.sendContent()`. V2.66.1 added C3 gates with `streamAlive()` helper. V2.66.1 still rebooted on page-refresh (field-observed). V2.66.2 hardens C3 further with a global abort flag.

**Root cause:** Arduino `WebServer` is synchronous. A handler blocks Core 1 until it returns. Client disconnect mid-response is not propagated by the library; the handler continues calling `sendContent()` into a dead socket, each call stalling up to TCP retransmit timeout. Accumulated stalls exceed WDT window.

**Decision:** Continue with bandaid patches on V2.66.x, but add `ESPAsyncWebServer` migration to V3.0 as mandatory scope. Async library handles disconnect automatically via event callbacks. This is the root-cause fix.

**Why not earlier:** Async migration is ~600 lines of handler rewrite. Not appropriate as a hotfix. V3.0 is already a breaking release (partition table change for LittleFS), so the handler rewrite lands in the same reflash event.

**What this means for reviewers:** C3-class findings in V2.66.x code are expected residual risk until V3.0 ships. Do not try to chase them in V2.66.x beyond reasonable gate placement.

### AD-002 — NVS is configuration storage, not time-series storage (2026-04-18)

**Context:** V2.65.x wrote chart history to NVS. Observed linear NVS entry growth that would exhaust the 24 KB partition within days. Each save marked ~123 entries invalid and wrote fresh copies, stressing the garbage collector.

**Decision:** V2.65.4 removed chart history from NVS permanently. RAM-only until V3.0 ships LittleFS. Energy counters (~15 entries) stay in NVS as they are truly configuration-like.

**Why:** NVS is designed for infrequent config writes with wear-leveling optimized for that pattern. Time-series data needs a log-structured store, which LittleFS provides.

**What this means for reviewers:** Any new persistent data should ask first whether it is configuration (goes to NVS) or history/telemetry (waits for LittleFS in V3.0, RAM-only until then).

### AD-003 — V2.66.x hotfix stream is intentionally conservative (2026-04-19)

**Context:** Review uncovered 25 findings. Field observations added C3. Temptation exists to ship comprehensive V2.67 earlier with everything fixed.

**Decision:** V2.66.x hotfixes stay small and targeted. Each addresses one specific reproducible problem. V2.67 batches quality items without time pressure.

**Why:** Mixing C-severity hotfixes with M-severity quality improvements means every regression becomes harder to bisect. Small hotfixes deploy with confidence; big releases need soak time.
