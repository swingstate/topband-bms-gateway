# Changelog

## V2.63 (2026-04-17)

### Bug Fixes
- **Arduino IDE auto-prototype compile errors fixed.** The IDE's auto-prototype generator references types (`TBFrame`, `TBDate`, `TBSysParam`, etc.) from functions declared anywhere in the file. In V2.61 these structs were forward-declared but the full definitions came later than the generated prototypes, causing errors like `'TBFrame' has not been declared` on some toolchain versions.
- Resolution: all 10 struct definitions (`BMSData`, `VictronType`, `TBFrame`, `TBDate`, `TBSysParam`, `TBAlarmInfoParsed`, `TBAnalogDiag`, `BMSCommStats`, `AlertEntry`, `AlertState`) consolidated into a single block immediately after the `TIMING & LIMITS` defines (line ~150), before any function or global variable declaration. Struct variables (`bms[]`, `victronData`, `g_bms_stats[]`, etc.) declared separately after WiFi/MQTT globals.
- `ALERT_RING_SIZE` and `ALERT_MSG_LEN` defines moved to the TIMING block so `AlertEntry` and `AlertState` structs can use them.
- `class WiFiManager;` forward declaration retained (inside `#ifdef MODE_SMART_WIFI`) because `WiFiManager.h` is only included within that block.

### No Functional Changes
- Identical runtime behavior to V2.61. All features from V2.61 (server-side alert ring buffer, mobile menu top, chart `hstart` + ResizeObserver, wide dashboard cards) intact.
- NVS layout unchanged. Direct OTA upgrade from V2.61 (or V2.60 via V2.61 migration path) works.

## V2.61 (2026-04-17)

### New Features
- **Server-side alert ring buffer**: Last 25 alerts persisted in NVS (`gateway` namespace, ~2.7 KB). Survives reboot, OTA, and power loss.
- Alert detection moved from browser to Core 1 (runs every 10s, rising-edge based to avoid spam).
- `GET /alerts` returns full list as JSON (newest first), `POST /alerts/clear` wipes RAM + NVS.
- Clear button on Alerts page now calls the server (was browser-only in V2.60).
- Alerts included in `/data` response, so dashboard restores alert history instantly on login.
- When NTP is unavailable, alert timestamps fall back to uptime (e.g. "2h 14m up").

### Architecture Changes
- `AlertEntry` struct (112 bytes) + `AlertState` for rising-edge tracking to prevent repeat emission.
- Throttled NVS save: every 60s if dirty, plus forced save on `/save`, OTA, and reboot paths.
- `historyFilled` counter added to chart history: distinguishes real 0 values from unwritten slots.
- `/data` now sends `hstart` (index where real data begins in the 192-point downsampled array).
- Client `drLine` trims leading unfilled slots using `hstart`, replacing the fragile "skip zeros" heuristic.

### UI Changes
- **Mobile sidebar moved to top** (was bottom in V2.60). Fixes Safari address-bar overlap and Chrome iOS late-load lag.
- `viewport-fit=cover` + `env(safe-area-inset-top)` respects iPhone notch / Dynamic Island.
- Mobile dashboard cards: SOC, Voltage, Energy today, and Cell drift span full width. Eliminates empty slots in 5-card rows.
- Card order in first `.mg` row adjusted (SOC, Power, Current, Voltage, Energy today) for cleaner mobile stacking.
- Active-page indicator bar flipped from bottom to top on mobile.

### Bug Fixes
- **Mobile chart render race**: `ResizeObserver` watches `.cp` containers, triggers redraw on late layout, Safari address-bar collapse, or tab switch.
- Two delayed redraws after first page load (300ms + 1500ms) catch late iOS layout events.
- Temperature chart empty on mobile after login: fixed by observer + delayed redraw.
- Power chart "ghost 0-line" before buffer fills: fixed by `hstart` trimming.
- Client-side `addAlert` function converted to no-op (dead code after server-side migration).

### Migration Notes
- History buffer: existing V2.60 data preserved. `hfill` key inferred as `HISTORY_LEN` on first V2.61 boot if `hdat` is present.
- Alert NVS keys are new (`a_cnt`, `a_hd`, `a_dat`). No migration needed.
- No breaking config changes. Direct upgrade from V2.60 via OTA supported.

## V2.60 (2026-04-16)

### New Features
- **Runtime pin configuration**: Board type and GPIO pins are now configured via the web UI (General > Hardware), no longer hardcoded. Three presets: Waveshare ESP32-S3, LilyGo T-CAN485, and Custom (manual pin entry).
- Pin config persisted in NVS, included in backup/restore.
- Active pin assignment shown in General > Device.
- Works on any ESP32 or ESP32-S3 board with RS485 and CAN transceivers.

### Architecture Changes
- All `#define` pin constants replaced with runtime globals (`g_pin_rs485_tx`, etc.)
- `applyBoardPreset()` function for Waveshare/LilyGo defaults
- `rs485SetTX()`/`rs485SetRX()` helpers replace compile-time `#if defined(BOARD_WAVESHARE)` checks
- NeoPixel deferred init with `setPin()` in setup()

### Bug Fixes
- Low-power display: dashboard status threshold lowered from 50W to 10W, BMS card threshold from 0.5A to 0.15A, runtime estimate from 0.5A to 0.2A. Fixes intermittent "Idle" display during 30-50W discharge.
- Only compile-time choices remaining: SD card support (LilyGo only) and WiFi mode

## V2.53 (2026-04-16)

### Bug Fixes
- Smart reboot polling: accepts any HTTP response (including 401) as "ESP is online"
- Dashboard caching: main page served with `Cache-Control: no-store`

## V2.52 (2026-04-16)

### New Features
- Login rate limiting (5 failed attempts trigger 60s lockout)
- Reboot reason logging (esp_reset_reason displayed in General > Device)
- mDNS support (access via topband-gateway.local)
- Per-BMS communication statistics (polls/ok/timeout/errors/spikes)
- MQTT alarm topic ({topic}/alarm, retained, fires on state changes)
- Rolling energy counters: today + 7-day + monthly (NVS-persistent, Battery tab)
- CSV history export (/export endpoint, button in General > Maintenance)
- Instant chart config save via AJAX (no reboot needed, syncs across devices)
- Smart reboot polling after OTA/save (replaces fixed timers)
- Extended alert system with severity tracking and clear function
- State-transition logging for safety events and temperature throttling

### UI Changes
- Glassmorphism sidebar layout with 5 pages
- BMS cards: auto-fit grid (up to 5 per row), 15s refresh throttle
- WiFi pill: color-only (no dBm text)
- Version number moved from header to General > Device
- Chart rendering: zero-gap handling, min/max shaded bands
- Energy section on Battery tab (today/week/month)

### Bug Fixes
- Chart race condition fixed (reqC1/reqC2 tracking)
- BMS diagnostics "Read" button DOM id collision fixed
- Password reset hint removed from login page

## V2.41 (2026-04-14)

- Glassmorphism sidebar UI (initial version)
- Cookie-based session authentication with SHA-256
- Auto-config from BMS system parameters (0x47)
- Settings backup/restore, OTA firmware update
- 48h chart history with NVS persistence

## Pre-V2.x

- Based on atomi23/Topband-BMS-to-CAN V1.25
- Tab-based UI, basic MQTT, Victron CAN output
