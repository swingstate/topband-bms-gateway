# HTTP API Reference — TopBand BMS Gateway V3.0.0

All endpoints are served on port 80 (plain HTTP). The web UI and API share the
same server instance.

---

## Authentication

Session-cookie authentication. Obtain a session cookie by `POST /api/auth/login`
and include it in all subsequent requests. Cookie name: `session`.

CSRF protection is enforced on all mutating (POST, DELETE) auth-protected endpoints.
The CSRF token is returned by `/api/health` and must be sent in the `X-CSRF-Token`
request header.

Unauthenticated requests to protected endpoints receive `401 Unauthorized`:

```json
{"error": "unauthorized"}
```

When `auth_enabled` is false (device not yet password-configured) some protected
endpoints are accessible without a session. Check `auth_enabled` in
`GET /api/health` at startup.

---

## Public endpoints (no authentication required)

### GET /api/health

Lightweight status check. Polled by the UI on every page load.

**Response** `200 application/json`:

```json
{
  "ok": true,
  "uptime_s": 3600,
  "auth_enabled": true,
  "version": "3.0.0 (a750b8d)",
  "ui_version": "3.0.0",
  "build": "Jun  4 2026 12:00:00",
  "mqtt": {
    "enabled": true,
    "state": "connected",
    "publish_ok": 1440,
    "publish_fail": 0,
    "publish_drops": 0
  },
  "wifi": {
    "connected": true,
    "rssi": -62
  },
  "now_ts_s": 1748995200,
  "ntp_synced": true,
  "free_heap_b": 180000,
  "free_psram_b": 7800000
}
```

`mqtt.state` values: `disabled`, `connecting`, `connected`, `disconnected`, `error`.

---

### POST /api/auth/login

Authenticate and obtain a session cookie.

**Request** `application/json`:
```json
{"password": "mysecret"}
```

**Response** `200` on success (sets `session` cookie + returns CSRF token):
```json
{"ok": true, "csrf": "<token>"}
```

`401` on wrong password. Rate-limited: repeated failures result in a temporary
lockout period.

---

### POST /api/auth/logout

Invalidate the current session cookie.

**Response** `200`: `{"ok": true}` and clears the `session` cookie.

---

### POST /api/auth/set_password

Set or change the dashboard password. On first boot (no password set), this
endpoint does not require a session. Once a password is set, a valid session is
required.

**Request** `application/json`:
```json
{"password": "newpassword"}
```

**Response** `200`: `{"ok": true}`

---

## Auth-protected endpoints

All endpoints below require a valid session cookie.

---

### GET /api/live

Live battery system state. Polled by the UI dashboard every 2.5 s.

**Response** `200 application/json` — large JSON object containing:
- Aggregated pack metrics: SOC, SOH, pack voltage/current/power, temperatures,
  cell min/max/drift, CVL/CCL/DCL, alarm flags, system message
- Per-pack array: each pack's online state, cell voltages, temperatures, SOC,
  alarms, communication statistics
- Safety state: factor_charge, factor_discharge, `lockout_flags` (0x01 charge
  disabled by protection, 0x02 discharge disabled by protection — drives the
  dashboard banner; distinct from a bare CCL/DCL of 0, which can also be the
  benign near-full SoC taper), `temp_alarm` (0x01 cold, 0x02 hot)
- Energy counters: today in/out, 7-day, monthly
- Runtime estimate

---

### GET /api/bms/\<id\>

Per-pack detail for pack `id` (0-based). Returns the full BMS snapshot including
individual cell voltages, temperatures, sysparam limits, and poll statistics.

**Response** `200 application/json` or `404` if `id` is out of range or pack is
not configured.

---

### GET /api/config

Read the current configuration as JSON. Password field is returned obfuscated.

**Response** `200 application/json` — full Config struct serialised to JSON.

---

### POST /api/config

Write configuration fields. Only fields present in the body are updated; omitted
fields retain their current values.

**Request** `application/json`: partial or full Config JSON object.

**Response** `200`: `{"ok": true}` on success. `400` with `{"error": "..."}` on
validation failure (field name and reason returned). Does not reboot automatically;
some changes (WiFi, CAN protocol) require a restart to take effect.

---

### POST /api/wifi

Legacy WiFi credential update (SSID + password in one call). Prefer
`POST /api/wifi/configure` for new integrations.

**Request** `application/json`: `{"ssid": "...", "password": "..."}`

**Response** `200`: `{"ok": true}`. Device queues a deferred restart.

---

### GET /api/wifi/status

Current WiFi connection state.

**Response** `200 application/json`:
```json
{
  "connected": true,
  "ssid": "MyNetwork",
  "rssi": -62,
  "ip": "192.168.1.42",
  "mac": "aa:bb:cc:dd:ee:ff"
}
```

---

### GET /api/wifi/scan

Trigger a WiFi scan and return results. Blocks for up to ~3 s while scan runs.

**Response** `200 application/json`:
```json
{
  "networks": [
    {"ssid": "MyNetwork", "rssi": -55, "auth": "WPA2"},
    ...
  ]
}
```

---

### POST /api/wifi/configure

Configure WiFi credentials (SSID + password). Queues a deferred restart.

**Request** `application/json`: `{"ssid": "...", "password": "..."}`

**Response** `200`: `{"ok": true}`

---

### POST /api/restart

Trigger a firmware restart. The device applies a short delay before rebooting
so the HTTP response can be sent first.

**Response** `200`: `{"ok": true}`

---

### GET /api/backup

Download the current configuration as a portable JSON backup file.

**Response** `200 application/json` with `Content-Disposition: attachment` header
and filename `topband-bms-<MAC4>-<YYYYMMDD>.json`. The file wraps the config under
a `config` key alongside metadata:

```json
{
  "_schema": 1,
  "_ts": "2026-06-04T12:00:00Z",
  "_firmware": "3.0.0 (a750b8d)",
  "_device": "A1B2",
  "config": { ... }
}
```

---

### POST /api/restore

Import a configuration backup. The body must be a backup JSON file produced by
`GET /api/backup`. Validates schema version and all field ranges before applying.

**Request** `application/json`: backup file contents (up to 32 KB).

**Response**:
- `200`: `{"ok": true}` — config applied, device queues restart.
- `400`: `{"error": "..."}` — malformed JSON, unsupported schema version, or
  validation failure. No config is written on failure; the device does not reboot.

---

### POST /api/factory_reset

Erase all NVS configuration and reboot into first-boot setup mode (captive portal).
**Irreversible without a backup.**

**Response** `200`: `{"ok": true}` (reboot follows immediately).

---

### POST /api/svc/ha/discovery/send

Force-publish all Home Assistant MQTT discovery payloads immediately, regardless
of whether they have been sent since last boot.

**Response** `200`: `{"ok": true}` or `400` if MQTT is not connected.

---

### POST /api/svc/ha/discovery/clear

Clear the firmware-version marker in NVS that gates HA stale-entity cleanup.
On next MQTT connect the cleanup tombstones are re-sent, then fresh discovery
messages are published.

**Response** `200`: `{"ok": true}`

---

### GET /api/history

Chart history data for the dashboard (48-hour ring buffers).

**Response** `200 application/json` — arrays of timestamped samples for power,
voltage, SOC, temperature, cell drift. Timestamps are Unix seconds (or 0 before
NTP sync).

---

### GET /api/history/export.csv

Download full history as CSV. Each row is one history sample with all channels.

**Response** `200 text/csv` with `Content-Disposition: attachment` header.

---

### GET /api/solar-day

Today's MPPT PV power curve (5-minute resolution, PSRAM ring persisted to
LittleFS across reboots). Points are watts; null marks slots without a valid
MPPT reading.

**Response** `200 application/json` with `t0_epoch`, `midnight_epoch`,
`resolution_s` and a `points` array.

---

### GET /api/drift

Per-cell voltage-band data for the Battery page Drift Details section. Per
online pack: live spread (`spread_now`), SoC-gated 5-day spreads at
top-of-charge and bottom-of-discharge (`toc_spread` / `bod_spread` with
`has_toc` / `has_bod`), the drift rate in mV/day computed between the first
and latest full-charge day (`drift_rate`, meaningful when `n_toc_days >= 2`),
first-full / first-empty window extremes, repetition tallies
(`ff_/fe_mode_idx`, `_days_won`, `_days_total`: which cell won the per-day
argmax how often), and per-cell band arrays (5-day, all-time, per-region
min/max in mV).

The completed-day ring behind this endpoint persists to LittleFS on every
UTC day commit, so reboots do not reset the trend.

**Response** `200 application/json` — chunked transfer.

---

### GET /api/diag

Full diagnostics snapshot: firmware version, build date, reset reason, uptime,
heap/PSRAM, MQTT counters, CAN counters, NTP state, task high-water marks,
coredump summary (if a coredump exists), and the last 200 lines of the log ring
buffer.

Also includes, for the Diagnostics page RS485 / Battery / CAN sections:

- `can.protocol` — active inverter protocol (`victron` / `pylontech` / `sma`).
- `battery` — the aggregated `SafetyState` fields that FEED the CAN encoders:
  `alarm_flags`, `temp_alarm` (raw bytes; decoded client-side), `packs_online`,
  `cvl_volts` / `ccl_amps` / `dcl_amps` / `dvl_volts` (0x351 limits),
  `factor_charge` / `factor_discharge`, and `sys_message`. Lets a discharge-lockout
  report be diagnosed from the UI alone (e.g. overvolt → DCL 0 → discharge-enable No).
- `packs[]` — per-pack diagnostics indexed by pack: RS485 comms
  (`polls`/`ok`/`timeouts`/`errors`/`success_pct`), `last_seen_age_ms`, `bms_id`,
  `soc`/`soh`, `cell_min_v`/`cell_max_v` (+ indices), `drift_mv`, raw `alarm_bits`,
  and (when `sysparam_valid`) `sys_charge_max_a`/`sys_discharge_max_a`/`sys_cell_high_v`.

**Response** `200 application/json` — chunked transfer (large response).

---

### GET /api/diag/coredump.bin

Download the raw coredump binary from the coredump partition. Only available if a
coredump was captured after the last reboot.

**Response** `200 application/octet-stream` (coredump binary) or `404` if no
coredump exists. `500` if the coredump partition is absent in the partition table.

---

### GET /api/alerts

Read the alert ring buffer (last 25 alerts, persisted across reboots in NVS).

**Response** `200 application/json`:
```json
{
  "alerts": [
    {
      "ts_ms": 1748995200000,
      "ts_human": "2026-06-04 12:00:00",
      "event": "CellOvervoltStart",
      "bms_id": 0,
      "message": "Cell overvolt on pack 0"
    },
    ...
  ],
  "count": 3
}
```

---

### DELETE /api/alerts

Clear the alert ring buffer (NVS and in-memory).

**Response** `200`: `{"ok": true}`

---

### POST /api/ota/upload

Upload a firmware binary for OTA update. The binary must be a valid ESP-IDF OTA
image for the `app` partition. After a successful flash the device runs the new
firmware; on self-test failure it rolls back automatically to the previous image.

**Request** `application/octet-stream` — raw binary body (up to partition size,
typically ~2 MB).

**Response**:
- `200`: `{"ok": true}` — flash successful, device reboots.
- `400`: `{"error": "..."}` — image rejected (wrong chip, partition error).
- `500`: internal flash error.

---

### GET /api/ota/status

OTA subsystem state and rollback status.

**Response** `200 application/json`:
```json
{
  "state": "idle",
  "rollback_possible": false,
  "running_partition": "app0",
  "next_partition": "app1"
}
```

`state` values: `idle`, `uploading`, `flashing`, `rebooting`, `error`.

---

### POST /api/mqtt/test

Trigger a test MQTT publish (a single retained message on `{base}/test`).
Used by the Settings → Network → MQTT test button to verify broker connectivity.

**Response** `200`: `{"ok": true, "msg_id": 42}` or `400` if MQTT is disabled or
not connected.

---

### GET /api/mqtt/test

Retrieve the result of the last MQTT test publish.

**Response** `200 application/json`:
```json
{
  "last_result": "ok",
  "last_ts_ms": 1748995200000
}
```

`last_result`: `"ok"`, `"fail"`, or `"none"` (no test run yet this session).

---

### POST /api/notify/telegram/test

Send a test Telegram notification to the configured chat/bot.

**Response** `200`: `{"ok": true}` on success. `400` if Telegram is not configured.
`502` if the send failed (HTTP error from Telegram API).

---

### GET /api/notify/telegram/test

Retrieve the result of the last Telegram test send.

**Response** `200 application/json`:
```json
{
  "last_result": "ok",
  "last_ts_ms": 1748995200000,
  "last_error": ""
}
```

---

### GET /api/notify/status

Current notification provider status.

**Response** `200 application/json`:
```json
{
  "telegram": {
    "enabled": true,
    "verified": true,
    "chat_id_set": true
  }
}
```

---

### GET /api/notify/alert-types

List all alert event types that can be enabled for notifications, with their
current enabled state.

**Response** `200 application/json`:
```json
{
  "alert_types": [
    {"key": "BmsWentOffline",      "enabled": true},
    {"key": "CellOvervoltStart",   "enabled": true},
    {"key": "PackOvervoltStart",   "enabled": true},
    ...
  ]
}
```

---

## Static file serving

### GET /\*

All requests not matching an API route are served from the LittleFS UI bundle
(`/lfs/ui/`). The server performs path → LittleFS lookup and serves the file with
the appropriate MIME type.

Cache-busting: the UI HTML references assets with a `?v=<UI_VERSION>` query
parameter (e.g. `?v=3.0.0`). The server strips the query string before the
filesystem lookup, so asset URLs are cache-safe across upgrades.

Unauthenticated browser requests to the dashboard are redirected to
`/login.html`. The captive portal (AP mode) serves `/setup.html`.
