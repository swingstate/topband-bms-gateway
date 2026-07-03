# MQTT Reference — TopBand BMS Gateway V3.0.0

---

## Base topic

All topics use an effective base derived at runtime:

```
{cfg.mqtt_base_topic}-{last4hexMAC}
```

Example: configured base `topband-bms`, MAC ending `a1b2` → effective base
`topband-bms-a1b2`. Multiple gateways on the same broker cannot collide.

---

## Topic model: plain text per entity

V3.0 publishes **one value per retained topic** (plain text). Each metric has
its own topic; no JSON parsing is required on the subscriber side. This is the
primary model used by Home Assistant auto-discovery.

A legacy JSON `/data` blob is also published every 5 s alongside the individual
topics (not retained) to preserve compatibility with custom integrations built
against V2.67.

---

## Publish levels

Set via `mqtt_level` in configuration (Settings → Network → MQTT):

| Level | Name | What is published |
|---|---|---|
| 0 | Off | Nothing (MQTT disabled entirely) |
| 1 | StatusOnly | LWT status topic only |
| 2 | DataSystem | + 20 plain-text system topics, JSON `/data`, `/alarm` |
| 3 | PerPack | + per-pack plain-text summary topics for each pack |
| 4 | PerCell | + individual cell voltage topics + JSON cells blob per pack |

`mqtt_diag_enabled` is an independent flag that adds the `/diag` JSON topic at
30 s cadence regardless of level (useful for dashboard-free monitoring).

---

## Topic table

### Always-on (when MQTT is connected)

| Topic | Retained | Cadence | Content |
|---|---|---|---|
| `{base}/status` | yes | on connect / broker LWT | `online` or `offline` |

### Level 2+ (DataSystem) — plain-text system topics

All retained. Published by rotating through 4 topics per 1 s tick; each topic
refreshes every ~5 s.

| Topic | Value type | Example |
|---|---|---|
| `{base}/soc` | integer % | `75` (V3.2: shunt-fused bank SOC when `ble_shunt_enabled` and fresh, else BMS mean — see below) |
| `{base}/voltage` | float V (2 dp) | `52.40` |
| `{base}/current` | float A (1 dp) | `10.5` |
| `{base}/power` | integer W | `550` |
| `{base}/temperature` | float °C (1 dp) | `25.2` |
| `{base}/cell_v_min` | float V (3 dp) | `3.324` |
| `{base}/cell_v_max` | float V (3 dp) | `3.327` |
| `{base}/cell_drift` | float V (3 dp) | `0.003` |
| `{base}/soh` | integer % | `98` |
| `{base}/cvl` | float V (1 dp) | `56.0` |
| `{base}/ccl` | integer A | `80` |
| `{base}/dcl` | integer A | `80` |
| `{base}/alarm_flags` | integer bitmask | `0` |
| `{base}/sys_message` | string | `OK` |
| `{base}/bms_online` | integer | `2` |
| `{base}/bms_configured` | integer | `2` |
| `{base}/energy_today_in` | float kWh (2 dp) | `3.52` |
| `{base}/energy_today_out` | float kWh (2 dp) | `1.84` |
| `{base}/runtime_est_min` | integer min | `240` |
| `{base}/runtime_est_state` | string | `until_empty` |

`runtime_est_state` values: `until_empty` (discharging), `until_full` (charging),
`idle`.

`alarm_flags` bitmask (byte-identical to V2.67 CAN alarm byte):
- `0x02` pack/cell overvolt
- `0x08` temperature stop (charge or discharge cutoff)
- `0x10` pack/cell undervolt
- `0x20` cell drift / imbalance warning
- `0x40` BMS reported critical alarm via 0x44 frame
- `0x80` no packs online

### Level 2+ — JSON `/data` blob (legacy, not retained)

Published every 5 s. Topic: `{base}/data`.

```json
{
  "ts_ms": 1748995200000,
  "uptime_s": 3600,
  "bms_count_online": 2,
  "bms_count_configured": 2,
  "soc_avg": 75,
  "soc_display": 73,
  "soc_source": "shunt",
  "soh_avg": 98,
  "pack_voltage_avg": 52.4,
  "pack_current_total": 10.5,
  "pack_power_w": 550.2,
  "temp_avg": 25.2,
  "temp_max": 27.1,
  "cell_v_min": 3.324,
  "cell_v_max": 3.327,
  "cell_v_drift": 0.003,
  "cvl_v": 56.0,
  "ccl_a": 80.0,
  "dcl_a": 80.0,
  "alarm_flags": 0,
  "sys_message": "OK"
}
```

`cell_v_min/max/drift` and `temp_max` are `null` when no packs are online.

`soc_avg` is the pure BMS mean (byte-identical to V2.67). `soc_display` (V3.2) is
the bank SOC actually shown on the dashboard and published to `{base}/soc` — the
SmartShunt reading when `ble_shunt_enabled` and fresh, else equal to `soc_avg`.
`soc_source` is `"shunt"` or `"bms"`, whichever fed `soc_display` this cycle.
Selection policy: `Config::soc_mode` (`Calculated` = shunt-primary/BMS-fallback,
`RawBms` = always BMS). Per-pack `pack{N}/soc` and CAN TX SOC/SOH always use the
BMS value, never `soc_display` — see `docs/research/v3.2-shunt-soc-fusion.md`.

### Level 2+ — Alarm topic (not retained)

Published on each safety state transition. Topic: `{base}/alarm`.

```json
{
  "ts_ms": 1748995200000,
  "event": "CellOvervoltStart",
  "bms_id": 0,
  "alarm_bits": 2
}
```

`bms_id` is 0-based. `bms_id = 255` for system-wide events (NoPacksOnline,
PacksOnlineRecovered). See [safety.md](safety.md) for the full event list.

### Level 3+ (PerPack) — per-pack plain-text topics

One sub-topic per pack, per metric. Pack number N is 1-based.
`online` is published every 1 s. All other value topics rotate 1 per tick
(full cycle every ~10 s). All retained.

| Topic | Value type | Example |
|---|---|---|
| `{base}/pack{N}/online` | `true` or `false` | `true` |
| `{base}/pack{N}/soc` | integer % | `75` |
| `{base}/pack{N}/voltage` | float V (2 dp) | `52.40` |
| `{base}/pack{N}/current` | float A (1 dp) | `5.2` |
| `{base}/pack{N}/power` | integer W | `270` |
| `{base}/pack{N}/temperature` | float °C (1 dp) | `25.3` |
| `{base}/pack{N}/cell_v_min` | float V (3 dp) | `3.324` |
| `{base}/pack{N}/cell_v_max` | float V (3 dp) | `3.327` |
| `{base}/pack{N}/cell_drift` | float V (3 dp) | `0.003` |
| `{base}/pack{N}/soh` | integer % | `98` |
| `{base}/pack{N}/cycles` | integer | `420` |

### Level 4 (PerCell) — individual cell voltage topics

One retained topic per cell per pack. Two topics are published per tick;
a full rotation across all packs takes `(n_packs × 15) / 2` seconds.

```
{base}/pack{N}/cell_v_{NN}
```

`N` is 1-based pack number, `NN` is 01-based cell index (e.g. `cell_v_01`).
Value: float V (3 dp), e.g. `3.325`.

### Level 4 (PerCell) — JSON cells blob (not retained)

Published once per pack approximately every 30 s. Intended for Home Assistant
`value_template` entities that need the full cell array.

Topic: `{base}/cells/bms{N}` (N is 0-based).

```json
{
  "ts_ms": 1748995200000,
  "bms_id": 0,
  "online": true,
  "cell_count": 16,
  "cells_v": [3.325, 3.325, 3.324, 3.326, 3.325, 3.325, 3.325, 3.325,
              3.325, 3.325, 3.325, 3.325, 3.325, 3.325, 3.325, 3.325],
  "temp_count": 7,
  "temps_c": [25.0, 25.1, 25.0, 24.9, 25.0, 25.0, 25.0],
  "cell_v_min": 3.324,
  "cell_v_max": 3.326,
  "cell_v_drift": 0.002,
  "alarm_bits": 0
}
```

### Diagnostics (independent flag `mqtt_diag_enabled`)

Published every 30 s when `mqtt_diag_enabled = true`. Topic: `{base}/diag`.
Not retained. JSON object with poller stats, CAN stats, heap, uptime, and
task high-water marks.

---

## Home Assistant auto-discovery

When `ha_discovery_enabled = true`, the gateway publishes MQTT discovery payloads
on every MQTT connect to:

```
homeassistant/sensor/{device_uid}_{entity_key}/config
```

All discovery payloads are retained and staggered at 50 ms cadence (a full set
for 16 packs takes approximately 4–5 s). The gateway also automatically
tombstones (deletes) stale discovery entries on firmware version change.

`device_uid` = `topband_bms_{full12hexMAC}` (lower-case hex).

### Device model

The gateway registers one **gateway device** in HA. Each pack registers as a
**sub-device** parented to the gateway via `via_device`. Only the gateway device
carries the `sw_version` field (`FW_VERSION_FULL`).

### System entities (gateway device)

State topics are the plain-text individual topics documented above.
All system entities are registered at all publish levels.

| Entity key | HA name | Device class | Unit |
|---|---|---|---|
| `soc` | SOC | battery | % |
| `soh` | SOH | — | % |
| `voltage` | Pack Voltage | voltage | V |
| `current` | Total Current | current | A |
| `power` | Pack Power | power | W |
| `temperature` | Temperature | temperature | °C |
| `cell_v_min` | Cell Voltage Min | voltage | V |
| `cell_v_max` | Cell Voltage Max | voltage | V |
| `cell_drift` | Cell Drift | voltage | V |
| `cvl` | Charge Voltage Limit | voltage | V |
| `ccl` | Charge Current Limit | current | A |
| `dcl` | Discharge Current Limit | current | A |
| `alarm_flags` | Alarm Flags | — | — |
| `bms_online` | Packs Online | — | — |
| `bms_configured` | Packs Configured | — | — |
| `sys_message` | System Message | — | — |
| `energy_today_in` | Energy In Today | energy | kWh |
| `energy_today_out` | Energy Out Today | energy | kWh |
| `runtime_est_min` | Runtime Estimate | — | min |
| `runtime_est_state` | Runtime State | — | — |

### Per-pack entities (Level 3+, PerPack)

Each pack gets a sub-device in HA. State topics are `{base}/pack{N}/{field}`.

| Field | Device class | Unit | Type |
|---|---|---|---|
| `soc` | battery | % | sensor |
| `voltage` | voltage | V | sensor |
| `current` | current | A | sensor |
| `power` | power | W | sensor |
| `temperature` | temperature | °C | sensor |
| `cell_v_min` | voltage | V | sensor |
| `cell_v_max` | voltage | V | sensor |
| `cell_drift` | voltage | V | sensor |
| `soh` | — | % | sensor |
| `cycles` | — | — | sensor |
| `online` | connectivity | — | binary_sensor |

### Per-cell entities (Level 4, PerCell)

Individual cell voltage sensors use the JSON cells blob with `value_template`.
State topic: `{base}/cells/bms{N}` (0-based N).

---

## Configuration reference

| Config field | Description | Default |
|---|---|---|
| `mqtt_enabled` | Enable MQTT client | false |
| `mqtt_host` | Broker hostname or IP | — |
| `mqtt_port` | Broker TCP port | 1883 |
| `mqtt_user` | Username (optional) | — |
| `mqtt_pass_obf` | Password (write-only, obfuscated in config JSON) | — |
| `mqtt_base_topic` | Topic prefix before MAC suffix | `topband-bms` |
| `mqtt_level` | Publish level 0–4 (see table above) | 2 |
| `mqtt_diag_enabled` | Publish `/diag` JSON every 30 s | false |
| `ha_discovery_enabled` | Publish HA auto-discovery payloads | false |

TLS is not supported in V3.0. Plain TCP port 1883 only. Suitable for LAN use;
not recommended for direct internet exposure without a reverse proxy or VPN.

---

## Service API

| Endpoint | Method | Description |
|---|---|---|
| `/api/svc/ha/discovery/send` | POST | Force-publish all HA discovery payloads now |
| `/api/svc/ha/discovery/clear` | POST | Reset stale-cleanup NVS marker |
| `/api/mqtt/test` | POST | Trigger a test publish to `{base}/test` |
| `/api/mqtt/test` | GET | Get result of last test publish |

All require authentication. See [api.md](api.md).

---

## Example HA manual sensor (without auto-discovery)

```yaml
mqtt:
  sensor:
    - name: "BMS SOC"
      state_topic: "topband-bms-a1b2/soc"
      unit_of_measurement: "%"
      device_class: battery
    - name: "BMS Voltage"
      state_topic: "topband-bms-a1b2/voltage"
      unit_of_measurement: "V"
      device_class: voltage
```
