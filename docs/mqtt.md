# MQTT Topics Reference

Phase H1 — TopBand BMS Gateway V3.0

---

## Base topic

All topics use an **effective base** derived at runtime:

```
{cfg.mqtt_base_topic}-{last4hexMAC}
```

Example: configured base `topband-bms`, MAC ending `a1b2` → effective base `topband-bms-a1b2`.

This makes multiple gateways coexist on the same broker without topic collision.

---

## Topic table

| Topic | Retained | Published by | Cadence |
|---|---|---|---|
| `{base}/status` | yes | publisher on connect/LWT | on connect / broker LWT |
| `{base}/data` | no | HousekeepingTask | every 5 s |
| `{base}/diag` | no | HousekeepingTask | every 30 s (when enabled) |
| `{base}/alarm` | no | ControlTask | on safety event |
| `{base}/cells/bms{N}` | no | HousekeepingTask | every 20 s, round-robin per pack |

---

## Payload schemas

### `/status`

Plain string: `online` or `offline` (LWT).

---

### `/data`

```json
{
  "ts_ms": 1234567890,
  "uptime_s": 3600,
  "bms_count_online": 2,
  "bms_count_configured": 2,
  "soc_avg": 75.0,
  "soh_avg": 98.0,
  "pack_voltage_avg": 52.0,
  "pack_current_total": 10.0,
  "pack_power_w": 520.0,
  "temp_avg": 25.0,
  "temp_max": 30.0,
  "cell_v_min": 3.3,
  "cell_v_max": 3.35,
  "cell_v_drift": 0.05,
  "cvl_v": 56.0,
  "ccl_a": 80.0,
  "dcl_a": 80.0,
  "alarm_flags": 0,
  "sys_message": ""
}
```

---

### `/diag`

All `/data` fields plus:

```json
{
  "cycles_completed": 1200,
  "cycle_avg_ms": 50,
  "cycle_max_ms": 120,
  "alarm_polls_ok": 400,
  "alarm_polls_err": 0,
  "can_tx_ok": 3600,
  "can_tx_fail": 0,
  "can_busoff_count": 0
}
```

---

### `/alarm`

Published once per safety state transition:

```json
{
  "ts_ms": 1234567890,
  "event": "CellOverVoltage",
  "bms_id": 0,
  "alarm_bits": 1
}
```

---

### `/cells/bms{N}`

N is 0-based pack index.

```json
{
  "ts_ms": 1234567890,
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

---

## Home Assistant auto-discovery

When `ha_discovery_enabled` is true, the gateway publishes discovery payloads on every MQTT connect to:

```
homeassistant/sensor/{device_uid}_{entity_key}/config
```

`device_uid` = `topband_bms_{full12hexMAC}`.

### System entities (state_topic: `/data`)

| Entity key | HA name | Device class | Unit |
|---|---|---|---|
| `soc_avg` | SOC Average | battery | % |
| `soh_avg` | SOH Average | — | % |
| `pack_voltage_avg` | Pack Voltage | voltage | V |
| `pack_current_total` | Total Current | current | A |
| `pack_power_w` | Pack Power | power | W |
| `temp_avg` | Temp Average | temperature | C |
| `temp_max` | Temp Max | temperature | C |
| `cell_v_min` | Cell Voltage Min | voltage | V |
| `cell_v_max` | Cell Voltage Max | voltage | V |
| `cell_v_drift` | Cell Voltage Drift | voltage | V |
| `cvl_v` | Charge Voltage Limit | voltage | V |
| `ccl_a` | Charge Current Limit | current | A |
| `dcl_a` | Discharge Current Limit | current | A |
| `alarm_flags` | Alarm Flags | — | — |
| `bms_count_online` | Packs Online | — | — |
| `sys_message` | System Message | — | — |

### Per-pack entities (state_topic: `/cells/bms{N}`)

Entity key format: `pack_{N}_{key}` (N is 1-based for HA readability).

| Key suffix | HA name template | Device class | Unit |
|---|---|---|---|
| `voltage` | Pack N Voltage | voltage | V |
| `current` | Pack N Current | current | A |
| `soc` | Pack N SOC | battery | % |
| `alarm_bits` | Pack N Alarms | — | — |

---

## Configuration

| Config field | Description | Default |
|---|---|---|
| `mqtt_enabled` | Enable MQTT | false |
| `mqtt_host` | Broker hostname or IP | — |
| `mqtt_port` | Broker port | 1883 |
| `mqtt_user` | Username (optional) | — |
| `mqtt_pass_obf` | Password (write-only) | — |
| `mqtt_base_topic` | Topic prefix | `topband-bms` |
| `mqtt_level` | 0=off, 1=status, 2=data, 3=per-cell | 2 |
| `mqtt_diag_enabled` | Publish /diag topic | false |
| `ha_discovery_enabled` | HA auto-discovery | false |

---

## Service API

| Endpoint | Method | Description |
|---|---|---|
| `/api/svc/ha/discovery/send` | POST | Force-publish all HA discovery payloads now |
| `/api/svc/ha/discovery/clear` | POST | Reset stale-cleanup NVS marker; re-publishes if connected |

Both endpoints require authentication.

---

## Example HA manual sensor (fallback without auto-discovery)

```yaml
mqtt:
  sensor:
    - name: "BMS SOC"
      state_topic: "topband-bms-a1b2/data"
      value_template: "{{ value_json.soc_avg }}"
      unit_of_measurement: "%"
      device_class: battery
```
