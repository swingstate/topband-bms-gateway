<details>
<summary><b>MQTT Topics Reference</b> (click to expand)</summary>

All topics are published by the gateway. Base topic is configurable in the MQTT tab (default `Topband/BMS`). Placeholders: `{base}` = configured base topic, `{uid}` = device UID (e.g. `tb_79944c`), `{n}` = BMS index (0-15).

### Runtime Data Topics (published by gateway)

| Topic | Cadence | Retained | Purpose | Toggle |
|---|---|---|---|---|
| `{base}/status` | On connect / LWT | Yes | `"online"` on connect, `"offline"` on unexpected disconnect (Last Will) | `mq_en` |
| `{base}/data` | 5 s | No | Main JSON payload: SOC, power, voltage, current, temperatures, alarms, active pack count, energy counters, full config, per-BMS array (if `mq_full` enabled) | `mq_en` |
| `{base}/alarm` | On state change | Yes | Alarm state transitions: `{"state": "...", "flags": N, "safe_lock": bool}` | `mq_en` |
| `{base}/diag` | 30 s | Yes | Diagnostic telemetry (21 keys): fw, uptime, boot_reason, heap, NVS, CAN counters, RS485 stack HWM, BMS stats, workaround counters, debug counters | `mq_diag` |

### Home Assistant Auto-Discovery Topics (published by gateway)

All retained. Empty payload published on toggle-off to remove entities from HA.

#### Main sensor discovery (14 entities, published when `ha_en` enabled)

| Topic | HA Entity | Unit | Device Class |
|---|---|---|---|
| `homeassistant/sensor/{uid}/soc/config` | SOC | % | battery |
| `homeassistant/sensor/{uid}/p/config` | Power | W | power |
| `homeassistant/sensor/{uid}/v/config` | Voltage | V | voltage |
| `homeassistant/sensor/{uid}/i/config` | Current | A | current |
| `homeassistant/sensor/{uid}/active/config` | Active BMS | packs | — |
| `homeassistant/sensor/{uid}/max_chg/config` | Max Charge Current | A | current |
| `homeassistant/sensor/{uid}/max_dis/config` | Max Discharge Current | A | current |
| `homeassistant/sensor/{uid}/alarm/config` | Alarm | — | — |
| `homeassistant/sensor/{uid}/avg_temp/config` | Avg Temperature | °C | temperature |
| `homeassistant/sensor/{uid}/soh/config` | SOH | % | — |
| `homeassistant/sensor/{uid}/rem_cap/config` | Remaining Capacity | Ah | — |
| `homeassistant/sensor/{uid}/total_cap/config` | Total Capacity | Ah | — |
| `homeassistant/sensor/{uid}/energy_in/config` | Energy In Today | kWh | energy |
| `homeassistant/sensor/{uid}/energy_out/config` | Energy Out Today | kWh | energy |

#### Per-BMS sensor discovery (6 entities per BMS, published when `ha_en` + `mq_full` enabled)

| Topic | HA Entity | Unit | Device Class |
|---|---|---|---|
| `homeassistant/sensor/{uid}/bms{n}_soc/config` | BMS {n} SOC | % | battery |
| `homeassistant/sensor/{uid}/bms{n}_v/config` | BMS {n} Voltage | V | voltage |
| `homeassistant/sensor/{uid}/bms{n}_i/config` | BMS {n} Current | A | current |
| `homeassistant/sensor/{uid}/bms{n}_min_cell/config` | BMS {n} Min Cell | V | voltage |
| `homeassistant/sensor/{uid}/bms{n}_max_cell/config` | BMS {n} Max Cell | V | voltage |
| `homeassistant/binary_sensor/{uid}/bms{n}_online/config` | BMS {n} Online | — | connectivity |

#### Diagnostic sensor discovery (21 entities, V2.66.3+, published when `ha_en` + `mq_diag` enabled)

| Topic | HA Entity | Unit | Device Class | State Class |
|---|---|---|---|---|
| `homeassistant/sensor/{uid}_diag_fw/config` | FW Version | — | — | — |
| `homeassistant/sensor/{uid}_diag_uptime/config` | Uptime | s | duration | total_increasing |
| `homeassistant/sensor/{uid}_diag_boot_reason/config` | Boot Reason | — | — | — |
| `homeassistant/sensor/{uid}_diag_heap_free/config` | Heap Free | B | data_size | measurement |
| `homeassistant/sensor/{uid}_diag_heap_min/config` | Heap Min | B | data_size | measurement |
| `homeassistant/sensor/{uid}_diag_nvs_used/config` | NVS Used | — | — | measurement |
| `homeassistant/sensor/{uid}_diag_nvs_free/config` | NVS Free | — | — | measurement |
| `homeassistant/sensor/{uid}_diag_can_tx_ok/config` | CAN TX OK | — | — | total_increasing |
| `homeassistant/sensor/{uid}_diag_can_tx_fail/config` | CAN TX Fail | — | — | total_increasing |
| `homeassistant/sensor/{uid}_diag_can_tx_fail_streak/config` | CAN TX Fail Streak | — | — | measurement |
| `homeassistant/sensor/{uid}_diag_can_status/config` | CAN Status | — | — | — |
| `homeassistant/sensor/{uid}_diag_rs485_hwm/config` | RS485 Stack HWM | B | data_size | measurement |
| `homeassistant/sensor/{uid}_diag_bms_polls/config` | BMS Total Polls | — | — | total_increasing |
| `homeassistant/sensor/{uid}_diag_bms_timeouts/config` | BMS Total Timeouts | — | — | total_increasing |
| `homeassistant/sensor/{uid}_diag_current_holds/config` | Current Holds | — | — | total_increasing |
| `homeassistant/sensor/{uid}_diag_spike_rejects/config` | Spike Rejects | — | — | total_increasing |
| `homeassistant/sensor/{uid}_diag_rl_rejects/config` | Rate-Limit Rejects | — | — | total_increasing |
| `homeassistant/sensor/{uid}_diag_mqtt_fail/config` | MQTT Fail Count | — | — | total_increasing |
| `homeassistant/sensor/{uid}_diag_session_age_d/config` | Session Age | d | duration | measurement |
| `homeassistant/sensor/{uid}_diag_stream_aborts/config` | Stream Aborts | — | — | total_increasing |
| `homeassistant/sensor/{uid}_diag_handler_max_ms/config` | Handler Max | ms | duration | measurement |
| `homeassistant/sensor/{uid}_diag_loop_max_ms/config` | Loop Max | ms | duration | measurement |
| `homeassistant/sensor/{uid}_diag_wdt_warnings/config` | WDT Warnings | — | — | total_increasing |

### UI Toggle Reference

| Toggle (UI) | NVS Key | Default | Effect |
|---|---|---|---|
| MQTT Enable | `mq_en` | off | Master switch for all MQTT publishing |
| Include per-BMS cell data | `mq_full` | off | Adds `bms[]` array to `/data` payload + per-BMS HA entities |
| Publish diagnostics (topic: .../diag, 30s) | `mq_diag` | off | Enables `/diag` topic publish + 21 diag HA entities |
| Home Assistant Auto-Discovery | `ha_en` | off | Publishes `homeassistant/*` discovery configs |

### Notes

- All HA discovery configs are published as retained messages, so they survive HA restarts. Broker persistence (`persistence true` in Mosquitto config) recommended so they also survive broker restarts.
- When `mq_diag` is toggled off, empty retained payloads are published to each of the 21 discovery topics, cleanly removing the entities from HA.
- When `mq_full` is toggled off, empty retained payloads are published to each of the 96 per-BMS discovery topics (16 BMS × 6 entities).
- Payload for `/data` is documented separately (see `docs/API.md` or the source comments in `handleData()`).
- Diagnostic payload schema (21 JSON keys) is documented in the V2.66.2 header comment in the source.

</details>
