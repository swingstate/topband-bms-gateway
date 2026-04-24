<details>
<summary><b>MQTT Topics Reference</b> (click to expand)</summary>

All topics are published by the gateway. Base topic is configurable in the MQTT tab (default `Topband/BMS`). Placeholders: `{base}` = configured base topic, `{uid}` = device UID (e.g. `tb_79944c`), `{n}` = BMS index (0-15).

### Runtime Data Topics (published by gateway)

| Topic | Cadence | Retained | Purpose | Condition |
|---|---|---|---|---|
| `{base}/status` | On connect / LWT | Yes | `"online"` on connect, `"offline"` on unexpected disconnect (Last Will) | MQTT enabled |
| `{base}/data` | 5 s | No | Main JSON payload: SOC, power, voltage, current, temperatures, alarms, active pack count, energy counters, full config. Per-BMS array included when `mq_level` >= 1 | MQTT enabled |
| `{base}/alarm` | On state change | Yes | Alarm state transitions: `{"state": "...", "flags": N, "safe_lock": bool}` | MQTT enabled |
| `{base}/diag` | 30 s | Yes | Diagnostic telemetry (24 keys plus a `_help` dictionary): fw, uptime, boot_reason, heap, NVS, CAN counters, RS485 stack HWM, BMS stats, workaround counters, debug counters, `last_reset_ts` | `mq_diag` |
| `{base}/cells/bms{n}` | 20 s | Yes | Per-pack cell voltages and temperatures: `{"v":1,"online":bool,"cell_count":int,"cells":[floats],"temps":[floats]}`. One topic per pack. Offline packs publish `online:false` with empty arrays so HA entities transition cleanly to unavailable | `mq_level` = 2 |

### Tiered MQTT detail levels (V2.67 F1)

The UI presents three detail levels as plain-English radio options. The NVS key `mq_level` stores the selected value.

| UI label | NVS value | What gets published |
|---|---|---|
| Off | 0 | Core battery data only. Pack aggregates on `{base}/data`, alarms, status. |
| Per-pack statistics | 1 | Off + per-BMS array in `{base}/data` + per-BMS HA discovery entities (SOC, V, I, min_cell, max_cell, online) plus 6 stat entities per pack (drift_mv, polls, timeouts, errors, spikes, current_holds). |
| Additional per-cell voltages | 2 | Per-pack statistics + retained `{base}/cells/bms{n}` topic per pack every 20 s with every individual cell voltage and pack temperature. |

`{base}/diag` (gateway self-monitoring) is orthogonal to `mq_level`. It has its own toggle and its own 30 s cadence.

### Home Assistant Auto-Discovery Topics (published by gateway)

All retained. Empty payload published on toggle-off to remove entities from HA.

#### Main sensor discovery (14 entities, published when Home Assistant Auto-discovery enabled)

| Topic | HA Entity | Unit | Device Class |
|---|---|---|---|
| `homeassistant/sensor/{uid}/soc/config` | SOC | % | battery |
| `homeassistant/sensor/{uid}/p/config` | Power | W | power |
| `homeassistant/sensor/{uid}/v/config` | Voltage | V | voltage |
| `homeassistant/sensor/{uid}/i/config` | Current | A | current |
| `homeassistant/sensor/{uid}/active/config` | Active BMS | packs | |
| `homeassistant/sensor/{uid}/max_chg/config` | Max Charge Current | A | current |
| `homeassistant/sensor/{uid}/max_dis/config` | Max Discharge Current | A | current |
| `homeassistant/sensor/{uid}/alarm/config` | Alarm | | |
| `homeassistant/sensor/{uid}/avg_temp/config` | Avg Temperature | °C | temperature |
| `homeassistant/sensor/{uid}/soh/config` | SOH | % | |
| `homeassistant/sensor/{uid}/rem_cap/config` | Remaining Capacity | Ah | |
| `homeassistant/sensor/{uid}/total_cap/config` | Total Capacity | Ah | |
| `homeassistant/sensor/{uid}/energy_in/config` | Energy In Today | kWh | energy |
| `homeassistant/sensor/{uid}/energy_out/config` | Energy Out Today | kWh | energy |

#### Per-BMS sensor discovery (12 entities per BMS, published when HA + `mq_level` >= 1)

Core entities (6) are published at level 1 and level 2. Stat entities (6) ship with Per-pack statistics (level 1).

| Topic | HA Entity | Unit | Device Class |
|---|---|---|---|
| `homeassistant/sensor/{uid}/bms{n}_soc/config` | BMS {n} SOC | % | battery |
| `homeassistant/sensor/{uid}/bms{n}_v/config` | BMS {n} Voltage | V | voltage |
| `homeassistant/sensor/{uid}/bms{n}_i/config` | BMS {n} Current | A | current |
| `homeassistant/sensor/{uid}/bms{n}_min_cell/config` | BMS {n} Min Cell | V | voltage |
| `homeassistant/sensor/{uid}/bms{n}_max_cell/config` | BMS {n} Max Cell | V | voltage |
| `homeassistant/binary_sensor/{uid}/bms{n}_online/config` | BMS {n} Online | | connectivity |
| `homeassistant/sensor/{uid}/bms{n}_drift_mv/config` | BMS {n} Cell Spread | mV | voltage |
| `homeassistant/sensor/{uid}/bms{n}_polls/config` | BMS {n} Polls | | |
| `homeassistant/sensor/{uid}/bms{n}_timeouts/config` | BMS {n} Timeouts | | |
| `homeassistant/sensor/{uid}/bms{n}_errors/config` | BMS {n} Errors | | |
| `homeassistant/sensor/{uid}/bms{n}_spikes/config` | BMS {n} Rejected Spikes | | |
| `homeassistant/sensor/{uid}/bms{n}_current_holds/config` | BMS {n} Held Current Reads | | |

#### Diagnostic sensor discovery (24 entities, published when HA + `mq_diag` enabled)

| Topic | HA Entity | Unit | Device Class | State Class |
|---|---|---|---|---|
| `homeassistant/sensor/{uid}_diag_fw/config` | FW Version | | | |
| `homeassistant/sensor/{uid}_diag_uptime/config` | Uptime | s | duration | total_increasing |
| `homeassistant/sensor/{uid}_diag_boot_reason/config` | Boot Reason | | | |
| `homeassistant/sensor/{uid}_diag_heap_free/config` | Heap Free | B | data_size | measurement |
| `homeassistant/sensor/{uid}_diag_heap_min/config` | Heap Min | B | data_size | measurement |
| `homeassistant/sensor/{uid}_diag_nvs_used/config` | NVS Used | | | measurement |
| `homeassistant/sensor/{uid}_diag_nvs_free/config` | NVS Free | | | measurement |
| `homeassistant/sensor/{uid}_diag_can_tx_ok/config` | CAN TX OK | | | total_increasing |
| `homeassistant/sensor/{uid}_diag_can_tx_fail/config` | CAN TX Fail | | | total_increasing |
| `homeassistant/sensor/{uid}_diag_can_tx_fail_streak/config` | CAN TX Fail Streak | | | measurement |
| `homeassistant/sensor/{uid}_diag_can_status/config` | CAN Status | | | |
| `homeassistant/sensor/{uid}_diag_rs485_hwm/config` | RS485 Stack HWM | B | data_size | measurement |
| `homeassistant/sensor/{uid}_diag_bms_polls/config` | BMS Total Polls | | | total_increasing |
| `homeassistant/sensor/{uid}_diag_bms_timeouts/config` | BMS Total Timeouts | | | total_increasing |
| `homeassistant/sensor/{uid}_diag_current_holds/config` | Current Holds | | | total_increasing |
| `homeassistant/sensor/{uid}_diag_spike_rejects/config` | Spike Rejects | | | total_increasing |
| `homeassistant/sensor/{uid}_diag_rl_rejects/config` | Rate-Limit Rejects | | | total_increasing |
| `homeassistant/sensor/{uid}_diag_mqtt_fail/config` | MQTT Fail Count | | | total_increasing |
| `homeassistant/sensor/{uid}_diag_session_age_d/config` | Session Age | d | duration | measurement |
| `homeassistant/sensor/{uid}_diag_stream_aborts/config` | Stream Aborts | | | total_increasing |
| `homeassistant/sensor/{uid}_diag_handler_max_ms/config` | Handler Max | ms | duration | measurement |
| `homeassistant/sensor/{uid}_diag_loop_max_ms/config` | Loop Max | ms | duration | measurement |
| `homeassistant/sensor/{uid}_diag_wdt_warnings/config` | WDT Warnings | | | total_increasing |
| `homeassistant/sensor/{uid}_diag_last_reset_ts/config` | Last Counter Reset | | | |

The `last_reset_ts` entity (V2.67 F4) is published as a plain integer Unix epoch. Users who want a wall-clock display in HA can add a template sensor that renders the value.

### UI Toggle Reference (V2.67.2 labels)

| Toggle (UI) | NVS Key | Default | Effect |
|---|---|---|---|
| Enable MQTT | `mq_en` | off | Master switch for all MQTT publishing |
| Extended battery monitoring: Off / Per-pack statistics / Additional per-cell voltages | `mq_level` | 0 | Tiers the per-BMS detail level. See level table above |
| Publish gateway diagnostics | `mq_diag` | off | Enables `/diag` topic publish + 24 diag HA entities |
| Home Assistant Auto-discovery | `ha_en` | off | Publishes `homeassistant/*` discovery configs |

### Notes

- All HA discovery configs are published as retained messages, so they survive HA restarts. Broker persistence (`persistence true` in Mosquitto config) recommended so they also survive broker restarts.
- When `mq_diag` is toggled off, empty retained payloads are published to each of the 24 discovery topics, cleanly removing the entities from HA.
- When `mq_level` is lowered, entities that exceed the new level are removed via empty retained payloads. Level 1 → 0 removes 12 per-BMS entities (16 packs × 12 = up to 192 topics). Level 2 → 1 additionally clears retained `{base}/cells/bms{n}` payloads.
- V2.67 originally introduced level 2 with a synchronous publish loop holding the data mutex. V2.67.1 replaced that with a snapshot pattern so a WiFi or broker stall during the publish cannot cause a main-loop watchdog reset.
- The `last_reset_ts` diag key is 0 until NTP has synced once after a fresh flash. After that it tracks the Unix epoch of the last counter reset (manual or 7-day automatic rollover).
- Payload for `/data` is documented in the `handleData()` source comments.
- Diagnostic payload schema (24 JSON keys plus `_help` dictionary) is documented in the V2.67 F3 header comment.

</details>
