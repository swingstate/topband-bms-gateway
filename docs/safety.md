# Safety Module — TopBand BMS Gateway V3.0.0

> **Safety disclaimer:** This firmware is intended for DIY integration by
> technically competent users. LiFePO4 batteries operating at high current carry
> risk of fire, injury, and property damage if safety limits are misconfigured.
> Always verify your configured thresholds against the manufacturer's cell and
> pack specifications. The gateway is one layer in your protection chain — the
> BMS itself and hardware protection circuits are the primary safeguard. Test all
> protection paths before live deployment.

---

## Safety model overview

The safety loop runs inside `ControlTask` (Core 0) every 1 s. Its only output is
a `SafetyState` struct that CAN TX, MQTT, and the diagnostics layer consume.
`runSafety()` is a **pure function** — same inputs produce the same output, no
I/O, no timers, no logging. The caller routes state-transition events to the alert
log and notification layer.

**The safety control response (CVL/CCL/DCL to the inverter) is immediate — it
changes on the next CAN frame within milliseconds of the triggering condition.**
The alert debounce and notification cooldown described below apply only to the
human-facing alert log and external notifications (Telegram). They do not delay,
soften, or suppress the protective control response.

---

## Thresholds and configuration

All thresholds are stored in `NvsConfig` and configurable via Settings → Battery.

| Config field | Description | Affects |
|---|---|---|
| `safe_pack_volt` | Pack voltage overvolt cutoff (V) | alarm_flags 0x02, CCL/DCL zeroed |
| `safe_cell_volt` | Cell voltage overvolt cutoff (V) | alarm_flags 0x02, CCL/DCL zeroed |
| `safe_cell_drift` | Cell imbalance warning threshold (V) | alarm_flags 0x20 |
| `cvl_voltage` | Base charge voltage limit (V) | CVL sent to inverter |
| `charge_amps_per_pack` | Max charge current per online pack (A) | CCL computation |
| `discharge_amps_per_pack` | Max discharge current per online pack (A) | DCL computation |
| `charge_temp_min/max` | Charging temperature range (°C) | CCL throttle/cutoff |
| `discharge_temp_min/max` | Discharging temperature range (°C) | DCL throttle/cutoff |
| `temp_mode` | Temperature source: `Hottest` (conservative) or `Average` | temp check value |

There is no undervolt threshold in the user-configurable safety parameters.
Undervolt detection is driven exclusively by the BMS sysparam frame (0x47)
`sys_module_under_v` field.

---

## CVL / CCL / DCL computation

Each cycle:

1. **Base CCL** = `charge_amps_per_pack × online_pack_count × temp_factor_charge`
2. **Base DCL** = `discharge_amps_per_pack × online_pack_count × temp_factor_discharge`
3. **Sysparam caps** (from BMS 0x47 frame, valid ≤ 5 min): if BMS-reported
   `sys_charge_max_a / sys_discharge_max_a` are lower than the base values, the
   sysparam values take precedence. CVL is similarly capped from
   `sys_module_high_v - 0.20 V`, further bounded by `safe_pack_volt - 0.20 V`
   and `cvl_voltage`.
4. **SOC taper** (when `maint_charge_enabled = false`): CCL is reduced to 2 A/pack
   at SOC ≥ 99 %, zeroed at SOC = 100 %. This shapes charge near full; it is not
   a fault and is not reported as a lockout.
5. **Direction-aware protection lock** (V3.2, applied AFTER the taper so a genuine
   fault stop is authoritative): each protection condition zeroes ONLY the
   direction whose continuation would worsen it:

   | Condition | Blocks | Rationale |
   |---|---|---|
   | Cell/pack over-voltage (`0x02`) | charge only | discharging lowers voltage, helping; charging worsens it |
   | Cell/pack under-voltage (`0x10`) | discharge only | charging raises voltage, helping; discharging worsens it |
   | Charge temp cutoff (`factor_charge == 0`, hot or cold) | charge only | too hot/cold to charge; discharging cold is fine |
   | Discharge temp cutoff (`factor_discharge == 0`) | discharge only | too hot to discharge |
   | BMS-reported critical alarm (`0x40`) | both | 0x44 bitmap not decoded per-direction; conservative |
   | No packs online (`0x80`) | both | no data at all |

   Prior to V3.2 a single blanket rule zeroed BOTH directions on `0x02 | 0x10 |
   0x40`. That was physically backwards for over-voltage (blocking discharge
   prevented the pack from self-correcting) and was the confirmed root cause of a
   field report where an inverter refused to discharge a full battery.

   `lockout_flags` (0x01 charge, 0x02 discharge) records which direction a
   *protection* condition disabled — driving the dashboard banner and alert log.
   The SoC taper reaching 100 % is deliberately NOT flagged (battery full ≠ fault).

The resulting CVL/CCL/DCL values are sent to the inverter on every CAN cycle
(see CAN protocols below). A value of 0.0 A for CCL/DCL is the protective
"stop" signal to the inverter — now applied per direction.

---

## Temperature throttle

`calc_factor(t, t_min, t_max)` returns one of four values. Hard-coded soft zone
width: 5 °C.

| Temperature range | Factor | Inverter effect |
|---|---|---|
| `t < t_min` | 0.0 | full cutoff |
| `t_min ≤ t < t_min + 5` | 0.2 | 20 % of limit |
| `t_min + 5 ≤ t ≤ t_max - 5` | 1.0 | full limit (normal) |
| `t_max - 5 < t ≤ t_max` | 0.5 | 50 % of limit |
| `t > t_max` | 0.0 | full cutoff |

The `temp_mode` setting determines which temperature value is used:
- `Hottest` (default, conservative): uses the highest `temp_max_c` across all
  online packs. Protects the hottest cell in the string.
- `Average`: uses the average `temp_avg_c` across all online packs.

---

## Alarm flags bitmap

`alarm_flags` is a single byte, byte-identical to the V2.67 CAN alarm byte
(architecture constraint — the byte layout cannot change without breaking inverter
compatibility).

| Bit | Value | Condition |
|---|---|---|
| 1 | `0x02` | Pack or cell overvolt (`pack_voltage > safe_pack_volt` or `cell_max_v > safe_cell_volt`) |
| 3 | `0x08` | Temperature stop — charge or discharge cutoff (factor = 0.0) |
| 4 | `0x10` | Pack undervolt (from BMS sysparam `sys_module_under_v`) |
| 5 | `0x20` | Cell imbalance warning (`cell_drift_v > safe_cell_drift`) |
| 6 | `0x40` | BMS-reported critical alarm via 0x44 frame (filtered by alarm classifier) |
| 7 | `0x80` | No packs online |

Multiple bits can be set simultaneously. CCL and DCL are zeroed whenever
`0x02 | 0x10 | 0x40` is set.

---

## Safety events

`runSafety()` emits state-transition events into `SafetyState.events[]`. The
calling layer routes them to the alert log, MQTT `/alarm` topic, and the
notification layer. Events are edge-triggered — they fire on the cycle a
condition first appears or clears, not repeatedly while it persists (exception:
per-pack OV and imbalance fire every cycle the condition is active, matching V2.67
behaviour).

The full set of 17 defined event types (enum `SafetyState::SafetyEvent`):

| Event | bms_id | When emitted |
|---|---|---|
| `BmsWentOffline` | pack idx | Pack transitions online → offline |
| `BmsCameOnline` | pack idx | Pack transitions offline → online |
| `PackOvervoltStart` | pack idx | Pack or cell OV condition active on this pack; also system-level on alarm_flags 0x02 rising edge |
| `PackOvervoltClear` | 0xFF | alarm_flags 0x02 falling edge |
| `CellOvervoltStart` | — | Defined in enum; not currently emitted (cell OV uses PackOvervoltStart) |
| `CellOvervoltClear` | — | Defined in enum; not currently emitted |
| `PackUndervoltStart` | 0xFF | alarm_flags 0x10 rising edge |
| `PackUndervoltClear` | 0xFF | alarm_flags 0x10 falling edge |
| `TempChargeStop` | 0xFF | temp_factor_charge drops below 1.0; also emitted when temp-stop flag (0x08) first appears |
| `TempChargeResume` | 0xFF | temp_factor_charge returns to 1.0 |
| `TempDischargeStop` | 0xFF | temp_factor_discharge drops below 1.0 |
| `TempDischargeResume` | 0xFF | temp_factor_discharge returns to 1.0 |
| `CellImbalanceStart` | pack idx | `cell_drift_v > safe_cell_drift` on this pack (fires every cycle while active) |
| `CellImbalanceClear` | — | Defined in enum; not currently emitted |
| `BmsReportedAlarm` | pack idx | BMS 0x44 alarm frame contains critical filtered bits (fires every cycle while active) |
| `NoPacksOnline` | 0xFF | All packs offline |
| `PacksOnlineRecovered` | 0xFF | At least one pack comes online after full outage |

`bms_id = 0xFF` indicates a system-wide event (no specific pack).

---

## Alert debounce (affects only reporting)

`notify_debounce_s` (default 30 s, configurable) holds begin-type events in a
pending table before passing them to the alert log and notification providers.
If the triggering condition clears within the debounce window (e.g., a transient
voltage spike), both the begin and clear events are suppressed — no alert entry
is written, no notification is sent.

**This debounce has zero effect on the safety control loop.** The CVL/CCL/DCL
values sent to the inverter change immediately on the first cycle the condition
is detected. Alarm flag bits are set and cleared at full control-loop speed
regardless of debounce state.

---

## CAN output protocols

The inverter CAN protocol is selected at runtime via Settings → General → CAN
Protocol. The `SafetyState` fields CVL/CCL/DCL/alarm_flags/SOC/SOH are encoded
into CAN frames and transmitted on every ControlTask cycle.

### Victron (5 frames)

| Frame ID | Content |
|---|---|
| 0x351 | CVL (0.1 V/LSB), CCL (0.1 A/LSB), DCL (0.1 A/LSB) |
| 0x355 | SOC (1 %/LSB), SOH (1 %/LSB), total capacity (Ah) |
| 0x356 | Pack voltage avg (0.01 V/LSB), current total (0.1 A/LSB), temp avg (0.1 °C/LSB) |
| 0x35A | Alarm and warning flags (Victron protocol mapping) |
| 0x35E | Manufacturer string ("TOPBAND ") |

### Pylontech (6 frames = Victron + 2)

Same 0x351/0x355/0x356/0x35E encoding as Victron. Additional frames:

| Frame ID | Content |
|---|---|
| 0x359 | Alarm bits (Pylontech protocol format) |
| 0x35C | Pack request flags (charge/discharge enable) |

### SMA Sunny Island (6 frames = Victron + 1)

Same 0x351/0x355/0x356 encoding as Victron. Different 0x35A and 0x35E, plus:

| Frame ID | Content |
|---|---|
| 0x35A | Victron alarm mapping + SMA charge/discharge enable bits |
| 0x35B | SMA-specific BMS status frame |
| 0x35E | Manufacturer string ("SMA     ") |

CAN bus speed: 500 kbps (standard for all three protocols).
