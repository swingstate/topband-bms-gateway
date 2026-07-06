#pragma once
#include <cstdint>

// ── SafetyState ──────────────────────────────────────────────────────────────
// Output of safety::runSafety(). Input to CAN TX builders (Phase E).
// Architecture §5.4 canonical definition.
// Stored in internal SRAM (not PSRAM) for hot-path latency. ~256 B.
struct SafetyState {
  uint32_t cycle_id;               // matches BmsSystemSnapshot.cycle_id
  uint32_t produced_ms;

  // Aggregated for inverter output
  float    cvl_volts;              // effective CVL (may be < cfg.cvl_voltage)
  float    dvl_volts;              // discharge voltage limit = configured pack
                                    // low-voltage cutoff (cfg.safe_pack_volt).
                                    // Reported to the inverter (Pylontech 0x351
                                    // bytes 6-7); NOT a runtime cutoff decision.
  float    ccl_amps;               // effective CCL after temp throttle + proto cap
  float    dcl_amps;               // effective DCL after temp throttle + proto cap
  float    pack_voltage_avg;
  float    pack_current_total;
  float    soc_avg;                // pure BMS mean. Charge-taper (below) and CAN TX
                                    // always use THIS field, never soc_display.
  // ── Battery Value Sources fusion (V3.2) — display/MQTT only ────────────────
  // *_display / *_source_shunt / *_display_valid are set by the caller
  // (bms/poller.cpp) via Aggregator::fuse_bank_voltage/current/soc() AFTER
  // runSafety() returns; runSafety() itself never touches these fields (no
  // I/O/globals rule). *_display_valid=false means neither BMS nor shunt has
  // usable data right now — the UI must show "no data", never a bare 0.
  // PERMANENT SAFETY INVARIANT: charge-taper and CAN TX always use the plain
  // BMS fields above (soc_avg, pack_voltage_avg, pack_current_total), never
  // any of the *_display fields, regardless of Battery Value Sources policy.
  float    voltage_display;
  bool     voltage_source_shunt;
  bool     voltage_display_valid;
  float    current_display;
  bool     current_source_shunt;
  bool     current_display_valid;
  float    soc_display;
  bool     soc_source_shunt;       // true if soc_display came from the shunt this cycle.
  bool     soc_display_valid;
  float    soh_avg;
  float    temp_avg;
  float    capacity_total_ah;
  float    capacity_remain_ah;

  // Throttle factors — same values calcFactor() returns: {0.0, 0.2, 0.5, 1.0}
  float    factor_charge;
  float    factor_discharge;

  // Alarm bitmap — byte-identical to V2.67 (CAN-level constraint; see arch §4.4)
  uint8_t  alarm_flags;
  // 0x02 = pack/cell overvolt
  // 0x08 = temperature stop (charge or discharge cutoff)
  // 0x10 = pack/cell undervolt  (sysparam-sourced or BMS alarm UV bits)
  // 0x20 = cell drift / imbalance warning
  // 0x40 = BMS reported critical alarm via 0x44
  // 0x80 = no packs online
  // Direction of an active temperature stop (0x08 above is combined; see
  // temp_alarm below). temp_alarm is declared further down to sit in existing
  // struct padding at zero DRAM cost, not next to alarm_flags.

  char     sys_message[48];
  uint8_t  packs_online;
  uint8_t  packs_configured;

  // State-transition events emitted per cycle. Caller routes to log/alert queues.
  // runSafety() itself never calls ESP_LOG or any I/O.
  uint8_t  event_count;

  enum SafetyEvent : uint8_t {
    None = 0,
    BmsWentOffline,
    BmsCameOnline,
    PackOvervoltStart, PackOvervoltClear,
    CellOvervoltStart, CellOvervoltClear,
    PackUndervoltStart, PackUndervoltClear,
    TempChargeStop, TempChargeResume,
    TempDischargeStop, TempDischargeResume,
    CellImbalanceStart, CellImbalanceClear,
    BmsReportedAlarm,
    NoPacksOnline, PacksOnlineRecovered,
  };

  struct EventEntry {
    SafetyEvent type;
    uint8_t     bms_id;      // 0xFF for system-wide events
    uint64_t    alarm_bits;  // valid for BmsReportedAlarm; else 0
  };

  static constexpr uint8_t MAX_EVENTS = 16;
  EventEntry events[MAX_EVENTS];
  bool       events_overflowed;

  // Direction of an active temperature stop. alarm_flags 0x08 (temperature stop)
  // is combined and does not distinguish cold from hot; CAN encoders that report
  // over/under temperature as separate bits (Pylontech 0x359) need the direction.
  // Set by runSafety() from the same t_check_val and thresholds that drive
  // factor_charge/factor_discharge. Zero whenever no temperature stop is active.
  //   0x01 = under-temperature (cold) stop active
  //   0x02 = over-temperature  (hot)  stop active
  // Declared last so it occupies the struct's existing 7-byte tail padding after
  // events_overflowed — net-zero internal DRAM (the pre-events region is already
  // 8-byte-aligned, so a byte there would cost a full 8-byte slot instead).
  uint8_t  temp_alarm;
};

// Reported SOC (V3.2): the integer SOC reported to the outside world follows the
// dashboard's "Combined SOC" — the Battery Value Sources fused reading (shunt-led
// when fresh, BMS fallback otherwise). Falls back to raw BMS soc_avg whenever no
// fused value is currently valid, because a reported frame must always carry a
// number (never a bare 0 / "no data").
//
// SINGLE SOURCE for every integer SOC consumer: the CAN builders (0x355) AND the
// MQTT aggregate {base}/soc topic both call this, so the inverter, Home Assistant
// and the dashboard can never disagree. Rounds half-up (SOC is always >= 0) to
// match the dashboard's toFixed(0); truncation here was why MQTT showed 99 while
// the dashboard rounded the same ~99.6 fused value to 100.
//
// IMPORTANT: this is the DISPLAY/reporting SOC only. It is deliberately NOT the
// same as the charge-taper safety input, which uses raw BMS soc_avg exclusively
// (see safety/runSafety.cpp). Do not conflate the two: the taper must never
// follow the shunt-fused value, this reporting field intentionally does.
inline int can_tx_soc(const SafetyState& s) {
  float v = s.soc_display_valid ? s.soc_display : s.soc_avg;
  return static_cast<int>(v + 0.5f);  // round half-up; SOC domain is [0, 100]
}

// Reported current (V3.2): the instantaneous current reported over CAN (0x356
// byte 2-3) follows the dashboard's "Combined Current" — the Battery Value
// Sources fused reading (shunt-led when fresh, BMS fallback otherwise), via the
// exact same rule the MQTT {base}/current topic uses (app/housekeeping.cpp
// iv_current). Falls back to raw BMS pack_current_total whenever no fused value
// is currently valid, because a reported frame must always carry a number.
//
// WHY: the BMS is blind below ~0.5 A and reports 0.0 A, so a battery idling at
// -0.8 A showed "Strom 0,0 A" on the inverter while the dashboard's shunt read
// the real sub-amp current. This is the current analogue of can_tx_soc(): the
// inverter, Home Assistant and the dashboard now agree on the same live current.
//
// Sign convention is preserved (negative = discharge). Returned in amps (float);
// the CAN builders apply the ×10 scale and signed-16-bit encoding themselves.
//
// IMPORTANT: this is the DISPLAY/reporting current only. It is deliberately NOT
// a safety input. CCL/DCL limits, charge-taper and the enable bits all stay on
// the plain BMS fields (see safety/runSafety.cpp) — do not conflate the two.
inline float can_tx_current(const SafetyState& s) {
  return s.current_display_valid ? s.current_display : s.pack_current_total;
}
