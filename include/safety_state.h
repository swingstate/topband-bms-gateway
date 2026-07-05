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
};

// CAN TX SOC (V3.2): the value reported to the inverter over CAN follows the
// dashboard's "Combined SOC" — the Battery Value Sources fused reading
// (shunt-led when fresh, BMS fallback otherwise). Falls back to raw BMS soc_avg
// whenever no fused value is currently valid, because a CAN frame must always
// carry a number (never a bare 0 / "no data").
//
// IMPORTANT: this is the DISPLAY/reporting SOC only. It is deliberately NOT the
// same as the charge-taper safety input, which uses raw BMS soc_avg exclusively
// (see safety/runSafety.cpp). Do not conflate the two: the taper must never
// follow the shunt-fused value, this reporting field intentionally does.
inline int can_tx_soc(const SafetyState& s) {
  return static_cast<int>(s.soc_display_valid ? s.soc_display : s.soc_avg);
}
