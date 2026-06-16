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
  float    ccl_amps;               // effective CCL after temp throttle + proto cap
  float    dcl_amps;               // effective DCL after temp throttle + proto cap
  float    pack_voltage_avg;
  float    pack_current_total;
  float    soc_avg;
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
