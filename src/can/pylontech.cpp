#include "can/pylontech.h"
#include "can/tx.h"
#include <cstring>

// V2.67 reference: legacy/Topband_BMS_Gateway_v2_67_2.ino lines 2332-2350
// sendVictronCAN() with g_can_protocol==1. Frames 0x351/0x355/0x356/0x35E are
// byte-identical to the Victron variant; only the manufacturer string changes.
//
// Frames 0x359 and 0x35C are NOT present in V2.67. They are spec-derived from
// the Pylontech LV BMS CAN Protocol v1.1 community documentation. Bit layout
// matches what widely-deployed Victron/Pylontech integrations expect. Users with
// Pylontech hardware should verify against their inverter's commissioning log.

namespace can::pylontech {

void build_0x351(const SafetyState& s, uint8_t out[8]) {
  memset(out, 0, 8);
  // V2.67 line 2334: cv=int(cvl*10), ccl=int(ccl*10), dcl=int(dcl*10)
  // V3.2: Pylontech 0x351 also carries the Discharge Voltage Limit (DVL) in
  // bytes 6-7 (×10, LE) — the minimum voltage the inverter should discharge to.
  // V2.67/Victron left these zero, which read to inverters as a 0.0 V floor
  // ("discharge to empty"). We now send the configured pack low-voltage cutoff.
  int cv  = static_cast<int>(s.cvl_volts * 10.0f);
  int ccl = static_cast<int>(s.ccl_amps  * 10.0f);
  int dcl = static_cast<int>(s.dcl_amps  * 10.0f);
  int dvl = static_cast<int>(s.dvl_volts * 10.0f);
  out[0] = static_cast<uint8_t>(cv  & 0xFF);  out[1] = static_cast<uint8_t>((cv  >> 8) & 0xFF);
  out[2] = static_cast<uint8_t>(ccl & 0xFF);  out[3] = static_cast<uint8_t>((ccl >> 8) & 0xFF);
  out[4] = static_cast<uint8_t>(dcl & 0xFF);  out[5] = static_cast<uint8_t>((dcl >> 8) & 0xFF);
  out[6] = static_cast<uint8_t>(dvl & 0xFF);  out[7] = static_cast<uint8_t>((dvl >> 8) & 0xFF);
}

void build_0x355(const SafetyState& s, uint8_t out[8]) {
  memset(out, 0, 8);
  // V2.67 lines 2337-2338: SOC=int(avgSOC), SOH=int(avgSOH), cap=int(totalCapacity*10)
  // V3.2: SOC follows the dashboard's Combined SOC (Battery Value Sources fused
  // value) via can_tx_soc() — see can/victron.cpp build_0x355() for the rationale.
  // SOH stays BMS-only (shunt does not measure state of health).
  int soc = can_tx_soc(s);
  int soh = static_cast<int>(s.soh_avg);
  int cap = static_cast<int>(s.capacity_total_ah * 10.0f);
  out[0] = static_cast<uint8_t>(soc & 0xFF);  out[1] = static_cast<uint8_t>((soc >> 8) & 0xFF);
  out[2] = static_cast<uint8_t>(soh & 0xFF);  out[3] = static_cast<uint8_t>((soh >> 8) & 0xFF);
  out[4] = static_cast<uint8_t>(cap & 0xFF);  out[5] = static_cast<uint8_t>((cap >> 8) & 0xFF);
}

void build_0x356(const SafetyState& s, uint8_t out[8]) {
  memset(out, 0, 8);
  // V2.67 lines 2340-2341: v=int(avgVoltage*100), i=int(totalCurrent*10), t=int(avgTemp*10)
  // V3.2: current follows the dashboard's Combined Current (Battery Value Sources
  // fused value, shunt-led when fresh; BMS pack_current_total fallback otherwise)
  // via can_tx_current() — the current analogue of the can_tx_soc() 0x355 fix.
  // Fixes "Strom 0,0 A" on the inverter while the shunt saw real sub-0.5 A current
  // the BMS is blind to. Voltage/temperature stay on the raw BMS aggregates.
  int v = static_cast<int>(s.pack_voltage_avg     * 100.0f);
  int i = static_cast<int>(can_tx_current(s)      * 10.0f);
  int t = static_cast<int>(s.temp_avg             * 10.0f);
  out[0] = static_cast<uint8_t>(v & 0xFF);  out[1] = static_cast<uint8_t>((v >> 8) & 0xFF);
  out[2] = static_cast<uint8_t>(i & 0xFF);  out[3] = static_cast<uint8_t>((i >> 8) & 0xFF);
  out[4] = static_cast<uint8_t>(t & 0xFF);  out[5] = static_cast<uint8_t>((t >> 8) & 0xFF);
}

void build_0x359(const SafetyState& s, uint8_t out[8]) {
  // Alarm / warning / status frame. Bit layout is the STANDARD Pylontech LV
  // protocol as decoded by OpenDTU-onBattery — verified byte-for-byte against its
  // Pylontech provider (src/battery/pylontech/Provider.cpp), whose getBit(v, n)
  // is LSB0: (v & (1<<n)) >> n. This replaces an earlier invented layout whose
  // bit numbering was shifted and which re-derived alarm conditions locally
  // (CCL/DCL "over-current" proxies). That is what made a healthy 23.9 °C battery
  // report a false under-temperature (the old charge-OCP proxy `ccl<0.1` landed on
  // OpenDTU's under-temperature bit, byte0 bit4). Every bit here is now sourced
  // ONLY from SafetyState (alarm_flags / temp_alarm / packs_online) — the same
  // values runSafety() computes and the Diagnostics/Alerts page shows — so the CAN
  // alarms can never diverge from what the gateway itself believes.
  //
  // Byte 0 — alarms (protection), LSB0:
  //   bit1 over voltage, bit2 under voltage, bit3 over temperature,
  //   bit4 under temperature, bit7 discharge over-current.
  // Byte 1 — alarms (protection) 2, LSB0:
  //   bit0 charge over-current, bit3 BMS internal / system error.
  // Bytes 2-3 — warning-level equivalents (same bit map as byte0/1). The gateway
  //   has no warning-level thresholds distinct from its protection flags, so these
  //   stay zero (honest: we do not fabricate a warning we cannot compute).
  // Byte 4 — battery module count = packs online (OpenDTU "Module Count" /
  //   "Batteriemodule"). See the prior task; unchanged here.
  //
  // Not encoded: cell drift / imbalance (alarm_flags 0x20) has no standard
  // Pylontech 0x359 slot; it is surfaced via the gateway UI/MQTT, not invented
  // onto an unrelated bit. There is no BMS-sourced over-current flag in
  // alarm_flags, so byte0 bit7 / byte1 bit0 are only ever driven by a genuine
  // BMS-reported alarm below, never by CCL/DCL being at their limit (which is
  // already communicated via 0x351 current limits and 0x35C enable bits).
  memset(out, 0, 8);
  const uint8_t af = s.alarm_flags;

  // Byte 0 — alarms (protection)
  if (af & 0x02)            out[0] |= 0x02;  // bit1 over voltage
  if (af & 0x10)            out[0] |= 0x04;  // bit2 under voltage
  if (s.temp_alarm & 0x02)  out[0] |= 0x08;  // bit3 over temperature (hot)
  if (s.temp_alarm & 0x01)  out[0] |= 0x10;  // bit4 under temperature (cold)

  // Byte 1 — alarms (protection) 2
  // BMS-reported critical alarm (0x44) and "no packs online" both map to the
  // BMS-internal / system-error bit — a genuine fault the inverter should see.
  if ((af & 0x40) || (af & 0x80)) out[1] |= 0x08;  // bit3 BMS internal / system error

  // Byte 4 — battery module count = packs currently online
  out[4] = s.packs_online;
}

void build_0x35C(const SafetyState& s, uint8_t out[8]) {
  // Spec-derived: Pylontech LV BMS CAN Protocol v1.1, not present in V2.67.
  // Standard 0x35C byte-0 layout (as decoded by Victron/Deye/SMA/OpenDTU):
  //   bit7 = charge enable, bit6 = discharge enable, bit5 = request force charge.
  //
  // V3.2 fix: bit6 and bit5 were previously swapped (discharge-enable was written
  // to bit5 and force-charge to bit6). Against the standard decode this made a
  // healthy battery report discharge-enable=No (bit6 clear) and force-charge=Yes
  // (bit5 set) — a false discharge lockout plus a bogus immediate-charge request,
  // and on the wrong sign of the safety state. Bits are now on their spec bits.
  memset(out, 0, 8);
  if (s.ccl_amps >= 0.1f)   out[0] |= 0x80;  // bit7 charge enable
  if (s.dcl_amps >= 0.1f)   out[0] |= 0x40;  // bit6 discharge enable
  if (s.alarm_flags & 0x10) out[0] |= 0x20;  // bit5 request force charge on undervolt
}

void build_0x35E(uint8_t out[8]) {
  // V2.67 line 2347: manufacturer ID for Pylontech protocol.
  memcpy(out, "PYLON   ", 8);
}

bool send_all_pylontech(const SafetyState& state) {
  // Send order: 0x351, 0x355, 0x356, 0x359, 0x35C, 0x35E.
  uint8_t d[8];
  bool ok = true;
  build_0x351(state, d);  ok &= can::tx::enqueue(0x351, d);
  build_0x355(state, d);  ok &= can::tx::enqueue(0x355, d);
  build_0x356(state, d);  ok &= can::tx::enqueue(0x356, d);
  build_0x359(state, d);  ok &= can::tx::enqueue(0x359, d);
  build_0x35C(state, d);  ok &= can::tx::enqueue(0x35C, d);
  build_0x35E(d);          ok &= can::tx::enqueue(0x35E, d);
  return ok;
}

}  // namespace can::pylontech
