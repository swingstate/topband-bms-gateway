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
  int cv  = static_cast<int>(s.cvl_volts * 10.0f);
  int ccl = static_cast<int>(s.ccl_amps  * 10.0f);
  int dcl = static_cast<int>(s.dcl_amps  * 10.0f);
  out[0] = static_cast<uint8_t>(cv  & 0xFF);  out[1] = static_cast<uint8_t>((cv  >> 8) & 0xFF);
  out[2] = static_cast<uint8_t>(ccl & 0xFF);  out[3] = static_cast<uint8_t>((ccl >> 8) & 0xFF);
  out[4] = static_cast<uint8_t>(dcl & 0xFF);  out[5] = static_cast<uint8_t>((dcl >> 8) & 0xFF);
}

void build_0x355(const SafetyState& s, uint8_t out[8]) {
  memset(out, 0, 8);
  // V2.67 lines 2337-2338: SOC=int(avgSOC), SOH=int(avgSOH), cap=int(totalCapacity*10)
  // V3.2 decision: intentionally s.soc_avg (BMS), never s.soc_display (shunt-fused).
  // See can/victron.cpp build_0x355() for the reasoning.
  int soc = static_cast<int>(s.soc_avg);
  int soh = static_cast<int>(s.soh_avg);
  int cap = static_cast<int>(s.capacity_total_ah * 10.0f);
  out[0] = static_cast<uint8_t>(soc & 0xFF);  out[1] = static_cast<uint8_t>((soc >> 8) & 0xFF);
  out[2] = static_cast<uint8_t>(soh & 0xFF);  out[3] = static_cast<uint8_t>((soh >> 8) & 0xFF);
  out[4] = static_cast<uint8_t>(cap & 0xFF);  out[5] = static_cast<uint8_t>((cap >> 8) & 0xFF);
}

void build_0x356(const SafetyState& s, uint8_t out[8]) {
  memset(out, 0, 8);
  // V2.67 lines 2340-2341: v=int(avgVoltage*100), i=int(totalCurrent*10), t=int(avgTemp*10)
  int v = static_cast<int>(s.pack_voltage_avg   * 100.0f);
  int i = static_cast<int>(s.pack_current_total * 10.0f);
  int t = static_cast<int>(s.temp_avg           * 10.0f);
  out[0] = static_cast<uint8_t>(v & 0xFF);  out[1] = static_cast<uint8_t>((v >> 8) & 0xFF);
  out[2] = static_cast<uint8_t>(i & 0xFF);  out[3] = static_cast<uint8_t>((i >> 8) & 0xFF);
  out[4] = static_cast<uint8_t>(t & 0xFF);  out[5] = static_cast<uint8_t>((t >> 8) & 0xFF);
}

void build_0x359(const SafetyState& s, uint8_t out[8]) {
  // Spec-derived: Pylontech LV BMS CAN Protocol v1.1, not present in V2.67.
  // Byte 0: protection (fault-level triggers cutoff)
  //   bit0 = cell OVP (alarm_flags 0x02)
  //   bit1 = cell UVP (alarm_flags 0x10)
  //   bit2 = charge OTP (alarm_flags 0x08 — temperature stop covers both charge+discharge)
  //   bit3 = discharge OTP (alarm_flags 0x08)
  //   bit4 = charge OCP  — no standalone V3.0 flag; CCL==0 is the proxy (mapped below)
  //   bit5 = discharge OCP — DCL==0 proxy
  //   bit7 = AFE / general critical (alarm_flags 0x01 | 0x40)
  // Byte 1: warning (softer)
  //   bit6 = cell drift / imbalance warning (alarm_flags 0x20)
  // Byte 2: system status
  //   bit0 = pack-level fault (alarm_flags 0x40)
  //   bit7 = system fault / no packs online (alarm_flags 0x80)
  memset(out, 0, 8);
  uint8_t af = s.alarm_flags;

  // Byte 0 — protection bits
  if (af & 0x02) out[0] |= 0x01;  // cell OVP
  if (af & 0x10) out[0] |= 0x02;  // cell UVP
  if (af & 0x08) out[0] |= 0x04;  // charge OTP
  if (af & 0x08) out[0] |= 0x08;  // discharge OTP
  if (s.ccl_amps < 0.1f) out[0] |= 0x10;  // charge OCP proxy: CCL driven to zero
  if (s.dcl_amps < 0.1f) out[0] |= 0x20;  // discharge OCP proxy: DCL driven to zero
  if (af & 0x01) out[0] |= 0x80;  // AFE / critical alarm
  if (af & 0x40) out[0] |= 0x80;  // BMS reported alarm → AFE bit

  // Byte 1 — warning bits
  if (af & 0x20) out[1] |= 0x40;  // cell drift / imbalance warning

  // Byte 2 — fault status
  if (af & 0x40) out[2] |= 0x01;  // BMS reported critical alarm
  if (af & 0x80) out[2] |= 0x80;  // no packs online = system fault
}

void build_0x35C(const SafetyState& s, uint8_t out[8]) {
  // Spec-derived: Pylontech LV BMS CAN Protocol v1.1, not present in V2.67.
  // Byte 0: bit7 = charge enable, bit6 = force charge (request), bit5 = discharge enable
  // Force charge is set when undervolt alarm is active — inverter should push current in.
  memset(out, 0, 8);
  if (s.ccl_amps >= 0.1f)  out[0] |= 0x80;  // charge enable
  if (s.alarm_flags & 0x10) out[0] |= 0x40;  // force charge on undervolt
  if (s.dcl_amps >= 0.1f)  out[0] |= 0x20;  // discharge enable
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
