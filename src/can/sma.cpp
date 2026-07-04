#include "can/sma.h"
#include "can/tx.h"
#include <cstring>

// V2.67 reference: legacy/Topband_BMS_Gateway_v2_67_2.ino lines 2332-2350
// sendVictronCAN() with g_can_protocol==2.
//
// Frames 0x351/0x355/0x356 are byte-identical to Victron.
// Frame 0x35A carries the Victron alarm bits plus two SMA-specific bits
//   from V2.67 line 2345:
//     if(g_can_protocol==2 && maxChargeCurrent < 0.1)    d[4] |= 0x02;
//     if(g_can_protocol==2 && maxDischargeCurrent < 0.1) d[4] |= 0x01;
// Frame 0x35E uses "SMA     " (V2.67 line 2348).
// Frame 0x35B is spec-derived (SMA Sunny Island BMS protocol); not in V2.67.

namespace can::sma {

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

void build_0x35A(const SafetyState& s, uint8_t out[8]) {
  memset(out, 0, 8);
  // V2.67 lines 2344-2345: Victron alarm mapping + SMA charge/discharge enable bits.
  // Alarm bits (identical to Victron):
  uint8_t af = s.alarm_flags;
  if (af & 0x01) out[4] |= 0x80;  // critical alarm → high-priority alarm
  if (af & 0x40) out[4] |= 0x80;  // BMS reported alarm → high-priority alarm
  if (af & 0x02) out[4] |= 0x40;  // overvolt alarm
  if (af & 0x08) out[4] |= 0x20;  // temperature stop
  if (af & 0x10) out[4] |= 0x10;  // undervolt alarm
  // SMA-specific charge/discharge enable bits (V2.67 line 2345):
  // bit1 set = charge disabled (CCL dropped to zero); bit0 set = discharge disabled.
  if (s.ccl_amps < 0.1f) out[4] |= 0x02;
  if (s.dcl_amps < 0.1f) out[4] |= 0x01;
}

void build_0x35B(const SafetyState& s, uint8_t out[8]) {
  // Spec-derived: SMA Sunny Island BMS protocol, not present in V2.67.
  // Warning bitmap — softer conditions that don't yet cut off charge/discharge.
  // Byte 4: bit6 = cell imbalance warning (alarm_flags 0x20).
  //         bit5 = temperature warning (temperature stop active implies warn too).
  //         bit4 = low SOC / undervolt advisory.
  memset(out, 0, 8);
  uint8_t af = s.alarm_flags;
  if (af & 0x20) out[4] |= 0x40;  // cell drift / imbalance warning
  if (af & 0x08) out[4] |= 0x20;  // temperature warning (accompanies temp stop)
  if (af & 0x10) out[4] |= 0x10;  // undervolt advisory
}

void build_0x35E(uint8_t out[8]) {
  // V2.67 line 2348: manufacturer ID for SMA protocol.
  memcpy(out, "SMA     ", 8);
}

bool send_all_sma(const SafetyState& state) {
  // Send order: 0x351, 0x355, 0x356, 0x35A, 0x35B, 0x35E.
  uint8_t d[8];
  bool ok = true;
  build_0x351(state, d);  ok &= can::tx::enqueue(0x351, d);
  build_0x355(state, d);  ok &= can::tx::enqueue(0x355, d);
  build_0x356(state, d);  ok &= can::tx::enqueue(0x356, d);
  build_0x35A(state, d);  ok &= can::tx::enqueue(0x35A, d);
  build_0x35B(state, d);  ok &= can::tx::enqueue(0x35B, d);
  build_0x35E(d);          ok &= can::tx::enqueue(0x35E, d);
  return ok;
}

}  // namespace can::sma
