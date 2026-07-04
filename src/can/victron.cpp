#include "can/victron.h"
#include "can/tx.h"
#include <cstring>

// V2.67 source reference: legacy/Topband_BMS_Gateway_v2_67_2.ino lines 2332-2350
// sendVictronCAN() — frame composition logic preserved verbatim.
//
// int() / static_cast<int>() truncates toward zero, matching C's (int) cast in V2.67.
// All multi-byte fields use little-endian byte order.

namespace can::victron {

void build_0x351(const SafetyState& s, uint8_t out[8]) {
  memset(out, 0, 8);
  // V2.67 line 2333: cv=int(cvl*10), ccl=int(ccl*10), dcl=int(dcl*10)
  int cv  = static_cast<int>(s.cvl_volts * 10.0f);
  int ccl = static_cast<int>(s.ccl_amps  * 10.0f);
  int dcl = static_cast<int>(s.dcl_amps  * 10.0f);
  out[0] = static_cast<uint8_t>(cv  & 0xFF);  out[1] = static_cast<uint8_t>((cv  >> 8) & 0xFF);
  out[2] = static_cast<uint8_t>(ccl & 0xFF);  out[3] = static_cast<uint8_t>((ccl >> 8) & 0xFF);
  out[4] = static_cast<uint8_t>(dcl & 0xFF);  out[5] = static_cast<uint8_t>((dcl >> 8) & 0xFF);
}

void build_0x355(const SafetyState& s, uint8_t out[8]) {
  memset(out, 0, 8);
  // V2.67 lines 2334-2335: SOC=int(avgSOC), SOH=int(avgSOH), cap=int(totalCapacity*10)
  // V3.2 decision: intentionally s.soc_avg (BMS), never s.soc_display (shunt-fused).
  // CAN TX feeds the inverter's own charge decisions, so it is treated as
  // safety-adjacent like charge-taper — kept on the established BMS path until
  // shunt behaviour is proven in the field. Revisit once V3.2 soaks; see
  // docs/research/v3.2-shunt-soc-fusion.md.
  int soc = static_cast<int>(s.soc_avg);
  int soh = static_cast<int>(s.soh_avg);
  int cap = static_cast<int>(s.capacity_total_ah * 10.0f);
  out[0] = static_cast<uint8_t>(soc & 0xFF);  out[1] = static_cast<uint8_t>((soc >> 8) & 0xFF);
  out[2] = static_cast<uint8_t>(soh & 0xFF);  out[3] = static_cast<uint8_t>((soh >> 8) & 0xFF);
  out[4] = static_cast<uint8_t>(cap & 0xFF);  out[5] = static_cast<uint8_t>((cap >> 8) & 0xFF);
}

void build_0x356(const SafetyState& s, uint8_t out[8]) {
  memset(out, 0, 8);
  // V2.67 lines 2336-2337: v=int(avgVoltage*100), i=int(totalCurrent*10), t=int(avgTemp*10)
  // current is signed (negative = discharge per V2.67 convention).
  // Signed 16-bit LE: i & 0xFF is low byte; (i >> 8) & 0xFF is high byte.
  // Arithmetic right-shift on ESP32 Xtensa LX7 is sign-extending, matching Python behavior.
  int v = static_cast<int>(s.pack_voltage_avg   * 100.0f);
  int i = static_cast<int>(s.pack_current_total * 10.0f);
  int t = static_cast<int>(s.temp_avg           * 10.0f);
  out[0] = static_cast<uint8_t>(v & 0xFF);  out[1] = static_cast<uint8_t>((v >> 8) & 0xFF);
  out[2] = static_cast<uint8_t>(i & 0xFF);  out[3] = static_cast<uint8_t>((i >> 8) & 0xFF);
  out[4] = static_cast<uint8_t>(t & 0xFF);  out[5] = static_cast<uint8_t>((t >> 8) & 0xFF);
}

void build_0x35A(const SafetyState& s, uint8_t out[8]) {
  memset(out, 0, 8);
  // V2.67 line 2338 — Victron-protocol alarm flag translation.
  // Bit 0x01 and 0x40 both map to byte4 bit 7 (high-priority alarm).
  // Bit 0x02 → byte4 bit 6 (overvolt alarm).
  // Bit 0x08 → byte4 bit 5 (temperature stop).
  // Bit 0x10 → byte4 bit 4 (undervolt alarm).
  // Bits 0x20 (imbalance) and 0x80 (no packs) have no Victron CAN mapping.
  uint8_t af = s.alarm_flags;
  if (af & 0x01) out[4] |= 0x80;
  if (af & 0x40) out[4] |= 0x80;
  if (af & 0x02) out[4] |= 0x40;
  if (af & 0x08) out[4] |= 0x20;
  if (af & 0x10) out[4] |= 0x10;
}

void build_0x35E(uint8_t out[8]) {
  // V2.67 line 2341: manufacturer ID for Victron protocol.
  memcpy(out, "TOPBAND ", 8);
}

bool send_all_victron(const SafetyState& state) {
  // V2.67 send order: 0x351, 0x355, 0x356, 0x35A, 0x35E.
  // V3.0 omits the delay(2) between frames — TWAI driver handles back-to-back enqueues.
  uint8_t d[8];
  bool ok = true;
  build_0x351(state, d);  ok &= can::tx::enqueue(0x351, d);
  build_0x355(state, d);  ok &= can::tx::enqueue(0x355, d);
  build_0x356(state, d);  ok &= can::tx::enqueue(0x356, d);
  build_0x35A(state, d);  ok &= can::tx::enqueue(0x35A, d);
  build_0x35E(d);          ok &= can::tx::enqueue(0x35E, d);
  return ok;
}

}  // namespace can::victron
