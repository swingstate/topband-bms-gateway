// TopBand BMS protocol parsing.
//
// Based on the byte-layout documentation in linedot's reverse-engineering work:
//   https://github.com/linedot/topbands-bms
//
// The implementation here is an independent clean rewrite for ESP-IDF.
// Original layout research and enum names credited to linedot.
// See THIRD_PARTY_NOTICES.md.

#include "bms/protocol.h"
#include <cstring>

// ── Wire format reference ──────────────────────────────────────────────────
//
// All multi-byte integers are big-endian within decoded INFO fields.
//
// Complete response frame on RS485 wire (ASCII hex encoded):
//   SOI(1=0x7E) | VER(2) | ADR(2) | CID1(2) | RTN(2) | LENID(4) |
//   INFO(lenid chars) | CHKSUM(4) | EOI(1=0x0D)
//
// All bytes after SOI and before EOI are ASCII hex characters ('0'-'9','A'-'F').
// SOI = 0x7E ('~'), EOI = 0x0D (CR).
//
// LENID (4 ASCII chars, 16-bit value):
//   bits 11-0 = lenid = number of ASCII chars in INFO field (= 2 × decoded bytes)
//   bits 15-12 = LCHKSUM = ((~sum_of_nibbles(bits11-0)) + 1) & 0xF
//
// CHKSUM (4 ASCII chars, 16-bit value):
//   sum of ASCII byte values from VER through end of INFO (12 + lenid bytes)
//   checksum = (~sum + 1) & 0xFFFF
//
// Cell voltage:  raw uint16 × 0.001  → volts  (0x0D06 = 3334 → 3.334 V)
// Pack voltage:  raw uint16 × 0.01   → volts  (0x1389 = 5001 → 50.01 V)
// Current:       raw int16  × 0.01   → amps   (positive = charge, V2.67 convention)
// Temperature:   (raw - 2731) / 10.0 → °C     (0x0B80 = 2944 → 21.3 °C)
// Capacity:      raw uint16 × 0.01   → Ah

namespace bms::protocol {

// ── ASCII hex helpers ───────────────────────────────────────────────────────

static uint8_t nibble(uint8_t c) {
  if (c >= '0' && c <= '9') return (uint8_t)(c - '0');
  if (c >= 'A' && c <= 'F') return (uint8_t)(c - 'A' + 10);
  if (c >= 'a' && c <= 'f') return (uint8_t)(c - 'a' + 10);
  return 0xFF;  // invalid
}

// Decode one byte from two ASCII hex chars; returns false on invalid input.
static bool decode_byte(uint8_t hi, uint8_t lo, uint8_t& out) {
  uint8_t h = nibble(hi);
  uint8_t l = nibble(lo);
  if (h == 0xFF || l == 0xFF) return false;
  out = (uint8_t)((h << 4) | l);
  return true;
}

// Decode uint16 from four ASCII hex chars (big-endian).
static bool decode_u16(const uint8_t* p, uint16_t& out) {
  uint8_t b0, b1;
  if (!decode_byte(p[0], p[1], b0)) return false;
  if (!decode_byte(p[2], p[3], b1)) return false;
  out = (uint16_t)((b0 << 8) | b1);
  return true;
}

// ── Payload read helpers ────────────────────────────────────────────────────
// payload is ASCII hex chars; index i addresses decoded byte i (pair 2i, 2i+1).

static bool payload_byte(const uint8_t* p, size_t byte_count, size_t i, uint8_t& out) {
  if (i >= byte_count) return false;
  return decode_byte(p[2 * i], p[2 * i + 1], out);
}

static bool payload_u16(const uint8_t* p, size_t byte_count, size_t i, uint16_t& out) {
  if (i + 1 >= byte_count) return false;
  uint8_t b0, b1;
  if (!decode_byte(p[2 * i],     p[2 * i + 1], b0)) return false;
  if (!decode_byte(p[2 * i + 2], p[2 * i + 3], b1)) return false;
  out = (uint16_t)((b0 << 8) | b1);
  return true;
}

static bool payload_s16(const uint8_t* p, size_t byte_count, size_t i, int16_t& out) {
  uint16_t raw;
  if (!payload_u16(p, byte_count, i, raw)) return false;
  out = (int16_t)raw;
  return true;
}

// ── LENID encoding / verification ──────────────────────────────────────────

static uint16_t build_lenid(uint16_t lenid) {
  lenid &= 0x0FFF;
  uint16_t s = (uint16_t)(((lenid >> 8) & 0xF) + ((lenid >> 4) & 0xF) + (lenid & 0xF));
  uint16_t lchk = (uint16_t)((~(s & 0xF) + 1) & 0xF);
  return (uint16_t)((lchk << 12) | lenid);
}

static bool check_lenid(uint16_t field, uint16_t& lenid_out) {
  uint16_t lenid = field & 0x0FFF;
  uint16_t lchk  = (field >> 12) & 0xF;
  uint16_t s     = (uint16_t)(((lenid >> 8) & 0xF) + ((lenid >> 4) & 0xF) + (lenid & 0xF));
  uint16_t expected = (uint16_t)((~(s & 0xF) + 1) & 0xF);
  if (lchk != expected) return false;
  lenid_out = lenid;
  return true;
}

// ── Checksum ────────────────────────────────────────────────────────────────
// Sum ASCII byte values from VER through end of INFO (positions 1..12+lenid in
// the full wire frame starting at SOI).  Checksum = (~sum + 1) & 0xFFFF.

static uint16_t compute_checksum(const uint8_t* buf, uint16_t lenid) {
  // buf[0] = SOI; sum starts at buf[1]
  uint32_t sum = 0;
  for (size_t i = 1; i < (size_t)(13 + lenid); i++) {
    sum += buf[i];
  }
  return (uint16_t)((~(sum & 0xFFFF) + 1) & 0xFFFF);
}

// ── ASCII hex output helpers ────────────────────────────────────────────────

static const char HEX_CHARS[] = "0123456789ABCDEF";

static void emit_byte(uint8_t val, uint8_t* out) {
  out[0] = (uint8_t)HEX_CHARS[val >> 4];
  out[1] = (uint8_t)HEX_CHARS[val & 0xF];
}

static void emit_u16(uint16_t val, uint8_t* out) {
  emit_byte((uint8_t)(val >> 8),   out);
  emit_byte((uint8_t)(val & 0xFF), out + 2);
}

// ── parse_response_header ───────────────────────────────────────────────────

ParseError parse_response_header(const uint8_t* buf, size_t len,
                                 uint8_t& bms_id_out, uint8_t& rtn_out,
                                 const uint8_t** payload_out,
                                 size_t& payload_len_out) {
  // Minimum frame: SOI + 12 header ASCII chars + 4 CHKSUM + EOI = 18 bytes
  if (len < 18) return ParseError::TruncatedHeader;
  if (buf[0] != TB_SOI) return ParseError::BadSoi;

  // Decode header fields (each is 2 ASCII hex chars)
  uint8_t ver, adr, cid1, rtn;
  if (!decode_byte(buf[1], buf[2], ver))  return ParseError::AsciiToBinaryFailure;
  if (!decode_byte(buf[3], buf[4], adr))  return ParseError::AsciiToBinaryFailure;
  if (!decode_byte(buf[5], buf[6], cid1)) return ParseError::AsciiToBinaryFailure;
  if (!decode_byte(buf[7], buf[8], rtn))  return ParseError::AsciiToBinaryFailure;

  if (ver != TB_PROTOCOL_VERSION) return ParseError::BadVer;
  if (cid1 != TB_CID1_BATTERY_DATA) return ParseError::BadCid1;

  // LENID (4 ASCII chars)
  uint16_t lenid_field;
  if (!decode_u16(buf + 9, lenid_field)) return ParseError::AsciiToBinaryFailure;
  uint16_t lenid;
  if (!check_lenid(lenid_field, lenid)) return ParseError::BadChecksum;

  // Full frame length = SOI(1) + header(12) + INFO(lenid) + CHKSUM(4) + EOI(1)
  size_t expected = (size_t)(18 + lenid);
  if (len < expected) return ParseError::TruncatedPayload;

  // Verify EOI
  if (buf[expected - 1] != TB_EOI) return ParseError::BadEoi;

  // Verify checksum (covers INFO chars regardless of whether they are hex or raw ASCII;
  // CID2=0x51 manufacturer frames use raw ASCII in the INFO field, not hex pairs)
  uint16_t rx_chk;
  if (!decode_u16(buf + 13 + lenid, rx_chk)) return ParseError::AsciiToBinaryFailure;
  uint16_t exp_chk = compute_checksum(buf, lenid);
  if (rx_chk != exp_chk) return ParseError::BadChecksum;

  bms_id_out      = adr;
  rtn_out         = rtn;
  *payload_out    = buf + 13;
  payload_len_out = lenid;
  return ParseError::Ok;
}

// ── interpret_analog_values_fixed_point ────────────────────────────────────
// INFO layout (decoded bytes):
//   [0]           DATAFLAG
//   [1]           cell_count (1..16)
//   [2..2+nc*2-1] CELL_VOLTAGES  (nc×uint16, /1000 V)
//   [2+nc*2]      temp_count (1..8)
//   [3+nc*2 .. 2+nc*2+nt*2] TEMPS (nt×uint16, (raw-2731)/10 °C)
//   +2            CURRENT  (int16, /100 A)
//   +2            PACK_VOLTAGE (uint16, /100 V)
//   +2            REM_AH (uint16, /100 Ah)
//   +1            skipped (undocumented status byte)
//   +2            FULL_AH (uint16, /100 Ah)
//   +2            CYCLES  (uint16)
//   +1            SOC (uint8, %)
//   +1            SOH (uint8, %)

ParseError interpret_analog_values_fixed_point(const uint8_t* payload,
                                               size_t payload_len,
                                               tb_analog_values_fixed_point& out) {
  memset(&out, 0, sizeof(out));

  if (payload_len < 4) return ParseError::TruncatedPayload;
  if ((payload_len & 1) != 0) return ParseError::LengthMismatch;

  const size_t byte_count = payload_len / 2;

  uint8_t dataflag, cell_count;
  if (!payload_byte(payload, byte_count, 0, dataflag))   return ParseError::TruncatedPayload;
  if (!payload_byte(payload, byte_count, 1, cell_count)) return ParseError::TruncatedPayload;
  if (cell_count == 0 || cell_count > 16) return ParseError::CellCountOutOfRange;

  out.dataflag    = dataflag;
  out.cell_count  = cell_count;

  // Cell voltages
  size_t pos = 2;
  if (pos + cell_count * 2 > byte_count + 1) return ParseError::TruncatedPayload;
  for (uint8_t i = 0; i < cell_count; i++) {
    uint16_t raw;
    if (!payload_u16(payload, byte_count, pos, raw)) return ParseError::TruncatedPayload;
    out.cells[i] = (float)raw * 0.001f;
    pos += 2;
  }

  // Temp count + temps
  uint8_t temp_count;
  if (!payload_byte(payload, byte_count, pos, temp_count)) return ParseError::TruncatedPayload;
  if (temp_count > 8) return ParseError::TempCountOutOfRange;
  out.temp_count = temp_count;
  pos++;

  if (pos + temp_count * 2 > byte_count) return ParseError::TruncatedPayload;
  for (uint8_t i = 0; i < temp_count; i++) {
    uint16_t raw;
    if (!payload_u16(payload, byte_count, pos, raw)) return ParseError::TruncatedPayload;
    out.temps[i] = ((float)raw - 2731.0f) / 10.0f;
    pos += 2;
  }

  // Extract named temperature aliases (last 3 are balancer/env/MOS)
  if (temp_count >= 3) {
    out.balancer_temp_c    = out.temps[temp_count - 3];
    out.environment_temp_c = out.temps[temp_count - 2];
    out.mosfet_temp_c      = out.temps[temp_count - 1];
  } else if (temp_count == 2) {
    out.environment_temp_c = out.temps[0];
    out.mosfet_temp_c      = out.temps[1];
  } else if (temp_count == 1) {
    out.mosfet_temp_c = out.temps[0];
  }

  // Minimum remaining: CURRENT(2) + PACK_V(2) = 4 bytes
  if (pos + 4 > byte_count) return ParseError::TruncatedPayload;

  int16_t current_raw;
  if (!payload_s16(payload, byte_count, pos, current_raw)) return ParseError::TruncatedPayload;
  out.pack_current = (float)current_raw * 0.01f;
  pos += 2;

  uint16_t pack_v_raw;
  if (!payload_u16(payload, byte_count, pos, pack_v_raw)) return ParseError::TruncatedPayload;
  out.pack_voltage = (float)pack_v_raw * 0.01f;
  pos += 2;

  // Extended fields (rem_ah, skip, full_ah, cycles, soc, soh) — present when enough data
  if (pos + 5 <= byte_count) {
    uint16_t rem_raw;
    if (!payload_u16(payload, byte_count, pos, rem_raw)) return ParseError::TruncatedPayload;
    out.rem_ah = (float)rem_raw * 0.01f;
    pos += 3;  // +2 for rem_ah, +1 skip (undocumented byte follows rem_ah)

    if (pos + 2 <= byte_count) {
      uint16_t full_raw;
      if (!payload_u16(payload, byte_count, pos, full_raw)) return ParseError::TruncatedPayload;
      out.full_ah = (float)full_raw * 0.01f;
      pos += 2;
    }

    if (pos + 2 <= byte_count) {
      uint16_t cyc;
      if (!payload_u16(payload, byte_count, pos, cyc)) return ParseError::TruncatedPayload;
      out.cycles = cyc;
      pos += 2;
    }

    if (pos < byte_count) {
      uint8_t soc;
      if (!payload_byte(payload, byte_count, pos, soc)) return ParseError::TruncatedPayload;
      out.soc = soc;
      pos++;
    }

    if (pos < byte_count) {
      uint8_t soh;
      if (!payload_byte(payload, byte_count, pos, soh)) return ParseError::TruncatedPayload;
      out.soh = soh;
    }
  }

  return ParseError::Ok;
}

// ── interpret_alarm_info ────────────────────────────────────────────────────
// INFO layout (decoded bytes):
//   [0]               DATAFLAG
//   [1]               cell_count
//   [2..2+nc-1]       CELL_VOLTAGE_STATUS (1 byte per cell)
//   [2+nc]            temp_count
//   [3+nc..2+nc+ctn]  CELL_TEMP_STATUS (ctn = max(0, temp_count-3) bytes)
//   [3+nc+ctn]        BALANCER_TEMP_STATUS
//   [+1]              ENV_TEMP_STATUS
//   [+1]              MOSFET_TEMP_STATUS
//   [+1]              CHARGE_CURRENT_STATUS
//   [+1]              MODULE_VOLTAGE_STATUS
//   [+1]              STATUS_COUNT (N bytes of 64-bit alarm bits follow, LE)
//   [+N]              STATUS_BYTES (little-endian alarm bit bytes)

ParseError interpret_alarm_info(const uint8_t* payload, size_t payload_len,
                                tb_alarm_info& out) {
  memset(&out, 0, sizeof(out));

  if ((payload_len & 1) != 0) return ParseError::LengthMismatch;
  const size_t byte_count = payload_len / 2;
  if (byte_count < 8) return ParseError::TruncatedPayload;

  size_t pos = 1;  // skip DATAFLAG

  uint8_t cell_count;
  if (!payload_byte(payload, byte_count, pos, cell_count)) return ParseError::TruncatedPayload;
  if (cell_count > 16) return ParseError::CellCountOutOfRange;
  out.cell_count = cell_count;
  pos++;

  for (uint8_t i = 0; i < cell_count; i++) {
    if (!payload_byte(payload, byte_count, pos, out.cell_v_alarm[i])) return ParseError::TruncatedPayload;
    pos++;
  }

  uint8_t temp_count;
  if (!payload_byte(payload, byte_count, pos, temp_count)) return ParseError::TruncatedPayload;
  out.temp_count = temp_count;
  pos++;

  uint8_t cell_temp_bytes = (temp_count >= 3) ? (uint8_t)(temp_count - 3) : 0;
  if (cell_temp_bytes > 16) cell_temp_bytes = 16;
  for (uint8_t i = 0; i < cell_temp_bytes; i++) {
    if (!payload_byte(payload, byte_count, pos, out.cell_t_alarm[i])) return ParseError::TruncatedPayload;
    pos++;
  }

  if (pos + 3 > byte_count) return ParseError::TruncatedPayload;
  if (!payload_byte(payload, byte_count, pos,     out.balancer_temp_alarm)) return ParseError::TruncatedPayload;
  if (!payload_byte(payload, byte_count, pos + 1, out.env_temp_alarm))      return ParseError::TruncatedPayload;
  if (!payload_byte(payload, byte_count, pos + 2, out.mosfet_temp_alarm))   return ParseError::TruncatedPayload;
  pos += 3;

  if (pos + 3 > byte_count) return ParseError::TruncatedPayload;
  if (!payload_byte(payload, byte_count, pos,     out.charge_curr_alarm)) return ParseError::TruncatedPayload;
  if (!payload_byte(payload, byte_count, pos + 1, out.module_v_alarm))    return ParseError::TruncatedPayload;
  uint8_t status_count;
  if (!payload_byte(payload, byte_count, pos + 2, status_count))          return ParseError::TruncatedPayload;
  out.status_count = status_count;
  pos += 3;

  uint8_t status_bytes = (status_count > 8) ? 8 : status_count;
  if (pos + status_bytes > byte_count) return ParseError::TruncatedPayload;
  out.alarm_bits = 0;
  for (uint8_t i = 0; i < status_bytes; i++) {
    uint8_t b;
    if (!payload_byte(payload, byte_count, pos + i, b)) return ParseError::TruncatedPayload;
    out.alarm_bits |= ((uint64_t)b << (8 * i));
  }

  return ParseError::Ok;
}

// ── interpret_system_parameter ──────────────────────────────────────────────
// INFO layout (decoded bytes, 24 required):
//   [0..1]   cell_high_v   (uint16, /1000 V)
//   [2..3]   cell_low_v
//   [4..5]   cell_under_v
//   [6..7]   charge_high_t (uint16, (raw-2731)/10 °C)
//   [8..9]   charge_low_t
//   [10..11] charge_max_a  (int16, /100 A)
//   [12..13] module_high_v (uint16, /100 V)
//   [14..15] module_low_v
//   [16..17] module_under_v
//   [18..19] discharge_high_t
//   [20..21] discharge_low_t
//   [22..23] discharge_max_a (int16, /100 A)

ParseError interpret_system_parameter(const uint8_t* payload, size_t payload_len,
                                      tb_system_parameter& out) {
  memset(&out, 0, sizeof(out));

  if ((payload_len & 1) != 0) return ParseError::LengthMismatch;
  const size_t byte_count = payload_len / 2;
  if (byte_count < 24) return ParseError::TruncatedPayload;

  auto get_u16 = [&](size_t i, float scale) -> float {
    uint16_t v = 0;
    payload_u16(payload, byte_count, i, v);
    return (float)v * scale;
  };
  auto get_s16 = [&](size_t i) -> float {
    int16_t v = 0;
    payload_s16(payload, byte_count, i, v);
    return (float)v * 0.01f;
  };
  auto kelvin10_to_c = [&](size_t i) -> float {
    uint16_t v = 0;
    payload_u16(payload, byte_count, i, v);
    return ((float)v - 2731.0f) / 10.0f;
  };

  out.cell_high_v        = get_u16(0,  0.001f);
  out.cell_low_v         = get_u16(2,  0.001f);
  out.cell_under_v       = get_u16(4,  0.001f);
  out.charge_high_t      = kelvin10_to_c(6);
  out.charge_low_t       = kelvin10_to_c(8);
  out.charge_max_a       = get_s16(10);
  out.module_high_v      = get_u16(12, 0.01f);
  out.module_low_v       = get_u16(14, 0.01f);
  out.module_under_v     = get_u16(16, 0.01f);
  out.discharge_high_t   = kelvin10_to_c(18);
  out.discharge_low_t    = kelvin10_to_c(20);
  out.discharge_max_a    = get_s16(22);

  return ParseError::Ok;
}

// ── interpret_manufacturer_info ─────────────────────────────────────────────
// The 0x51 INFO field is raw ASCII, NOT hex-encoded: each payload byte is a
// string character.  payload_len is the byte count directly (= lenid = char count).
// Layout: hw[20] | sw[4] | id[remaining]

ParseError interpret_manufacturer_info(const uint8_t* payload, size_t payload_len,
                                       tb_manufacturer_info& out) {
  memset(&out, 0, sizeof(out));
  if (payload_len < 24) return ParseError::TruncatedPayload;

  size_t hw_len = (payload_len >= 20) ? 20 : payload_len;
  for (size_t i = 0; i < hw_len; i++) out.hw[i] = (char)payload[i];
  // Trim trailing spaces
  for (int i = (int)hw_len - 1; i >= 0 && out.hw[i] == ' '; i--) out.hw[i] = '\0';

  size_t sw_len = (payload_len >= 24) ? 4 : (payload_len - 20);
  for (size_t i = 0; i < sw_len; i++) out.sw[i] = (char)payload[20 + i];
  for (int i = (int)sw_len - 1; i >= 0 && out.sw[i] == ' '; i--) out.sw[i] = '\0';

  if (payload_len > 24) {
    size_t id_len = payload_len - 24;
    if (id_len > 32) id_len = 32;
    for (size_t i = 0; i < id_len; i++) out.id[i] = (char)payload[24 + i];
    for (int i = (int)id_len - 1; i >= 0 && out.id[i] == ' '; i--) out.id[i] = '\0';
  }

  return ParseError::Ok;
}

// ── interpret_date ──────────────────────────────────────────────────────────
// INFO layout (decoded bytes):
//   [0..1] year (uint16)
//   [2]    month
//   [3]    day
//   [4]    hour
//   [5]    minute
//   [6]    second

ParseError interpret_date(const uint8_t* payload, size_t payload_len,
                          tb_date& out) {
  memset(&out, 0, sizeof(out));

  if ((payload_len & 1) != 0) return ParseError::LengthMismatch;
  const size_t byte_count = payload_len / 2;
  if (byte_count < 7) return ParseError::TruncatedPayload;

  uint16_t year;
  if (!payload_u16(payload, byte_count, 0, year)) return ParseError::TruncatedPayload;
  out.year = year;

  uint8_t month, day, hour, minute, second;
  if (!payload_byte(payload, byte_count, 2, month))   return ParseError::TruncatedPayload;
  if (!payload_byte(payload, byte_count, 3, day))     return ParseError::TruncatedPayload;
  if (!payload_byte(payload, byte_count, 4, hour))    return ParseError::TruncatedPayload;
  if (!payload_byte(payload, byte_count, 5, minute))  return ParseError::TruncatedPayload;
  if (!payload_byte(payload, byte_count, 6, second))  return ParseError::TruncatedPayload;

  out.month  = month;
  out.day    = day;
  out.hour   = hour;
  out.minute = minute;
  out.second = second;
  return ParseError::Ok;
}

// ── build_request ────────────────────────────────────────────────────────────
// Builds a complete ASCII hex request frame into out_buf including SOI/EOI.
// info_field bytes are hex-encoded into the INFO field.

size_t build_request(uint8_t bms_address, uint8_t cid2,
                     const uint8_t* info_field, size_t info_field_len,
                     uint8_t* out_buf, size_t out_buf_size) {
  // lenid = 2 × info_field_len (ASCII chars for INFO)
  if (info_field_len > 0x7FF) return 0;  // would overflow 12-bit lenid
  uint16_t lenid = (uint16_t)(info_field_len * 2);
  uint16_t lenid_field = build_lenid(lenid);

  // Frame size = SOI(1) + 12 header + lenid INFO + 4 CHKSUM + EOI(1)
  size_t frame_len = (size_t)(18 + lenid);
  if (out_buf_size < frame_len) return 0;

  uint8_t* p = out_buf;

  *p++ = TB_SOI;
  emit_byte(TB_PROTOCOL_VERSION, p); p += 2;
  emit_byte(bms_address,         p); p += 2;
  emit_byte(TB_CID1_BATTERY_DATA, p); p += 2;
  emit_byte(cid2,                p); p += 2;
  emit_u16(lenid_field,          p); p += 4;

  for (size_t i = 0; i < info_field_len; i++) {
    emit_byte(info_field[i], p);
    p += 2;
  }

  uint16_t chk = compute_checksum(out_buf, lenid);
  emit_u16(chk, p); p += 4;
  *p++ = TB_EOI;

  return frame_len;
}

} // namespace bms::protocol
