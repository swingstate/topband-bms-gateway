// TopBand BMS protocol parsing.
//
// Based on the byte-layout documentation in linedot's reverse-engineering work:
//   https://github.com/linedot/topbands-bms
//
// The implementation here is an independent clean rewrite for ESP-IDF.
// Original layout research and enum names credited to linedot.
// See THIRD_PARTY_NOTICES.md.
//
// Wire format note: the TopBand BMS communicates over RS485 using an ASCII-hex
// encoded protocol.  Every byte of VER/ADR/CID1/RTN/LENID/INFO is transmitted
// as two ASCII hex characters.  SOI (0x7E = '~') and EOI (0x0D = CR) are the
// only raw binary framing bytes.  All parsers in this file receive the raw
// UART byte stream and handle ASCII decoding internally.

#pragma once
#include <cstddef>
#include <cstdint>

namespace bms::protocol {

// ── Protocol framing constants ─────────────────────────────────────────────
constexpr uint8_t TB_SOI              = 0x7E;  // '~'
constexpr uint8_t TB_EOI              = 0x0D;  // CR
constexpr uint8_t TB_PROTOCOL_VERSION = 0x21;
constexpr uint8_t TB_CID1_BATTERY_DATA = 0x46;

// ── CID2 request codes ──────────────────────────────────────────────────────
constexpr uint8_t TB_CID2_ANALOG_VALUES_FIXED_POINT = 0x42;
constexpr uint8_t TB_CID2_ALARM_INFO                = 0x44;
constexpr uint8_t TB_CID2_SYSTEM_PARAMETER          = 0x47;
constexpr uint8_t TB_CID2_BATTERY_DATE              = 0x4D;
constexpr uint8_t TB_CID2_MANUFACTURER_INFO         = 0x51;

// ── RTN status codes (response byte that replaces CID2 position) ────────────
constexpr uint8_t TB_RTN_OK                = 0x00;
constexpr uint8_t TB_RTN_VER_ERROR         = 0x01;
constexpr uint8_t TB_RTN_CHKSUM_ERROR      = 0x02;
constexpr uint8_t TB_RTN_LCHKSUM_ERROR     = 0x03;
constexpr uint8_t TB_RTN_CID2_UNSUPPORTED  = 0x04;
constexpr uint8_t TB_RTN_CMD_FORMAT_ERROR  = 0x05;
constexpr uint8_t TB_RTN_CMD_INVALID       = 0x06;

// ── Alarm bit flags ─────────────────────────────────────────────────────────
// Assembled from the status_bytes field in the 0x44 response, little-endian.
// Bit positions from linedot's research, transcribed as fresh code.
enum class TB_ALRMS : uint64_t {
  CELL_OVER_VOLTAGE_PROTECT         = 1ULL << 0,
  CELL_UNDER_VOLTAGE                = 1ULL << 1,
  CHARGE_OVER_CURRENT_PROTECT       = 1ULL << 2,
  CELL_OVER_VOLTAGE_ALARM           = 1ULL << 3,
  DISCHARGE_OVER_CURRENT1_PROTECT   = 1ULL << 4,
  CELL_TEMP_DISCHARGE_PROTECT       = 1ULL << 5,
  CELL_TEMP_CHARGE_PROTECT          = 1ULL << 6,
  PACK_UNDER_VOLTAGE_ALARM          = 1ULL << 7,
  OPEN_CURRENT_LIMIT                = 1ULL << 8,
  CHARGE_MOSFET_ON                  = 1ULL << 9,
  DISCHARGE_MOSFET_ON               = 1ULL << 10,
  SHORT_CIRCUIT_PROTECT             = 1ULL << 11,
  CELL_UNDER_VOLTAGE_PROTECT        = 1ULL << 12,
  PACK_UNDER_VOLTAGE_PROTECT        = 1ULL << 13,
  REVERSE_PROTECT                   = 1ULL << 14,
  SOC_LOW_ALARM                     = 1ULL << 15,
  CHARGER_CONNECTED                 = 1ULL << 20,
  DISCHARGING                       = 1ULL << 22,
  CHARGING                          = 1ULL << 23,
  CELL_LOW_FORCE_PROTECT            = 1ULL << 28,
  PACK_OVER_VOLTAGE_ALARM           = 1ULL << 40,
  MOS_NTC_TEMPERATURE_ALARM         = 1ULL << 41,
  ENV_TEMPERATURE_LOW_ALARM         = 1ULL << 42,
  ENV_TEMPERATURE_HIGH_ALARM        = 1ULL << 43,
  DISCHARGE_CURRENT_ALARM           = 1ULL << 46,
  CHARGE_CURRENT_ALARM              = 1ULL << 47,
  DISCHARGE_OVER_CURRENT2_PROTECT   = 1ULL << 56,
};

// ── Parse error codes ───────────────────────────────────────────────────────
enum class ParseError {
  Ok,
  BadSoi,
  BadEoi,
  BadVer,
  BadCid1,
  BadChecksum,
  TruncatedHeader,
  TruncatedPayload,
  BadRtn,
  AsciiToBinaryFailure,
  LengthMismatch,
  CellCountOutOfRange,
  TempCountOutOfRange,
};

// ── Parsed response structs ─────────────────────────────────────────────────

// CID2 = 0x42 — analog values (fixed-point).
// Temperatures: last three of temps[] are balancer/env/MOSFET when temp_count >= 3.
struct tb_analog_values_fixed_point {
  uint8_t  dataflag;            // raw DATAFLAG byte (matches BMS address)
  uint8_t  cell_count;          // 1..16
  float    cells[16];           // volts; unused slots = 0.0
  uint8_t  temp_count;          // total temp sensors
  float    temps[8];            // °C; last 3 are balancer/env/MOS when temp_count >= 3
  float    balancer_temp_c;     // temps[temp_count-3], 0 if temp_count < 3
  float    environment_temp_c;  // temps[temp_count-2], 0 if temp_count < 2
  float    mosfet_temp_c;       // temps[temp_count-1], 0 if temp_count < 1
  float    pack_current;        // A, signed; + = charge (V2.67 convention)
  float    pack_voltage;        // V
  float    rem_ah;              // Ah remaining
  float    full_ah;             // Ah design capacity
  uint16_t cycles;              // charge cycle count
  uint8_t  soc;                 // State of Charge %
  uint8_t  soh;                 // State of Health %
};

// CID2 = 0x44 — alarm info.
struct tb_alarm_info {
  uint64_t alarm_bits;          // assembled LE from status_bytes
  uint8_t  cell_count;
  uint8_t  temp_count;
  uint8_t  cell_v_alarm[16];    // per-cell voltage status code
  uint8_t  cell_t_alarm[16];    // per-cell temp status (cell_temp_count = temp_count-3)
  uint8_t  balancer_temp_alarm;
  uint8_t  env_temp_alarm;
  uint8_t  mosfet_temp_alarm;
  uint8_t  charge_curr_alarm;
  uint8_t  module_v_alarm;
  uint8_t  status_count;        // how many status_bytes were present
};

// CID2 = 0x47 — system parameters.
struct tb_system_parameter {
  float cell_high_v;
  float cell_low_v;
  float cell_under_v;
  float module_high_v;
  float module_low_v;
  float module_under_v;
  float charge_high_t;
  float charge_low_t;
  float discharge_high_t;
  float discharge_low_t;
  float charge_max_a;
  float discharge_max_a;
};

// CID2 = 0x51 — manufacturer info.
// INFO field for 0x51 is raw ASCII (not hex-encoded).
struct tb_manufacturer_info {
  char hw[21];   // hardware version string, NUL-terminated (20 chars max)
  char sw[5];    // software version string, NUL-terminated (4 chars max)
  char id[33];   // manufacturer ID string, NUL-terminated (up to 32 chars)
};

// CID2 = 0x4D — battery date.
struct tb_date {
  uint16_t year;
  uint8_t  month;
  uint8_t  day;
  uint8_t  hour;
  uint8_t  minute;
  uint8_t  second;
};

// ── Public API ──────────────────────────────────────────────────────────────

// Parse and validate a complete response frame from the UART byte stream.
// buf must include SOI (0x7E) and EOI (0x0D).
// On success: bms_id_out = ADR value, rtn_out = RTN status code,
//             payload_out points into buf at the start of the ASCII hex INFO
//             chars, payload_len_out = lenid (number of ASCII INFO chars =
//             2 × number of decoded INFO bytes).
// The payload pointer is valid as long as buf is alive; no allocation occurs.
ParseError parse_response_header(const uint8_t* buf, size_t len,
                                 uint8_t& bms_id_out, uint8_t& rtn_out,
                                 const uint8_t** payload_out,
                                 size_t& payload_len_out);

// Per-CID2 interpreters.  payload / payload_len come directly from
// parse_response_header.  Caller must know which CID2 was requested
// (no CID2 echo in the response — caller tracks request/response pairing).
ParseError interpret_analog_values_fixed_point(const uint8_t* payload,
                                               size_t payload_len,
                                               tb_analog_values_fixed_point& out);

ParseError interpret_alarm_info(const uint8_t* payload, size_t payload_len,
                                tb_alarm_info& out);

ParseError interpret_system_parameter(const uint8_t* payload, size_t payload_len,
                                      tb_system_parameter& out);

// For 0x51 the INFO field is raw ASCII, not hex-encoded: payload bytes are
// the string characters directly.
ParseError interpret_manufacturer_info(const uint8_t* payload, size_t payload_len,
                                       tb_manufacturer_info& out);

ParseError interpret_date(const uint8_t* payload, size_t payload_len,
                          tb_date& out);

// Build a request frame into out_buf (includes SOI and EOI).
// info_field / info_field_len are the binary INFO bytes to encode.
// Returns bytes written, or 0 if out_buf_size is too small.
size_t build_request(uint8_t bms_address, uint8_t cid2,
                     const uint8_t* info_field, size_t info_field_len,
                     uint8_t* out_buf, size_t out_buf_size);

} // namespace bms::protocol
