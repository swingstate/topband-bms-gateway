/*
 * TOPBAND BMS GATEWAY V2.63
 * =========================
 * ESP32/ESP32-S3 firmware that bridges TopBand LiFePO4 BMS battery packs to
 * Victron/Pylontech/SMA inverters via CAN bus, with a web dashboard,
 * MQTT integration, and Home Assistant auto-discovery.
 *
 * Hardware
 *   Runs on any ESP32 or ESP32-S3 with RS485 and CAN transceivers.
 *   Pin assignments are configured at runtime via the web UI.
 *   Board presets: Waveshare ESP32-S3, LilyGo T-CAN485, or Custom.
 *
 * Architecture
 *   Core 0 (rs485Task): RS485 BMS polling, CAN output, alarm/sysparam monitoring
 *   Core 1 (loop):      WebServer, MQTT, WiFi, LED, energy tracking, NTP
 *
 * Thread safety
 *   rs485Mutex  - serializes RS485 bus access between polling (Core 0) and
 *                 web service requests (Core 1, /svc/* endpoints)
 *   dataMutex   - protects victronData, bms[], alarm_flags, sys_error_msg
 *                 during writes (Core 0) and reads (Core 1)
 *
 * Web UI
 *   Glassmorphism sidebar layout, 5 pages: Dashboard, Battery, Network,
 *   General, Alerts. Frosted glass cards, light + dark mode.
 *   Served as raw literal from handleRoot(), polls /data every 2.5s.
 *   BMS cards throttled to 15s refresh. Chart selection syncs across devices.
 *
 * Authentication
 *   Cookie-based session auth with SHA-256 hashed password stored in NVS.
 *   Login page at /login, session token regenerated on each boot.
 *   Rate limiting: 5 failed attempts trigger 60s lockout.
 *   Password reset via rapid power-cycle (5x within 15s).
 *
 * MQTT
 *   Publishes to {topic}/data every 5s. LWT for offline detection.
 *   Separate {topic}/alarm topic (retained) fires on state changes.
 *   HA auto-discovery re-sent on every connect (keeps FW version current).
 *
 * History & Energy
 *   4 chart types (power/voltage/SOC/temperature) persisted to NVS.
 *   960 points x 3min = 48h depth. Saved every 15 min + on OTA/reboot.
 *   CSV export via /export endpoint.
 *   Rolling energy counters: today + 7-day + monthly, NVS-persistent.
 *
 * Networking
 *   WiFiManager captive portal on first boot. mDNS (.local hostname).
 *   Smart reboot polling on OTA/save (no fixed timers).
 *
 * Diagnostics
 *   Per-BMS communication stats (polls/ok/timeout/errors/spikes).
 *   Reboot reason logged (esp_reset_reason).
 *   State-transition logging for safety events and temperature throttling.
 *
 * NVS layout
 *   "gateway" namespace - all settings (cnt, cells, pins, auth, mqtt, ...)
 *   "h" namespace       - chart history, energy counters (day/week/month)
 *
 * Protocol work based on https://github.com/linedot/topband-bms
 *
 * License: MIT
 */

// ==========================================
// COMPILE-TIME OPTIONS
// Pin assignments and board type are configured at runtime via the web UI.
// The only compile-time choice is SD card support (LilyGo only) and WiFi mode.
// ==========================================
// #define BOARD_LILYGO        // Uncomment ONLY if using LilyGo T-CAN485 with SD card
#define BOARD_WAVESHARE        // Default: no SD card (works for any ESP32/ESP32-S3)

// ==========================================
// COMMUNICATION MODE (uncomment one)
// ==========================================
#define MODE_SMART_WIFI        // WiFi + WebServer + MQTT + OTA
// #define MODE_SIMPLE_CABLE   // RS485 + CAN only, no WiFi
// ==========================================

#include <Adafruit_NeoPixel.h>
#include <Arduino.h>
#include <driver/twai.h>
#include <esp_task_wdt.h>
#include <Update.h>
#include <mbedtls/sha256.h>    // On-chip SHA-256 for password hashing
#include <math.h>
#include <Preferences.h>
#if defined(BOARD_LILYGO)
  #include <SD.h>              // SD card only available on LilyGo board
  #include <SPI.h>
#endif

// ==========================================
// V2.63: FUNCTION FORWARD DECLARATIONS
// ==========================================
// Struct types are defined later, after the TIMING block. We still need
// forward-declare functions here so Arduino IDE's auto-prototype generator
// has them available. WiFiManager class is forward-declared because its
// full header is included only inside MODE_SMART_WIFI.
// ==========================================
#ifdef MODE_SMART_WIFI
class WiFiManager;
#endif
void setLed(int r, int g, int b);
void addToLog(String msg, bool error);

// WiFi credentials: leave empty to use WiFiManager captive portal on first boot
const char* FIXED_SSID = ""; 
const char* FIXED_PASS = "";
const char *HOSTNAME = "Topband-Gateway"; 
const char *FW_VERSION = "2.63";

// Pre-built RS485 poll commands for BMS address 0-15 (CID2=0x42, analog data request)
const char *POLL_CMDS[16] = {
    "~21004642E00200FD36\r", "~21014642E00201FD34\r", "~21024642E00202FD32\r",
    "~21034642E00203FD30\r", "~21044642E00204FD2E\r", "~21054642E00205FD2C\r",
    "~21064642E00206FD2A\r", "~21074642E00207FD28\r", "~21084642E00208FD26\r",
    "~21094642E00209FD24\r", "~210A4642E0020AFD14\r", "~210B4642E0020BFD12\r",
    "~210C4642E0020CFD10\r", "~210D4642E0020DFD0E\r", "~210E4642E0020EFD0C\r",
    "~210F4642E0020FFD0A\r"};

// ==========================================
// TIMING & LIMITS
// ==========================================
#define NUM_LEDS 1              // Onboard NeoPixel LED count
#define MAX_BMS 16              // Maximum supported BMS packs
#define RS485_BAUD 9600         // TopBand BMS protocol baud rate
#define POLL_INTERVAL 3000      // BMS data poll interval (ms)
#define RS485_TIMEOUT 600       // Response timeout per BMS (ms)
#define BUS_GUARD_TIME 200      // Delay between RS485 transactions (ms)
#define WDT_TIMEOUT 30          // Watchdog timeout (seconds), both cores
#define HISTORY_LEN 960         // 960 x 3min = 48h chart history depth
#define HISTORY_INTERVAL 180000 // Chart data point interval: 3 minutes (ms)
#define SD_LOG_INTERVAL 60000   // SD card log write interval (ms)
#define MQTT_INTERVAL 5000      // MQTT publish interval (ms)
#define ALARM_POLL_INTERVAL 15000   // BMS alarm status poll (round-robin, ms)
#define SYSPARAM_POLL_INTERVAL 30000 // BMS system parameter poll (round-robin, ms)
#define TB_INFO_MAX 256         // Max TopBand protocol INFO field length
#define ALERT_RING_SIZE 25      // V2.61: alert ring buffer depth
#define ALERT_MSG_LEN 96        // V2.61: max chars per alert message

// ==========================================
// V2.63: ALL STRUCT DEFINITIONS CONSOLIDATED HERE
// ==========================================
// Structs are defined here, before any function declarations, so that
// Arduino IDE's auto-prototype generator sees complete types. Previously,
// forward-declarations were used but the IDE still emitted prototypes
// referencing incomplete types in some toolchain versions, causing
// "TBFrame has not been declared" errors. Defining structs upfront fixes
// that class of error entirely.
// ==========================================

// Per-BMS live data (updated by parseTopband on Core 0)
struct BMSData {
  bool valid; float voltage; float current; int soc; int soh; float rem_ah; float full_ah; float cells[32];
  int cell_count; float temps[8]; int temp_count; uint16_t cycles;
  float minCellV; float maxCellV; float avgCellV;
  int minCellIdx; int maxCellIdx; float maxTemp; float avgTemp; unsigned long last_seen;
};

// Aggregated data sent to inverter via CAN (updated by calculateVictronData on Core 0)
struct VictronType {
  float totalCurrent; float avgVoltage; float avgSOC; float avgSOH; float maxChargeCurrent; float maxDischargeCurrent;
  float avgTemp; float totalCapacity; float remainCapacity; float totalPower; int activePacks;
};

// TopBand RS485 protocol frame (SOI ~ VER ADR CID1 CID2 LENGTH INFO CHKSUM EOI)
struct TBFrame {
  bool valid;
  bool info_ascii_raw;
  uint8_t version;
  uint8_t adr;
  uint8_t cid1;
  uint8_t cid2;
  uint16_t length_field;
  uint16_t lenid;
  uint16_t checksum;
  uint8_t info[TB_INFO_MAX];
  size_t info_len;
};

struct TBDate {
  int year;
  int month;
  int day;
  int hour;
  int minute;
  int second;
};

// BMS system parameters (read via CID2=0x47)
struct TBSysParam {
  float cell_high_v;
  float cell_low_v;
  float cell_under_v;
  float charge_high_t;
  float charge_low_t;
  float charge_current_max_a;
  float module_high_v;
  float module_low_v;
  float module_under_v;
  float discharge_high_t;
  float discharge_low_t;
  float discharge_current_max_a;
};

// BMS alarm/status bitmap (read via CID2=0x44)
struct TBAlarmInfoParsed {
  uint8_t cell_count;
  uint8_t temp_count;
  uint8_t cell_voltage_status[16];
  uint8_t cell_temp_status[16];
  uint8_t balancer_temp_status;
  uint8_t environment_temp_status;
  uint8_t mosfet_temp_status;
  uint8_t charge_current_status;
  uint8_t module_voltage_status;
  uint8_t status_count;
  uint64_t status_bits;
};

struct TBAnalogDiag {
  uint8_t id;
  uint8_t cell_count;
  uint8_t temp_count;
  float min_cell_v;
  float max_cell_v;
  float current_a;
  float pack_v;
  float rem_ah;
  float full_ah;
  int soc;
  int soh;
};

// Per-BMS communication statistics (reset on reboot)
struct BMSCommStats {
  uint32_t polls;      // Total poll attempts
  uint32_t ok;         // Successful responses
  uint32_t timeouts;   // No response within timeout
  uint32_t errors;     // Parse errors or bad frames
  uint32_t spikes;     // Spike filter rejections
};

// V2.61: alert ring buffer entry (NVS-persisted)
struct AlertEntry {
  uint32_t ts;                // Unix epoch (0 if NTP unavailable)
  uint32_t uptime_s;          // Fallback: seconds since boot when alert raised
  uint8_t sev;                // 0=info, 1=warn, 2=err
  uint8_t _pad[3];
  char msg[ALERT_MSG_LEN];    // Null-terminated
};

// V2.61: state-transition tracking to avoid alert spam on 10s tick
struct AlertState {
  bool no_packs;              // activePacks==0
  bool bms_offline[MAX_BMS];
  bool cell_max_near[MAX_BMS];
  bool cell_min_crit[MAX_BMS];
  bool drift_near[MAX_BMS];
  bool temp_near[MAX_BMS];
  bool soc_low[MAX_BMS];
  bool sys_soc_low;
  bool can_error_state;
  bool mqtt_fail_state;
};


// ==========================================
// PIN ASSIGNMENTS (runtime-configurable, persisted in NVS)
// Board type: 0=Waveshare, 1=LilyGo, 2=Custom
// Presets loaded at boot, overridable via web UI.
// ==========================================

// SD card support remains compile-time (controls #include)
#if defined(BOARD_LILYGO)
  #define HAS_SD_CARD true
  #define SD_MISO 2
  #define SD_MOSI 15
  #define SD_SCLK 14
  #define SD_CS 13
#else
  #define HAS_SD_CARD false
  #define SD_MISO -1
  #define SD_MOSI -1
  #define SD_SCLK -1
  #define SD_CS -1
#endif

// Pin variables (set from NVS or preset at boot)
int g_pin_rs485_tx = 17;
int g_pin_rs485_rx = 18;
int g_pin_rs485_dir = 21;     // Single direction pin (Waveshare/Custom mode)
int g_pin_can_tx = 15;
int g_pin_can_rx = 16;
int g_pin_led = 38;
// LilyGo-specific pins (active when board_type=1)
int g_pin_lilygo_pwr = 16;
int g_pin_lilygo_en = 19;
int g_pin_lilygo_cb = 17;
// Board type: 0=Waveshare, 1=LilyGo, 2=Custom
int g_board_type = 0;

// Presets: apply default pin values for a given board type
void applyBoardPreset(int type) {
  if (type == 1) {
    // LilyGo T-CAN485
    g_pin_rs485_tx = 22; g_pin_rs485_rx = 21; g_pin_rs485_dir = -1;
    g_pin_can_tx = 27; g_pin_can_rx = 26; g_pin_led = 4;
    g_pin_lilygo_pwr = 16; g_pin_lilygo_en = 19; g_pin_lilygo_cb = 17;
  } else {
    // Waveshare ESP32-S3 (also default for Custom)
    g_pin_rs485_tx = 17; g_pin_rs485_rx = 18; g_pin_rs485_dir = 21;
    g_pin_can_tx = 15; g_pin_can_rx = 16; g_pin_led = 38;
    g_pin_lilygo_pwr = -1; g_pin_lilygo_en = -1; g_pin_lilygo_cb = -1;
  }
}

#ifdef MODE_SMART_WIFI
  #include <WiFi.h>
  #include <WebServer.h>
  #include <ESPmDNS.h>
  #include <WiFiManager.h>
  #include <PubSubClient.h>
#endif

// ==========================================
// HARDWARE OBJECTS
// ==========================================
Adafruit_NeoPixel pixels(NUM_LEDS, 0, NEO_GRB + NEO_KHZ800); // Pin set in setup()
HardwareSerial RS485(1);       // UART1 for RS485 communication
Preferences preferences;       // NVS storage for settings

// ==========================================
// WIFI / WEB / MQTT GLOBALS
// ==========================================
#ifdef MODE_SMART_WIFI
  WebServer server(80);
  WiFiClient espClient;
  PubSubClient mqtt(espClient);
  Preferences histStore;       // Separate NVS namespace "h" for history data
  
  // MQTT configuration (persisted in NVS)
  String g_mqtt_server = ""; int g_mqtt_port = 1883; String g_mqtt_user = ""; String g_mqtt_pass = "";
  String g_mqtt_topic = "Topband/BMS";
  String g_dev_uid = "tb_000000"; // Generated from MAC address at boot
  bool g_mqtt_enable = false; bool g_mqtt_full = false; bool g_ha_enable = false;
  bool g_sd_enable = false; bool g_sd_spy = false; bool g_serial_debug = false;
  String debug_log = ""; String debug_can_status = "Init";
  bool sd_ok = false; bool sd_error_flag = false;
  int g_theme_id = 0; String g_ntp_server = "pool.ntp.org"; int g_timezone_offset = 1;

  // Chart history: 4 data types x 960 points = 48 hours at 3-min intervals
  // Avg arrays are NVS-persisted; min/max rebuild from avg after reboot
  int16_t powerHistory[HISTORY_LEN]; int historyIdx = 0;
  int16_t voltHistory[HISTORY_LEN];  // Stored as V * 100 (int16)
  int16_t socHistory[HISTORY_LEN];   // Stored as % * 10 (int16)
  int16_t tempHistory[HISTORY_LEN];  // Stored as C * 10 (int16)
  int16_t powerMin[HISTORY_LEN]; int16_t powerMax[HISTORY_LEN];
  int16_t voltMin[HISTORY_LEN];  int16_t voltMax[HISTORY_LEN];
  int16_t socMin[HISTORY_LEN];   int16_t socMax[HISTORY_LEN];
  int16_t tempMin[HISTORY_LEN];  int16_t tempMax[HISTORY_LEN];
  int historyFilled = 0;             // V2.61: slots written since first boot, capped at HISTORY_LEN. Distinguishes real 0 from "no data".

  // Accumulators: collect min/max/avg between 3-min history intervals
  float acc_sum[4] = {0};  int acc_count = 0;
  float acc_min[4], acc_max[4];
  bool acc_reset = true;
  int g_chart1 = 0; int g_chart2 = 1;  // Chart type selection: 0=power, 1=voltage, 2=soc, 3=temp

  // Energy tracking: calculated by ESP (power * time), reset at midnight
  float energy_in_today = 0; float energy_out_today = 0;
  unsigned long last_energy_calc = 0;
  // Rolling energy history: 7 days + 12 months, persisted in NVS "h" namespace
  float energy_days_in[7] = {0};   // [0]=yesterday, [1]=2 days ago, ... [6]=7 days ago
  float energy_days_out[7] = {0};
  float energy_months_in[12] = {0}; // [0]=this month, [1]=last month, ... [11]=12 months ago
  float energy_months_out[12] = {0};
  int current_month = -1;           // Tracks month for rollover
  unsigned long last_hist_time = 0; unsigned long last_sd_time = 0; unsigned long last_mqtt_time = 0;
  unsigned long last_mqtt_reconnect_attempt = 0; unsigned long mqtt_reconnect_interval = 5000;
  uint16_t mqtt_fail_count = 0; uint32_t heap_min = UINT32_MAX;
  int current_day = -1;        // Tracks day-of-month for midnight energy reset
#endif

// ==========================================
// BMS & SAFETY CONFIGURATION (persisted in NVS "gateway" namespace)
// ==========================================
int g_bms_count = 2; int g_force_cell_count = 0; int g_detected_cells = 0; 
bool g_victron_enable = true;
float g_charge_amps = 30.0; float g_discharge_amps = 30.0;
float g_cvl_voltage = 52.5;       // Charge Voltage Limit sent to inverter
float g_safe_volt = 53.25;        // Safety cutoff: max pack voltage
float g_safe_cell = 3.55;         // Safety cutoff: max cell voltage
float g_safe_drift = 0.20;        // Safety warning: max cell drift (V)
// Temperature cutoffs (C): charge/discharge are throttled in soft zones
float g_tc_min = 5.0; float g_tc_max = 50.0;   // Charge temperature range
float g_td_min = -20.0; float g_td_max = 60.0;  // Discharge temperature range
int g_temp_mode = 0;               // 0=use hottest sensor (safe), 1=use average
int g_can_protocol = 0;            // 0=Victron, 1=Pylontech, 2=SMA
bool g_expert_mode = true; bool g_easy_mode = false;          // Deprecated: kept for NVS compat
bool g_setup_done = true; bool g_easy_auto_applied = false;   // Deprecated: kept for NVS compat
int g_soc_source_mode = 2;        // SOC source: 0=calculated, 1=raw BMS, 2=hybrid
bool g_maint_charge_mode = false;
float g_maint_target_v = 51.00;   // Maintenance charge target voltage
bool g_auto_balance_enable = true; // Auto-balance watchdog (30 days)
uint32_t g_auto_balance_last_ts = 0;
bool g_runtime_auto_balance_due = false;
bool g_runtime_balance_reached_prev = false;
float factor_charge = 1.0; float factor_discharge = 1.0; // Temperature throttle factors (0.0-1.0)

// Web authentication (cookie-based session, SHA-256 hashed password in NVS)
bool g_auth_enable = false;
String g_auth_user = "admin";
String g_auth_hash = "";           // SHA-256 hex digest, never stored in plaintext

// ==========================================
// RUNTIME STATE (not persisted)
// ==========================================
unsigned long last_poll_time = 0; unsigned long last_can_time = 0;
unsigned long stat_rx_count = 0; unsigned long stat_tx_count = 0;
bool can_error_flag = false;
uint8_t alarm_flags = 0;          // Bitmask: 0x02=overvolt, 0x08=temp, 0x10=undervolt, 0x20=drift, 0x40=BMS alarm, 0x80=no packs
String sys_error_msg = "OK";
bool simulation_active = false;
String g_boot_reason = "UNKNOWN";   // Set once in setup() from esp_reset_reason()
SemaphoreHandle_t rs485Mutex = NULL;  // Protects RS485 bus access between cores
SemaphoreHandle_t dataMutex = NULL;   // Protects victronData/bms[]/alarm_flags between cores
TaskHandle_t rs485TaskHandle = NULL;
unsigned long last_alarm_poll_time = 0;
int next_alarm_poll_bms = 0;
unsigned long last_sysparam_poll_time = 0;
int next_sysparam_poll_bms = 0;

// ==========================================
// GLOBAL STORAGE (structs defined earlier, after TIMING block)
// ==========================================

BMSData bms[MAX_BMS];
VictronType victronData;

// Per-BMS alarm and system parameter cache (updated by round-robin polling on Core 0)
uint64_t g_bms_alarm_bits[MAX_BMS];
bool g_bms_was_online[MAX_BMS];       // Tracks online->offline transitions for logging
unsigned long g_bms_alarm_last_seen[MAX_BMS];
uint8_t g_bms_alarm_status_count[MAX_BMS];
float g_bms_sys_module_high_v[MAX_BMS];
float g_bms_sys_module_under_v[MAX_BMS];
float g_bms_sys_charge_max_a[MAX_BMS];
float g_bms_sys_discharge_max_a[MAX_BMS];
float g_bms_sys_cell_high_v[MAX_BMS];
float g_bms_sys_charge_low_t[MAX_BMS];
float g_bms_sys_charge_high_t[MAX_BMS];
float g_bms_sys_discharge_low_t[MAX_BMS];
float g_bms_sys_discharge_high_t[MAX_BMS];
unsigned long g_bms_sys_last_seen[MAX_BMS];

BMSCommStats g_bms_stats[MAX_BMS];

// V2.61: alert ring buffer storage. Server-side detection runs every 10s on Core 1.
// Dedup: same message within 30s is suppressed. Clear: POST /alerts/clear wipes RAM + NVS.
AlertEntry g_alerts[ALERT_RING_SIZE];
int g_alert_count = 0;        // Total entries stored (capped at ALERT_RING_SIZE)
int g_alert_head = 0;         // Next write index (ring position)
bool g_alerts_dirty = false;
unsigned long g_alerts_last_save = 0;
unsigned long g_alerts_last_detect = 0;
AlertState g_alert_state = {};


// Runtime protocol-derived limits (capped from BMS 0x47 system params)
float g_runtime_cvl = 0.0f;          // Effective CVL (may be lower than config)
float g_runtime_proto_ccl_cap = 0.0f; // Protocol charge current cap (sum of all BMS)
float g_runtime_proto_dcl_cap = 0.0f; // Protocol discharge current cap
bool g_runtime_proto_uv_hit = false;   // True if any BMS near undervolt threshold

// ==========================================
// TOPBAND RS485 PROTOCOL HELPERS
// ==========================================
// Frame format: ~VER ADR CID1 CID2 LENGTH INFO CHKSUM CR
// All values are ASCII hex encoded. Checksum is modular complement.
// Protocol details: https://github.com/linedot/topband-bms
// ==========================================
void setLed(int r, int g, int b) { pixels.setPixelColor(0, pixels.Color(r, g, b)); pixels.show(); }
uint16_t get_u16(const uint8_t *b, int o) { return (b[o] << 8) | b[o + 1]; }
int16_t get_s16(const uint8_t *b, int o) { uint16_t v = get_u16(b, o); return (v > 32767) ? (v - 65536) : v; }
uint8_t parse_hex(char c) { if (c >= '0' && c <= '9') return c - '0'; if (c >= 'A' && c <= 'F') return c - 'A' + 10; if (c >= 'a' && c <= 'f') return c - 'a' + 10; return 0xFF; }

uint16_t tbBuildLengthField(uint16_t lenid) {
  lenid &= 0x0FFF;
  uint16_t sum = ((lenid >> 8) & 0xF) + ((lenid >> 4) & 0xF) + (lenid & 0xF);
  uint16_t lchk = ((~(sum & 0xF)) + 1) & 0xF;
  return (lchk << 12) | lenid;
}

bool tbCheckLengthField(uint16_t length_field) {
  uint16_t lenid = length_field & 0x0FFF;
  uint16_t lchk = (length_field >> 12) & 0xF;
  uint16_t sum = ((lenid >> 8) & 0xF) + ((lenid >> 4) & 0xF) + (lenid & 0xF);
  uint16_t expected = ((~(sum & 0xF)) + 1) & 0xF;
  return lchk == expected;
}

String toHex2(uint8_t v) {
  char b[3];
  snprintf(b, sizeof(b), "%02X", v);
  return String(b);
}

String toHex4(uint16_t v) {
  char b[5];
  snprintf(b, sizeof(b), "%04X", v);
  return String(b);
}

bool parseHexByteAt(const String &s, int pos, uint8_t &out) {
  if (pos < 0 || pos + 1 >= s.length()) return false;
  uint8_t hi = parse_hex(s[pos]);
  uint8_t lo = parse_hex(s[pos + 1]);
  if (hi == 0xFF || lo == 0xFF) return false;
  out = (hi << 4) | lo;
  return true;
}

bool parseHexU16At(const String &s, int pos, uint16_t &out) {
  if (pos < 0 || pos + 3 >= s.length()) return false;
  uint8_t b0, b1;
  if (!parseHexByteAt(s, pos, b0)) return false;
  if (!parseHexByteAt(s, pos + 2, b1)) return false;
  out = ((uint16_t)b0 << 8) | b1;
  return true;
}

String tbBuildCommand(uint8_t addr, uint8_t cid2, const uint8_t *info, size_t info_len_bytes) {
  uint16_t lenid = (uint16_t)(info_len_bytes * 2);
  uint16_t length_field = tbBuildLengthField(lenid);

  String body = "";
  body.reserve(32 + lenid);
  body += toHex2(0x21);  // protocol version
  body += toHex2(addr);
  body += toHex2(0x46);  // CID1 battery
  body += toHex2(cid2);
  body += toHex4(length_field);
  for (size_t i = 0; i < info_len_bytes; i++) body += toHex2(info[i]);

  uint16_t sum = 0;
  for (int i = 0; i < body.length(); i++) sum += (uint8_t)body[i];
  uint16_t chk = ((~(sum & 0xFFFF)) + 1) & 0xFFFF;

  String cmd = "~";
  cmd += body;
  cmd += toHex4(chk);
  cmd += "\r";
  return cmd;
}

// RS485 direction helpers (runtime pin config)
void rs485SetTX() {
  if (g_pin_rs485_dir >= 0) digitalWrite(g_pin_rs485_dir, HIGH);
}
void rs485SetRX() {
  if (g_pin_rs485_dir >= 0) digitalWrite(g_pin_rs485_dir, LOW);
}

// Send command and receive response over RS485 bus (mutex-protected)
bool rs485ExchangeFrame(const String &cmd, String &resp, uint32_t timeout_ms) {
  if (!rs485Mutex || !xSemaphoreTake(rs485Mutex, pdMS_TO_TICKS(3000))) return false;

  bool ok = false;
  resp = "";
  resp.reserve(512);

  rs485SetTX();
  RS485.flush();
  while (RS485.available()) RS485.read();
  RS485.print(cmd);
  RS485.flush();
  rs485SetRX();

  unsigned long start = millis();
  while (millis() - start < timeout_ms) {
    if (RS485.available()) {
      char c = RS485.read();
      if (resp.length() < 700) resp += c;
      if (c == '\r') { ok = true; break; }
    } else {
      vTaskDelay(1);
    }
  }

  xSemaphoreGive(rs485Mutex);
  return ok && resp.length() > 0;
}

bool tbDecodeFrame(const String &raw_resp, TBFrame &frame, String &err) {
  frame = {};
  err = "";
  int soi = raw_resp.indexOf('~');
  if (soi < 0) { err = "No SOI"; return false; }

  String s = raw_resp.substring(soi);
  int eoi = s.indexOf('\r');
  if (eoi < 0) eoi = s.indexOf('\n');
  if (eoi >= 0) s = s.substring(0, eoi + 1);

  if (s.length() < 18) { err = "Frame too short"; return false; }
  char eoi_ch = s[s.length() - 1];
  if (eoi_ch != '\r' && eoi_ch != '\n') { err = "No EOI"; return false; }

  uint8_t version, adr, cid1, cid2;
  uint16_t length_field, rx_chk;
  if (!parseHexByteAt(s, 1, version) || !parseHexByteAt(s, 3, adr) || !parseHexByteAt(s, 5, cid1) || !parseHexByteAt(s, 7, cid2)) {
    err = "Header hex parse failed"; return false;
  }
  if (!parseHexU16At(s, 9, length_field)) { err = "Length parse failed"; return false; }
  if (!tbCheckLengthField(length_field)) { err = "LCHKSUM invalid"; return false; }

  uint16_t lenid = length_field & 0x0FFF;
  int expected_len = 18 + lenid;
  if (s.length() < expected_len) { err = "Truncated frame"; return false; }
  if (!parseHexU16At(s, 13 + lenid, rx_chk)) { err = "CHKSUM parse failed"; return false; }

  uint16_t sum = 0;
  for (int i = 1; i < 13 + lenid; i++) sum += (uint8_t)s[i];
  uint16_t expected_chk = ((~(sum & 0xFFFF)) + 1) & 0xFFFF;
  if (rx_chk != expected_chk) { err = "CHKSUM invalid"; return false; }

  frame.valid = true;
  frame.version = version;
  frame.adr = adr;
  frame.cid1 = cid1;
  frame.cid2 = cid2;
  frame.length_field = length_field;
  frame.lenid = lenid;
  frame.checksum = rx_chk;
  frame.info_ascii_raw = false;

  if (lenid == 0) {
    frame.info_len = 0;
    return true;
  }
  if (lenid > TB_INFO_MAX) { err = "Info too long"; return false; }

  bool hex_ok = (lenid % 2) == 0;
  if (hex_ok) {
    size_t info_bytes = lenid / 2;
    for (size_t i = 0; i < info_bytes; i++) {
      uint8_t b;
      if (!parseHexByteAt(s, 13 + (int)(i * 2), b)) { hex_ok = false; break; }
      frame.info[i] = b;
    }
    if (hex_ok) {
      frame.info_len = info_bytes;
      return true;
    }
  }

  // Manufacturer info can be raw ASCII in protocol field.
  for (uint16_t i = 0; i < lenid; i++) frame.info[i] = (uint8_t)s[13 + i];
  frame.info_ascii_raw = true;
  frame.info_len = lenid;
  return true;
}

bool tbRequestFrame(uint8_t bms_id, uint8_t cid2, const uint8_t *info, size_t info_len_bytes, TBFrame &reply, String &err, bool allow_nonzero_cid2 = false) {
  String cmd = tbBuildCommand(bms_id, cid2, info, info_len_bytes);
  String raw;
  if (!rs485ExchangeFrame(cmd, raw, RS485_TIMEOUT + 200)) {
    err = "No response";
    return false;
  }
  if (!tbDecodeFrame(raw, reply, err)) return false;
  if (!allow_nonzero_cid2 && reply.cid2 != 0x00) {
    err = "BMS reply code 0x" + String(reply.cid2, HEX);
    return false;
  }
  return true;
}

float tbKelvin10ToC(uint16_t raw) {
  return ((float)raw - 2731.0f) / 10.0f;
}

bool tbParseDateFromFrame(const TBFrame &f, TBDate &d) {
  if (!f.valid || f.info_len < 7) return false;
  d.year = ((int)f.info[0] << 8) | (int)f.info[1];
  d.month = f.info[2];
  d.day = f.info[3];
  d.hour = f.info[4];
  d.minute = f.info[5];
  d.second = f.info[6];
  return true;
}

bool tbParseSystemParameterFromFrame(const TBFrame &f, TBSysParam &sp) {
  if (!f.valid || f.info_len < 24) return false;
  sp.cell_high_v = ((float)get_u16(f.info, 0)) / 1000.0f;
  sp.cell_low_v = ((float)get_u16(f.info, 2)) / 1000.0f;
  sp.cell_under_v = ((float)get_u16(f.info, 4)) / 1000.0f;
  sp.charge_high_t = tbKelvin10ToC(get_u16(f.info, 6));
  sp.charge_low_t = tbKelvin10ToC(get_u16(f.info, 8));
  sp.charge_current_max_a = ((float)((int16_t)get_u16(f.info, 10))) * 0.01f;
  sp.module_high_v = ((float)get_u16(f.info, 12)) / 100.0f;
  sp.module_low_v = ((float)get_u16(f.info, 14)) / 100.0f;
  sp.module_under_v = ((float)get_u16(f.info, 16)) / 100.0f;
  sp.discharge_high_t = tbKelvin10ToC(get_u16(f.info, 18));
  sp.discharge_low_t = tbKelvin10ToC(get_u16(f.info, 20));
  sp.discharge_current_max_a = ((float)((int16_t)get_u16(f.info, 22))) * 0.01f;
  return true;
}

bool tbParseAlarmInfoFromFrame(const TBFrame &f, TBAlarmInfoParsed &a) {
  if (!f.valid || f.info_len < 8) return false;
  memset(&a, 0, sizeof(a));

  size_t off = 1;
  if (off >= f.info_len) return false;
  a.cell_count = f.info[off];
  if (a.cell_count > 16) a.cell_count = 16;
  off += 1;
  if (off + a.cell_count > f.info_len) return false;
  for (uint8_t i = 0; i < a.cell_count; i++) a.cell_voltage_status[i] = f.info[off + i];
  off += a.cell_count;

  if (off >= f.info_len) return false;
  a.temp_count = f.info[off];
  off += 1;
  uint8_t cell_temp_bytes = (a.temp_count >= 3) ? (a.temp_count - 3) : 0;
  if (cell_temp_bytes > 16) cell_temp_bytes = 16;
  if (off + cell_temp_bytes > f.info_len) return false;
  for (uint8_t i = 0; i < cell_temp_bytes; i++) a.cell_temp_status[i] = f.info[off + i];
  off += cell_temp_bytes;

  if (off + 3 > f.info_len) return false;
  a.balancer_temp_status = f.info[off];
  a.environment_temp_status = f.info[off + 1];
  a.mosfet_temp_status = f.info[off + 2];
  off += 3;

  if (off + 3 > f.info_len) return false;
  a.charge_current_status = f.info[off];
  a.module_voltage_status = f.info[off + 1];
  a.status_count = f.info[off + 2];
  off += 3;

  if (off + a.status_count > f.info_len) return false;
  uint8_t status_bytes = a.status_count > 8 ? 8 : a.status_count;
  for (uint8_t i = 0; i < status_bytes; i++) a.status_bits |= ((uint64_t)f.info[off + i]) << (8 * i);
  return true;
}

bool tbParseAnalogDiagFromFrame(const TBFrame &f, TBAnalogDiag &a) {
  if (!f.valid || f.info_len < 16) return false;
  memset(&a, 0, sizeof(a));
  size_t p = 0;
  a.id = f.info[p++];
  a.cell_count = f.info[p++];
  if (a.cell_count > 32) a.cell_count = 32;

  float min_v = 99.0f, max_v = 0.0f;
  for (uint8_t i = 0; i < a.cell_count; i++) {
    if (p + 1 >= f.info_len) return false;
    float v = ((float)get_u16(f.info, (int)p)) / 1000.0f;
    if (v > 0.1f) {
      if (v < min_v) min_v = v;
      if (v > max_v) max_v = v;
    }
    p += 2;
  }
  a.min_cell_v = min_v < 90.0f ? min_v : 0.0f;
  a.max_cell_v = max_v;

  if (p >= f.info_len) return false;
  a.temp_count = f.info[p++];
  if (a.temp_count > 8) a.temp_count = 8;
  size_t temp_bytes = (size_t)a.temp_count * 2;
  if (p + temp_bytes > f.info_len) return false;
  p += temp_bytes;

  if (p + 10 >= f.info_len) return false;
  a.current_a = ((float)((int16_t)get_u16(f.info, (int)p))) * 0.01f; p += 2;
  a.pack_v = ((float)get_u16(f.info, (int)p)) * 0.01f; p += 2;
  a.rem_ah = ((float)get_u16(f.info, (int)p)) * 0.01f; p += 3;
  a.full_ah = ((float)get_u16(f.info, (int)p)) * 0.01f; p += 2;

  int soc_calc = 0;
  if (a.full_ah > 0.01f) soc_calc = (int)((a.rem_ah / a.full_ah) * 100.0f);
  if (soc_calc > 100) soc_calc = 100;
  if (soc_calc < 0) soc_calc = 0;
  a.soc = soc_calc;
  a.soh = 100;

  // Newer Topband payloads carry cycle(2B), SOC(1B), SOH(1B).
  // Keep legacy fallback to avoid regressions with older frames.
  size_t meta = p;
  if (meta + 3 < f.info_len) {
    uint8_t raw_soc = f.info[meta + 2];
    uint8_t raw_soh = f.info[meta + 3];
    bool extended_layout = (meta + 7 < f.info_len);
    bool raw_valid = (raw_soc <= 100) && (raw_soh <= 100);
    a.soc = tbSelectSocValue(soc_calc, (int)raw_soc, raw_valid, extended_layout);
    if (raw_valid && (extended_layout || g_soc_source_mode == 1 || abs((int)raw_soc - soc_calc) <= 15)) {
      a.soh = (int)raw_soh;
    } else {
      uint16_t s16 = get_u16(f.info, (int)(meta + 2));
      if (s16 <= 100) a.soh = (int)s16;
    }
  }
  return true;
}

const char* tbAlarmBitName(uint8_t bit) {
  switch (bit) {
    case 0: return "Cell over voltage protect";
    case 1: return "Cell under voltage";
    case 2: return "Charge over current protect";
    case 3: return "Cell over voltage alarm";
    case 4: return "Discharge over current 1 protect";
    case 5: return "Cell temperature discharge protect";
    case 6: return "Cell temperature charge protect";
    case 7: return "Pack under voltage alarm";
    case 8: return "Open current limit";
    case 9: return "Charge MOSFET on";
    case 10: return "Discharge MOSFET on";
    case 11: return "Short circuit protect";
    case 12: return "Cell under voltage protect";
    case 13: return "Pack under voltage protect";
    case 14: return "Reverse protect";
    case 15: return "SOC low alarm";
    case 20: return "Charger connected";
    case 22: return "Discharging";
    case 23: return "Charging";
    case 28: return "Cell low force protect";
    case 40: return "Pack over voltage alarm";
    case 41: return "MOS NTC temperature alarm";
    case 42: return "Environment temperature low alarm";
    case 43: return "Environment temperature high alarm";
    case 44: return "Cell temperature low alarm";
    case 45: return "Cell temperature high alarm";
    case 46: return "Discharge current alarm";
    case 47: return "Charge current alarm";
    case 48: return "Balance temperature alarm";
    case 49: return "Balance temperature protect";
    case 50: return "Discharge MOSFET fault";
    case 51: return "Charge MOSFET fault";
    case 52: return "Current sensor fault";
    case 53: return "AFE fault";
    case 54: return "NTC fault";
    case 55: return "Cell fault";
    case 56: return "Discharge over current 2 protect";
    case 58: return "Pack over voltage protect";
    case 59: return "MOS temperature protect";
    case 60: return "Discharge MOS forced close";
    case 61: return "Charge MOS forced close";
    case 62: return "Environment protect discharging";
    case 63: return "Environment protect charging";
    default: return "Alarm/Status";
  }
}

const uint64_t TB_CRITICAL_ALARM_MASK =
    (1ULL << 0)  | (1ULL << 2)  | (1ULL << 4)  | (1ULL << 5)  |
    (1ULL << 6)  | (1ULL << 11) | (1ULL << 12) | (1ULL << 13) |
    (1ULL << 50) | (1ULL << 51) | (1ULL << 52) | (1ULL << 53) |
    (1ULL << 54) | (1ULL << 55) | (1ULL << 58) | (1ULL << 59);

float tbUnderVoltSanityCap(int cells) {
  if (cells <= 0 || cells > 32) return 57.0f;
  float cap = ((float)cells) * 3.20f;
  if (cap < 40.0f) cap = 40.0f;
  if (cap > 57.0f) cap = 57.0f;
  return cap;
}

bool tbShouldFlagProtoUnderVolt(float pack_v, float module_under_v, int cells) {
  if (module_under_v <= 1.0f) return false;
  if (cells <= 0 || cells > 32) return false;
  float trigger_v = module_under_v + 0.05f;
  float sanity_cap = tbUnderVoltSanityCap(cells);
  if (trigger_v > sanity_cap) return false;
  return pack_v <= trigger_v;
}

float tbOverVoltCellSanityFloor(float cell_high_v) {
  float floor = cell_high_v - 0.12f;
  if (cell_high_v < 3.2f || cell_high_v > 4.5f) floor = 3.58f;
  if (floor < 3.50f) floor = 3.50f;
  if (floor > 4.05f) floor = 4.05f;
  return floor;
}

float tbOverVoltPackSanityFloor(float module_high_v, int cells) {
  float floor = module_high_v - 0.80f;
  if (module_high_v < 30.0f || module_high_v > 70.0f) {
    if (cells > 0 && cells <= 32) floor = ((float)cells) * 3.55f;
    else floor = 50.0f;
  }
  if (floor < 40.0f) floor = 40.0f;
  if (floor > 61.0f) floor = 61.0f;
  return floor;
}

uint64_t tbFilterCriticalAlarmBits(uint64_t status_bits, float pack_v, float max_temp_c, int cells, float temp_limit_c, float max_cell_v, float cell_high_v, float module_high_v) {
  uint64_t bits = status_bits & TB_CRITICAL_ALARM_MASK;
  const uint64_t uv_bits = (1ULL << 12) | (1ULL << 13);
  if (cells <= 0 || cells > 32) bits &= ~uv_bits;
  else if (pack_v > tbUnderVoltSanityCap(cells)) bits &= ~uv_bits;

  const uint64_t ov_bits = (1ULL << 0) | (1ULL << 58);
  float ov_cell_floor = tbOverVoltCellSanityFloor(cell_high_v);
  float ov_pack_floor = tbOverVoltPackSanityFloor(module_high_v, cells);
  if (max_cell_v < ov_cell_floor && pack_v < ov_pack_floor) bits &= ~ov_bits;

  const uint64_t temp_bits = (1ULL << 5) | (1ULL << 6) | (1ULL << 59);
  float temp_guard = temp_limit_c - 3.0f;
  if (temp_guard < 15.0f) temp_guard = 15.0f;
  if (max_temp_c < temp_guard) bits &= ~temp_bits;

  return bits;
}

bool tbHasCriticalAlarm(uint64_t status_bits) {
  return (status_bits & TB_CRITICAL_ALARM_MASK) != 0;
}

String jsonEscape(const String &in) {
  String out;
  out.reserve(in.length() + 16);
  const char *hexDigits = "0123456789ABCDEF";
  for (int i = 0; i < in.length(); i++) {
    uint8_t c = (uint8_t)in[i];
    switch (c) {
      case '\\': out += "\\\\"; break;
      case '\"': out += "\\\""; break;
      case '\b': out += "\\b"; break;
      case '\f': out += "\\f"; break;
      case '\n': out += "\\n"; break;
      case '\r': out += "\\r"; break;
      case '\t': out += "\\t"; break;
      default:
        if (c < 0x20) {
          out += "\\u00";
          out += hexDigits[(c >> 4) & 0x0F];
          out += hexDigits[c & 0x0F];
        } else {
          out += (char)c;
        }
        break;
    }
  }
  return out;
}

String u64Hex(uint64_t v) {
  char b[19];
  snprintf(b, sizeof(b), "%016llX", (unsigned long long)v);
  return String(b);
}

int tbSelectSocValue(int soc_calc, int soc_raw, bool raw_valid, bool extended_layout) {
  if (soc_calc < 0) soc_calc = 0;
  if (soc_calc > 100) soc_calc = 100;
  if (soc_raw < 0) soc_raw = 0;
  if (soc_raw > 100) soc_raw = 100;

  if (g_soc_source_mode == 0) return soc_calc;
  if (g_soc_source_mode == 1) return raw_valid ? soc_raw : soc_calc;

  if (!raw_valid) return soc_calc;
  if (extended_layout || abs(soc_raw - soc_calc) <= 15) return soc_raw;
  return soc_calc;
}

bool tbParseManufacturerFromFrame(const TBFrame &f, String &hw, String &sw, String &id) {
  if (!f.valid || f.info_len < 24) return false;
  String raw = "";
  raw.reserve(f.info_len + 1);
  for (size_t i = 0; i < f.info_len; i++) raw += (char)f.info[i];
  if (raw.length() < 24) return false;
  hw = raw.substring(0, 20); hw.trim();
  sw = raw.substring(20, 24); sw.trim();
  id = raw.substring(24); id.trim();
  return true;
}

bool tbParseHistoricalFirstFromFrame(const TBFrame &f, TBDate &d, int &event_type, uint64_t &status_bits, float &current_a, float &pack_v, float &remain_ah) {
  if (!f.valid || f.info_len < 24) return false;
  if (f.info_len < 11) return false;

  d.year = ((int)f.info[2] << 8) | (int)f.info[3];
  d.month = f.info[4];
  d.day = f.info[5];
  d.hour = f.info[6];
  d.minute = f.info[7];
  d.second = f.info[8];
  event_type = f.info[9];
  uint8_t status_count = f.info[10];
  status_bits = 0;
  size_t off = 11;
  uint8_t copy_n = status_count > 8 ? 8 : status_count;
  if (off + copy_n <= f.info_len) {
    for (uint8_t i = 0; i < copy_n; i++) status_bits |= ((uint64_t)f.info[off + i]) << (8 * i);
  }
  if (f.info_len >= 24) {
    current_a = ((float)((int16_t)get_u16(f.info, 18))) / 100.0f;
    pack_v = ((float)get_u16(f.info, 20)) / 100.0f;
    remain_ah = ((float)get_u16(f.info, 22)) / 100.0f;
  } else {
    current_a = 0; pack_v = 0; remain_ah = 0;
  }
  return true;
}

void tbBuildAutoCfgSuggestion(const TBSysParam &sp, float buf_cell_mv, float buf_pack_v, float buf_temp_c, float buf_curr_pct,
                              float &safe_cell, float &safe_volt, float &tc_min, float &tc_max, float &td_min, float &td_max,
                              float &chg_a, float &dis_a, float &cvl_v) {
  float buf_cell_v = buf_cell_mv / 1000.0f;
  safe_cell = sp.cell_high_v - buf_cell_v;
  safe_volt = sp.module_high_v - buf_pack_v;

  tc_min = sp.charge_low_t + buf_temp_c;
  tc_max = sp.charge_high_t - buf_temp_c;
  td_min = sp.discharge_low_t + buf_temp_c;
  td_max = sp.discharge_high_t - buf_temp_c;

  // Safety-first: auto profiles never allow charging below +5 C.
  if (tc_min < 5.0f) tc_min = 5.0f;
  if (tc_max > 50.0f) tc_max = 50.0f;
  if (td_min < -20.0f) td_min = -20.0f;
  if (td_max > 60.0f) td_max = 60.0f;

  float scale = 1.0f - (buf_curr_pct / 100.0f);
  if (scale < 0.1f) scale = 0.1f;
  chg_a = sp.charge_current_max_a * scale;
  dis_a = sp.discharge_current_max_a * scale;

  if (safe_cell < 3.2f) safe_cell = 3.2f;
  if (safe_cell > 3.6f) safe_cell = 3.6f;
  if (safe_volt < 40.0f) safe_volt = 40.0f;
  if (safe_volt > 60.0f) safe_volt = 60.0f;

  int est_cells = 0;
  if (sp.cell_high_v > 2.5f) {
    float ratio = sp.module_high_v / sp.cell_high_v;
    if (ratio >= 8.0f && ratio <= 24.0f) est_cells = (int)(ratio + 0.5f);
  }
  if (est_cells > 0) {
    float pack_cap = safe_cell * (float)est_cells;
    if (safe_volt > pack_cap) safe_volt = pack_cap;
  }

  if (tc_min > tc_max - 1.0f) tc_min = tc_max - 1.0f;
  if (td_min > td_max - 1.0f) td_min = td_max - 1.0f;
  if (chg_a < 1.0f) chg_a = 1.0f;
  if (dis_a < 1.0f) dis_a = 1.0f;

  cvl_v = safe_volt - 0.20f;
  if (cvl_v < 48.0f) cvl_v = 48.0f;
  if (cvl_v > safe_volt) cvl_v = safe_volt;
}
bool tbSystemParamPlausible(const TBSysParam &sp) {
  if (sp.cell_high_v < 3.0f || sp.cell_high_v > 4.5f) return false;
  if (sp.cell_low_v < 2.0f || sp.cell_low_v > 3.6f) return false;
  if (sp.cell_under_v < 1.8f || sp.cell_under_v > 3.5f) return false;
  if (sp.module_high_v < 40.0f || sp.module_high_v > 65.0f) return false;
  if (sp.module_under_v < 20.0f || sp.module_under_v > 60.0f) return false;
  if (sp.module_under_v >= sp.module_high_v) return false;
  if (sp.charge_current_max_a < 1.0f || sp.charge_current_max_a > 500.0f) return false;
  if (sp.discharge_current_max_a < 1.0f || sp.discharge_current_max_a > 500.0f) return false;
  if (sp.charge_low_t < -40.0f || sp.charge_low_t > 40.0f) return false;
  if (sp.charge_high_t < 20.0f || sp.charge_high_t > 100.0f) return false;
  if (sp.discharge_low_t < -40.0f || sp.discharge_low_t > 40.0f) return false;
  if (sp.discharge_high_t < 20.0f || sp.discharge_high_t > 100.0f) return false;
  return true;
}

bool pollBmsAlarmStatus(int bms_id) {
  if (bms_id < 0 || bms_id >= MAX_BMS) return false;
  uint8_t req[1] = {(uint8_t)bms_id};
  TBFrame fr;
  String err;
  if (!tbRequestFrame((uint8_t)bms_id, 0x44, req, 1, fr, err)) return false;
  TBAlarmInfoParsed a;
  if (!tbParseAlarmInfoFromFrame(fr, a)) return false;
  g_bms_alarm_bits[bms_id] = a.status_bits;
  g_bms_alarm_status_count[bms_id] = a.status_count;
  g_bms_alarm_last_seen[bms_id] = millis();
  return true;
}

bool pollBmsSystemParameter(int bms_id) {
  if (bms_id < 0 || bms_id >= MAX_BMS) return false;
  TBFrame fr;
  String err;
  if (!tbRequestFrame((uint8_t)bms_id, 0x47, nullptr, 0, fr, err)) return false;
  TBSysParam sp;
  if (!tbParseSystemParameterFromFrame(fr, sp)) return false;
  if (!tbSystemParamPlausible(sp)) {
    addToLog("Spike Filter: SystemParam out of range BMS" + String(bms_id), true);
    return false;
  }
  bool prev_fresh = (g_bms_sys_last_seen[bms_id] > 0) && (millis() - g_bms_sys_last_seen[bms_id] < 300000);
  if (prev_fresh) {
    if (fabs(sp.module_high_v - g_bms_sys_module_high_v[bms_id]) > 5.0f) {
      addToLog("Spike Filter: module_high jump BMS" + String(bms_id), true);
      return false;
    }
    if (fabs(sp.charge_current_max_a - g_bms_sys_charge_max_a[bms_id]) > 100.0f) {
      addToLog("Spike Filter: charge_max jump BMS" + String(bms_id), true);
      return false;
    }
    if (fabs(sp.discharge_current_max_a - g_bms_sys_discharge_max_a[bms_id]) > 100.0f) {
      addToLog("Spike Filter: discharge_max jump BMS" + String(bms_id), true);
      return false;
    }
  }
  g_bms_sys_module_high_v[bms_id] = sp.module_high_v;
  g_bms_sys_module_under_v[bms_id] = sp.module_under_v;
  g_bms_sys_charge_max_a[bms_id] = sp.charge_current_max_a;
  g_bms_sys_discharge_max_a[bms_id] = sp.discharge_current_max_a;
  g_bms_sys_cell_high_v[bms_id] = sp.cell_high_v;
  g_bms_sys_charge_low_t[bms_id] = sp.charge_low_t;
  g_bms_sys_charge_high_t[bms_id] = sp.charge_high_t;
  g_bms_sys_discharge_low_t[bms_id] = sp.discharge_low_t;
  g_bms_sys_discharge_high_t[bms_id] = sp.discharge_high_t;
  g_bms_sys_last_seen[bms_id] = millis();
  return true;
}

// ==========================================
// LOGGING, AUTH, LED STATUS, AND LOGIN RATE LIMITING
// ==========================================
void addToLog(String msg, bool error) {
  #ifdef MODE_SMART_WIFI
    String clr = error ? "log-err" : "log-ok";
    if (debug_log.length() > 6000) debug_log = debug_log.substring(0, 5000);
    String tm = getTimeStr();
    debug_log = "<div class='" + clr + "'>[" + tm + "] " + msg + "</div>" + debug_log;
    if(error || g_serial_debug) Serial.println("[" + tm + "] " + msg);
  #else
    if(error) Serial.println(msg);
  #endif
}

String getTimeStr() {
  struct tm t; if (!getLocalTime(&t)) return String(millis() / 1000) + "s";
  char b[20]; strftime(b, sizeof(b), "%H:%M:%S", &t); return String(b);
}

String sha256Hash(const String &input) {
  uint8_t hash[32];
  mbedtls_sha256_context ctx;
  mbedtls_sha256_init(&ctx);
  mbedtls_sha256_starts(&ctx, 0);
  mbedtls_sha256_update(&ctx, (const unsigned char*)input.c_str(), input.length());
  mbedtls_sha256_finish(&ctx, hash);
  mbedtls_sha256_free(&ctx);
  String hex; hex.reserve(64);
  for (int i = 0; i < 32; i++) hex += toHex2(hash[i]);
  return hex;
}

// Cookie-based session authentication (replaces browser Basic Auth popup)
// Session token is generated at boot, stored in RAM only. Reboot = new session.
#ifdef MODE_SMART_WIFI
String g_session_token = "";  // Random 32-char hex, generated in setup()

void generateSessionToken() {
  g_session_token = "";
  for (int i = 0; i < 16; i++) g_session_token += toHex2((uint8_t)esp_random());
}

String getSessionCookie() {
  if (!server.hasHeader("Cookie")) return "";
  String cookies = server.header("Cookie");
  int pos = cookies.indexOf("tbsid=");
  if (pos < 0) return "";
  int start = pos + 6;
  int end = cookies.indexOf(';', start);
  if (end < 0) end = cookies.length();
  return cookies.substring(start, end);
}

bool checkAuth(bool jsonMode = false) {
  if (!g_auth_enable || g_auth_hash.length() == 0) return true;
  // Check session cookie
  if (getSessionCookie() == g_session_token && g_session_token.length() > 0) return true;
  // No valid session
  if (jsonMode) {
    server.send(401, "application/json", "{\"auth\":false}");
  } else {
    server.sendHeader("Location", "/login");
    server.send(302, "text/plain", "Redirecting to login");
  }
  return false;
}

void handleLogin() {
  // Rate limiting: max 5 failed attempts, then 60s lockout
  static uint8_t login_fails = 0;
  static unsigned long lockout_until = 0;
  if (millis() < lockout_until) {
    server.send(429, "text/plain", "Too many attempts. Try again later.");
    return;
  }
  if (server.method() == HTTP_POST) {
    // Validate credentials
    String user = server.arg("user");
    String pass = server.arg("pass");
    if (user == g_auth_user && sha256Hash(pass) == g_auth_hash) {
      // Success: set session cookie and redirect to dashboard
      login_fails = 0;
      server.sendHeader("Set-Cookie", "tbsid=" + g_session_token + "; Path=/; HttpOnly; SameSite=Strict");
      server.sendHeader("Location", "/");
      server.send(302, "text/plain", "Login OK");
      addToLog("Auth: login success (" + user + ")", false);
      return;
    }
    login_fails++;
    addToLog("Auth: login failed (" + user + "), attempt " + String(login_fails), true);
    if (login_fails >= 5) {
      lockout_until = millis() + 60000;
      addToLog("Auth: lockout active for 60s after " + String(login_fails) + " failures", true);
      login_fails = 0;
    }
  }
  // Serve login form (GET, or failed POST)
  bool failed = (server.method() == HTTP_POST);
  String h = R"LOGIN(<!DOCTYPE html><html><head><meta charset="UTF-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>Login</title><style>
:root{--bg:#f3f4f6;--bg2:#fff;--tx:#1f2937;--tx2:#6b7280;--acc:#059669;--red:#ef4444;--r:12px}
@media(prefers-color-scheme:dark){:root{--bg:#0f172a;--bg2:#1a1a2e;--tx:#e2e8f0;--tx2:#94a3b8}}
*{box-sizing:border-box;margin:0;padding:0}body{background:var(--bg);color:var(--tx);font-family:-apple-system,sans-serif;display:flex;justify-content:center;align-items:center;min-height:100vh}
.b{background:var(--bg2);border-radius:var(--r);padding:32px;max-width:340px;width:90%;box-shadow:0 4px 12px rgba(0,0,0,0.1)}
h2{font-size:16px;font-weight:500;margin-bottom:4px}
.s{color:var(--tx2);font-size:12px;margin-bottom:20px}
.e{color:var(--red);font-size:12px;margin-bottom:12px}
label{display:block;font-size:11px;color:var(--tx2);margin-bottom:3px}
input{width:100%;padding:9px 10px;border:1px solid rgba(128,128,128,0.3);border-radius:8px;background:var(--bg);color:var(--tx);font-size:13px;margin-bottom:12px;outline:none}
input:focus{border-color:var(--acc)}
button{width:100%;padding:10px;background:var(--acc);color:#fff;border:none;border-radius:8px;font-size:13px;font-weight:500;cursor:pointer}
button:hover{opacity:0.9}
</style></head><body><div class="b"><h2>TopBand BMS Gateway</h2><div class="s">Sign in to continue</div>)LOGIN";
  if (failed) h += "<div class=\"e\">Invalid username or password</div>";
  h += R"LOGIN2(<form method="POST" action="/login">
<label>Username</label><input type="text" name="user" autocomplete="username" autofocus required>
<label>Password</label><input type="password" name="pass" autocomplete="current-password" required>
<button type="submit">Sign in</button></form>
</div></body></html>)LOGIN2";
  server.send(200, "text/html", h);
}

void handleLogout() {
  server.sendHeader("Set-Cookie", "tbsid=; Path=/; Max-Age=0");
  server.sendHeader("Location", "/login");
  server.send(302, "text/plain", "Logged out");
}
#endif

void handleLedStatus() {
  unsigned long m = millis();
  #ifdef MODE_SMART_WIFI
  if(WiFi.status() != WL_CONNECTED && WiFi.getMode() != WIFI_AP) { 
      int val = (int)((sin(m / 300.0) + 1.0) * 60.0); setLed(val, 0, val); return; 
  }
  #endif
  if (victronData.activePacks == 0) {
      int val = (int)((sin(m / 200.0) + 1.0) * 100.0); setLed(0, 0, val); 
  } else if( (alarm_flags & 0x02) || (alarm_flags & 0x08) || (alarm_flags & 0x10) || (alarm_flags & 0x40) ) {
      bool on = (m / 80) % 2; setLed(on ? 255 : 0, 0, 0); 
  } else if (alarm_flags & 0x20) {
      bool on = (m / 500) % 2; setLed(on ? 255 : 0, on ? 160 : 0, 0); 
  } else {
      int val = (int)((sin(m / 1500.0) + 1.0) * 60.0); setLed(0, val, 0); 
  }
}

// --- AP MODE CALLBACK ---
#ifdef MODE_SMART_WIFI
void configModeCallback(WiFiManager *myWiFiManager) {
  Serial.println("AP Mode Active");
  setLed(255, 0, 0); // Red = AP mode
}
#endif

// ==========================================
// BMS DATA PARSER (called on Core 0 after each RS485 response)
// Decodes TopBand analog data frame: cell voltages, temperatures,
// pack voltage/current/SOC/SOH. Includes spike filter to reject
// implausible jumps (>5V voltage, >250A current, >20% SOC).
// Updates per-BMS communication stats (ok/errors/spikes).
// ==========================================
void parseTopband(String raw, int addr) {
  #ifdef MODE_SMART_WIFI
    if(g_serial_debug) { Serial.print("RX BMS "); Serial.print(addr); Serial.print(": "); Serial.println(raw); }
    #if HAS_SD_CARD
    if(g_sd_spy && sd_ok) {
        File f = SD.open("/spy.txt", FILE_APPEND);
        if(f) { f.printf("[%s] BMS%d: %s\n", getTimeStr().c_str(), addr, raw.c_str()); f.close(); }
    }
    #endif
  #endif
  if (raw.length() < 20) { addToLog("Err: Short Frame", true); if(addr>=0&&addr<MAX_BMS)g_bms_stats[addr].errors++; return; }
  stat_rx_count++; uint8_t b[512]; int blen = 0;
  for (int i = 0; i < raw.length() - 1; i += 2) { if (blen >= 511) break; b[blen++] = (parse_hex(raw[i]) << 4) | parse_hex(raw[i + 1]); }
  int idx = -1; for (int i = 0; i < blen - 1; i++) { if (b[i] == 0xD0 && b[i + 1] == 0x7C) { idx = i; break; } }
  if (idx == -1) return;
  int p = idx + 2; p++; int cells = b[p++];
  if (g_force_cell_count > 0) cells = g_force_cell_count;
  if (cells > 32) cells = 32;
  if (cells < 0) cells = 0;
  g_detected_cells = cells;
  float vSum = 0, minV = 99, maxV = 0; int minI = 0, maxI = 0;
  for (int i = 0; i < cells; i++) {
    if (p + 1 >= blen) return;
    float v = get_u16(b, p) / 1000.0; bms[addr].cells[i] = v;
    if (v > 0.1) { vSum += v; if (v < minV) { minV = v; minI = i + 1; } if (v > maxV) { maxV = v; maxI = i + 1; } }
    p += 2;
  }
  bms[addr].cell_count = cells; bms[addr].minCellV = minV; bms[addr].maxCellV = maxV;
  bms[addr].minCellIdx = minI; bms[addr].maxCellIdx = maxI; bms[addr].avgCellV = (cells > 0) ? vSum / cells : 0;
  if (p < blen) {
    int t_cnt = b[p++]; if (t_cnt > 8) t_cnt = 8; bms[addr].temp_count = t_cnt;
    float max_t = -99; for (int i = 0; i < t_cnt; i++) {
      if (p + 1 >= blen) break; float t = (get_u16(b, p) - 2731) / 10.0;
      if (t < -50 || t > 150) t = 25.0; bms[addr].temps[i] = t; if (t > max_t) max_t = t; p += 2;
    }
    bms[addr].maxTemp = max_t; bms[addr].avgTemp = max_t; 
  }
  if (p + 10 < blen) {
    float cur = get_s16(b, p) * 0.01; p += 2; float volt = get_u16(b, p) * 0.01; p += 2; float rem = get_u16(b, p) * 0.01; p += 3; float full = get_u16(b, p) * 0.01; p += 2;
    float v_min = (cells > 0) ? (cells * 2.0f) : 20.0f;
    float v_max = (cells > 0) ? (cells * 4.05f) : 65.0f;
    if (volt < v_min || volt > v_max || abs(cur) > 500 || rem < 0 || full < 0 || full > 2000) { addToLog("Bad Main Data", true); g_bms_stats[addr].errors++; return; }
    int new_soc_calc = (full > 0) ? (int)((rem / full) * 100.0) : 0; if (new_soc_calc > 100) new_soc_calc = 100; if (new_soc_calc < 0) new_soc_calc = 0;
    int new_soc = new_soc_calc;
    int new_soh = 100;

    // Newer payload variant: cycle(2B), SOC(1B), SOH(1B) after full_ah.
    // Accept only if plausible against calculated SOC, then fallback to legacy parse.
    int meta = p;
    uint16_t new_cycles = 0;
    if (meta + 3 < blen) {
      new_cycles = get_u16(b, meta);
      uint8_t raw_soc = b[meta + 2];
      uint8_t raw_soh = b[meta + 3];
      bool extended_layout = (meta + 7 < blen);
      bool raw_valid = (raw_soc <= 100) && (raw_soh <= 100);
      new_soc = tbSelectSocValue(new_soc_calc, (int)raw_soc, raw_valid, extended_layout);
      if (raw_valid && (extended_layout || g_soc_source_mode == 1 || abs((int)raw_soc - new_soc_calc) <= 15)) {
        new_soh = (int)raw_soh;
      } else {
        uint16_t s16 = get_u16(b, meta + 2);
        if (s16 <= 100) new_soh = (int)s16;
      }
    }

    bool prev_valid = bms[addr].valid && (millis() - bms[addr].last_seen < 15000);
    if (prev_valid) {
      if (fabs(volt - bms[addr].voltage) > 5.0f) { addToLog("Spike Filter: Voltage jump BMS" + String(addr), true); g_bms_stats[addr].spikes++; return; }
      if (fabs(cur - bms[addr].current) > 250.0f) { addToLog("Spike Filter: Current jump BMS" + String(addr), true); g_bms_stats[addr].spikes++; return; }
      if (abs(new_soc - bms[addr].soc) > 20) { addToLog("Spike Filter: SOC jump BMS" + String(addr), true); g_bms_stats[addr].spikes++; return; }
    }

    bms[addr].current = cur; bms[addr].voltage = volt; bms[addr].rem_ah = rem; bms[addr].full_ah = full;
    bms[addr].soc = new_soc;
    bms[addr].soh = new_soh;
    bms[addr].cycles = new_cycles;
  }
  bms[addr].valid = true; bms[addr].last_seen = millis(); g_bms_stats[addr].ok++;
}

float calcFactor(float t, float min, float max) {
    if (t < min) return 0.0; if (t > max) return 0.0;
    float soft_zone = 5.0; 
    if (t < (min + soft_zone)) return 0.2; 
    if (t > (max - soft_zone)) return 0.5; 
    return 1.0; 
}

float getEffectiveChargeMinTemp() {
  if (g_expert_mode) return g_tc_min;
  return (g_tc_min < 5.0f) ? 5.0f : g_tc_min;
}

float getEffectiveDischargeMinTemp() {
  if (g_expert_mode) return g_td_min;
  return (g_td_min < -20.0f) ? -20.0f : g_td_min;
}
void enforceCellBasedSafetyCaps() {
  int cells = g_force_cell_count > 0 ? g_force_cell_count : g_detected_cells;
  if (cells <= 0 || cells > 32) return;

  float cap_cvl = ((float)cells) * 3.50f;
  float cap_safe = ((float)cells) * 3.55f;
  float cap_cell = 3.55f;

  if (g_cvl_voltage > cap_cvl) g_cvl_voltage = cap_cvl;
  if (g_safe_volt > cap_safe) g_safe_volt = cap_safe;
  if (g_safe_cell > cap_cell) g_safe_cell = cap_cell;
  if (g_maint_target_v > g_safe_volt) g_maint_target_v = g_safe_volt;
}

void saveAutoBalanceTimestamp(uint32_t ts) {
  preferences.begin("gateway", false);
  preferences.putULong("ab_last", ts);
  preferences.end();
}

// ==========================================
// SAFETY LOGIC & VICTRON DATA AGGREGATION (runs on Core 0 every 500ms)
// Aggregates all BMS packs into victronData struct for CAN output.
// Applies temperature throttling (soft zones), protocol-level current caps
// from BMS 0x47, overvolt/undervolt cutoffs, and alarm flag management.
// State-transition logging: safety events and throttle changes are logged
// only when they change, not on every cycle.
// Alarm flags bitmask: 0x02=overvolt, 0x08=temp stop, 0x10=undervolt,
//   0x20=cell imbalance, 0x40=BMS status alarm, 0x80=no packs online
// ==========================================
void calculateVictronData() {
  float sumI = 0, sumV = 0, sumSOC = 0, sumSOH = 0, sumT = 0, sumCap = 0, sumRem = 0;
  float proto_ccl_cap = 0, proto_dcl_cap = 0, proto_cvl_cap = 999.0f;
  bool proto_under_v_hit = false;
  int first_proto_idx = -1;
  int proto_count = 0;
  int count = 0; alarm_flags = 0; sys_error_msg = "OK";
  #ifdef MODE_SIMPLE_CABLE
    if(g_detected_cells > 0) { g_cvl_voltage = g_detected_cells * 3.5; g_safe_volt = g_detected_cells * 3.65; }
  #endif
  enforceCellBasedSafetyCaps();

  float t_check_val = 25.0; 
  for (int i = 0; i < g_bms_count; i++) {
    bool on = bms[i].valid && (millis() - bms[i].last_seen < 120000);
    if (simulation_active) on = true;
    // Detect BMS going offline
    if (g_bms_was_online[i] && !on) {
      addToLog("ALARM: BMS" + String(i) + " went offline", true);
      if (sys_error_msg == "OK") sys_error_msg = "ALARM: BMS" + String(i) + " OFFLINE";
    }
    g_bms_was_online[i] = on;
    if (on) {
      bool alarm_fresh = (g_bms_alarm_last_seen[i] > 0) && (millis() - g_bms_alarm_last_seen[i] < 60000);
      if (alarm_fresh) {
        int cells_for_filter = (bms[i].cell_count > 0) ? bms[i].cell_count : g_detected_cells;
        float temp_limit = (g_tc_max < g_td_max) ? g_tc_max : g_td_max;
        uint64_t crit_bits = tbFilterCriticalAlarmBits(
            g_bms_alarm_bits[i],
            bms[i].voltage,
            bms[i].maxTemp,
            cells_for_filter,
            temp_limit,
            bms[i].maxCellV,
            g_bms_sys_cell_high_v[i],
            g_bms_sys_module_high_v[i]);
        if (crit_bits != 0) {
          const uint64_t uv_alarm_bits = (1ULL << 12) | (1ULL << 13);
          if (crit_bits & uv_alarm_bits) {
            alarm_flags |= 0x10;
            if (sys_error_msg == "OK") sys_error_msg = "ALARM: PACK UNDERVOLT";
          } else {
            alarm_flags |= 0x40;
            if (sys_error_msg == "OK") sys_error_msg = "ALARM: BMS STATUS";
          }
        }
      }
      bool sp_fresh = (g_bms_sys_last_seen[i] > 0) && (millis() - g_bms_sys_last_seen[i] < 300000);
      if (sp_fresh) {
        if (first_proto_idx < 0) first_proto_idx = i;
        if (g_bms_sys_charge_max_a[i] > 0.1f) proto_ccl_cap += g_bms_sys_charge_max_a[i];
        if (g_bms_sys_discharge_max_a[i] > 0.1f) proto_dcl_cap += g_bms_sys_discharge_max_a[i];
        int cells_for_cap = (bms[i].cell_count > 0) ? bms[i].cell_count : g_detected_cells;
        float cvl_cap = g_bms_sys_module_high_v[i] - 0.20f;
        if (cells_for_cap > 0 && cells_for_cap <= 32) {
          float chem_cap = ((float)cells_for_cap) * 3.60f;
          if (cvl_cap > chem_cap) cvl_cap = chem_cap;
        }
        if (g_safe_volt > 40.0f && cvl_cap > (g_safe_volt - 0.20f)) cvl_cap = g_safe_volt - 0.20f;
        if (g_cvl_voltage > 40.0f && cvl_cap > g_cvl_voltage) cvl_cap = g_cvl_voltage;
        if (cvl_cap > 40.0f && cvl_cap < proto_cvl_cap) proto_cvl_cap = cvl_cap;
        int cells_for_uv = (bms[i].cell_count > 0) ? bms[i].cell_count : g_detected_cells;
        if (tbShouldFlagProtoUnderVolt(bms[i].voltage, g_bms_sys_module_under_v[i], cells_for_uv)) proto_under_v_hit = true;
        proto_count++;
      }
      if(bms[i].voltage > g_safe_volt) { alarm_flags |= 0x02; sys_error_msg = "ALARM: PACK OVERVOLT"; }
      if(bms[i].maxCellV > g_safe_cell) { alarm_flags |= 0x02; sys_error_msg = "ALARM: CELL OVERVOLT"; }
      if(g_temp_mode == 0) t_check_val = bms[i].maxTemp; else t_check_val = bms[i].avgTemp; 
      if((bms[i].maxCellV - bms[i].minCellV) > g_safe_drift) { alarm_flags |= 0x20; if(sys_error_msg == "OK") sys_error_msg = "WARN: CELL IMBALANCE"; }
      sumI += bms[i].current; sumV += bms[i].voltage; sumSOC += bms[i].soc; sumSOH += bms[i].soh; sumT += bms[i].avgTemp; sumCap += bms[i].full_ah; sumRem += bms[i].rem_ah; count++;
    }
  }
  
  if (count > 0) {
    if (g_easy_mode && !g_expert_mode && !g_easy_auto_applied && first_proto_idx >= 0) {
      TBSysParam sp;
      sp.cell_high_v = g_bms_sys_cell_high_v[first_proto_idx];
      sp.module_high_v = g_bms_sys_module_high_v[first_proto_idx];
      sp.charge_low_t = g_bms_sys_charge_low_t[first_proto_idx];
      sp.charge_high_t = g_bms_sys_charge_high_t[first_proto_idx];
      sp.discharge_low_t = g_bms_sys_discharge_low_t[first_proto_idx];
      sp.discharge_high_t = g_bms_sys_discharge_high_t[first_proto_idx];
      sp.charge_current_max_a = g_bms_sys_charge_max_a[first_proto_idx];
      sp.discharge_current_max_a = g_bms_sys_discharge_max_a[first_proto_idx];

      float n_safe_cell, n_safe_volt, n_tc_min, n_tc_max, n_td_min, n_td_max, n_chg_a, n_dis_a, n_cvl_v;
      tbBuildAutoCfgSuggestion(sp, 20.0f, 0.30f, 2.0f, 10.0f,
                               n_safe_cell, n_safe_volt, n_tc_min, n_tc_max, n_td_min, n_td_max, n_chg_a, n_dis_a, n_cvl_v);
      g_safe_cell = n_safe_cell;
      g_safe_volt = n_safe_volt;
      g_tc_min = n_tc_min;
      g_tc_max = n_tc_max;
      g_td_min = n_td_min;
      g_td_max = n_td_max;
      g_charge_amps = n_chg_a;
      g_discharge_amps = n_dis_a;
      g_cvl_voltage = n_cvl_v;
      g_easy_auto_applied = true;
      addToLog("Easy Mode: Auto config applied from BMS system params", false);
    }

    victronData.activePacks = count; victronData.totalCurrent = sumI; victronData.avgVoltage = sumV / count;
    victronData.totalPower = sumI * victronData.avgVoltage; victronData.avgSOC = sumSOC / count;
    victronData.avgSOH = sumSOH / count; victronData.avgTemp = sumT / count; victronData.totalCapacity = sumCap; victronData.remainCapacity = sumRem;
    
    float eff_tc_min = getEffectiveChargeMinTemp();
    float eff_td_min = getEffectiveDischargeMinTemp();
    factor_charge = calcFactor(t_check_val, eff_tc_min, g_tc_max);
    factor_discharge = calcFactor(t_check_val, eff_td_min, g_td_max);

    float safe_chg = count * g_charge_amps * factor_charge;
    float safe_dis = count * g_discharge_amps * factor_discharge;

    if (proto_count > 0) {
      if (proto_ccl_cap > 0.1f && safe_chg > proto_ccl_cap) safe_chg = proto_ccl_cap;
      if (proto_dcl_cap > 0.1f && safe_dis > proto_dcl_cap) safe_dis = proto_dcl_cap;
      if (proto_cvl_cap < 900.0f) g_runtime_cvl = proto_cvl_cap;
      else g_runtime_cvl = g_cvl_voltage;
      g_runtime_proto_ccl_cap = proto_ccl_cap;
      g_runtime_proto_dcl_cap = proto_dcl_cap;
    } else {
      g_runtime_cvl = g_cvl_voltage;
      g_runtime_proto_ccl_cap = 0.0f;
      g_runtime_proto_dcl_cap = 0.0f;
    }

    if(factor_charge == 0.0) { alarm_flags |= 0x08; sys_error_msg = "INFO: TEMP CHARGE STOP"; }
    if(factor_discharge == 0.0) { alarm_flags |= 0x08; sys_error_msg = "INFO: TEMP DISCHG STOP"; }
    g_runtime_proto_uv_hit = proto_under_v_hit;
    if(proto_under_v_hit) { alarm_flags |= 0x10; safe_dis = 0; if(sys_error_msg == "OK") sys_error_msg = "ALARM: PACK UNDERVOLT"; }
    if( (alarm_flags & 0x02) || (alarm_flags & 0x10) || (alarm_flags & 0x40) ) { safe_chg = 0; safe_dis = 0; } 
    // State-transition logging (only log on change to avoid spam)
    static uint8_t prev_alarm_flags = 0;
    static float prev_factor_charge = 1.0f, prev_factor_discharge = 1.0f;
    if ((alarm_flags & 0x02) && !(prev_alarm_flags & 0x02)) addToLog("SAFETY: Overvoltage cutoff triggered", true);
    if ((alarm_flags & 0x10) && !(prev_alarm_flags & 0x10)) addToLog("SAFETY: Undervoltage cutoff triggered", true);
    if ((alarm_flags & 0x08) && !(prev_alarm_flags & 0x08)) addToLog("SAFETY: Temperature cutoff active", true);
    if ((alarm_flags & 0x20) && !(prev_alarm_flags & 0x20)) addToLog("SAFETY: Cell imbalance detected", true);
    if ((alarm_flags & 0x40) && !(prev_alarm_flags & 0x40)) addToLog("SAFETY: BMS alarm status active", true);
    if (factor_charge < 1.0f && prev_factor_charge >= 1.0f) addToLog("THROTTLE: Charge current reduced to " + String((int)(factor_charge * 100)) + "% (temp)", false);
    if (factor_discharge < 1.0f && prev_factor_discharge >= 1.0f) addToLog("THROTTLE: Discharge current reduced to " + String((int)(factor_discharge * 100)) + "% (temp)", false);
    if (factor_charge >= 1.0f && prev_factor_charge < 1.0f) addToLog("THROTTLE: Charge current restored to 100%", false);
    if (factor_discharge >= 1.0f && prev_factor_discharge < 1.0f) addToLog("THROTTLE: Discharge current restored to 100%", false);
    prev_alarm_flags = alarm_flags; prev_factor_charge = factor_charge; prev_factor_discharge = factor_discharge; 
    if (!g_maint_charge_mode) {
      if (victronData.avgSOC >= 99) safe_chg = count * 2.0;
      if (victronData.avgSOC == 100) safe_chg = 0.0;
    } else {
      float maint_target = g_maint_target_v;
      if (maint_target < 40.0f) maint_target = 40.0f;
      if (maint_target > g_safe_volt) maint_target = g_safe_volt;
      if (victronData.avgVoltage >= maint_target && safe_chg > count * 2.0f) safe_chg = count * 2.0f;
    }

    // Optional auto-balance watchdog: if target was not reached for 30 days, keep a small charge allowance.
    g_runtime_auto_balance_due = false;
    if (g_auto_balance_enable) {
      time_t t_now;
      time(&t_now);
      if (t_now > 1700000000) {
        uint32_t now_ts = (uint32_t)t_now;
        float bal_target = g_maint_target_v;
        if (bal_target < 40.0f) bal_target = 40.0f;
        if (bal_target > g_safe_volt) bal_target = g_safe_volt;
        bool reached_now = victronData.avgVoltage >= bal_target;
        if (reached_now && !g_runtime_balance_reached_prev) {
          g_auto_balance_last_ts = now_ts;
          saveAutoBalanceTimestamp(g_auto_balance_last_ts);
        }
        g_runtime_balance_reached_prev = reached_now;

        if (!reached_now && !g_maint_charge_mode) {
          const uint32_t window_s = 30UL * 24UL * 3600UL;
          if (g_auto_balance_last_ts == 0 || (now_ts > g_auto_balance_last_ts && (now_ts - g_auto_balance_last_ts) > window_s)) {
            g_runtime_auto_balance_due = true;
            if (!(alarm_flags & 0x02) && !(alarm_flags & 0x10) && factor_charge > 0.0f) {
              float hold_chg = count * 2.0f;
              if (safe_chg < hold_chg) safe_chg = hold_chg;
              if (sys_error_msg == "OK") sys_error_msg = "INFO: AUTO BALANCE ACTIVE";
            }
          }
        }
      }
    }
    
    victronData.maxChargeCurrent = safe_chg; victronData.maxDischargeCurrent = safe_dis;
  } else { victronData.activePacks = 0; victronData.totalCurrent = 0; victronData.totalPower = 0; victronData.maxChargeCurrent = 0; victronData.maxDischargeCurrent = 0; g_runtime_proto_uv_hit = false; g_runtime_auto_balance_due = false; g_runtime_balance_reached_prev = false; alarm_flags |= 0x80; }
}

// ==========================================
// CAN BUS OUTPUT (Victron/Pylontech/SMA protocol)
// Sends battery data to inverter: 0x351=limits, 0x355=SOC/SOH,
// 0x356=voltage/current/temp, 0x35A=alarms, 0x35E=manufacturer ID
// ==========================================
void sendCanFrame(uint32_t id, uint8_t *data) {
  twai_message_t m; m.identifier = id; m.extd = 0; m.data_length_code = 8; memcpy(m.data, data, 8);
  esp_err_t result = twai_transmit(&m, pdMS_TO_TICKS(5));
  if (result == ESP_OK) { can_error_flag = false; debug_can_status = "OK - Connected"; }
  else {
    twai_status_info_t status; twai_get_status_info(&status);
    if (status.state == TWAI_STATE_BUS_OFF) {
      can_error_flag = true;
      debug_can_status = "Bus off - check cable and termination (120 Ohm)";
      twai_initiate_recovery();
    } else if (status.state == TWAI_STATE_RECOVERING) {
      can_error_flag = true;
      debug_can_status = "Bus recovery in progress...";
    } else if (status.state == TWAI_STATE_STOPPED) {
      can_error_flag = true;
      debug_can_status = "Bus stopped - restart required";
    } else if (result == 0x107) {
      can_error_flag = false;
      if (status.tx_error_counter > 127) {
        debug_can_status = "No inverter found - check cable";
      } else {
        debug_can_status = "Waiting for inverter (no device on CAN bus)";
      }
    } else if (result == 0x103) {
      can_error_flag = true;
      debug_can_status = "CAN driver not started - restart needed";
    } else {
      can_error_flag = true;
      debug_can_status = "TX error - check CAN connection";
    }
  }
  stat_tx_count++;
}

void sendVictronCAN() {
  uint8_t d[8] = {0}; float cvl = (g_runtime_cvl > 0.1f) ? g_runtime_cvl : g_cvl_voltage;
  int cv = (int)(cvl * 10), ccl = (int)(victronData.maxChargeCurrent * 10), dcl = (int)(victronData.maxDischargeCurrent * 10);
  d[0] = cv & 0xFF; d[1] = cv >> 8; d[2] = ccl & 0xFF; d[3] = ccl >> 8; d[4] = dcl & 0xFF; d[5] = dcl >> 8;
  sendCanFrame(0x351, d); delay(2);
  d[0] = (int)victronData.avgSOC & 0xFF; d[1] = (int)victronData.avgSOC >> 8; d[2] = (int)victronData.avgSOH & 0xFF; d[3] = (int)victronData.avgSOH >> 8;
  int cap = (int)(victronData.totalCapacity * 10); d[4] = cap & 0xFF; d[5] = cap >> 8; d[6]=0; d[7]=0;
  sendCanFrame(0x355, d); delay(2);
  int v = (int)(victronData.avgVoltage * 100), i = (int)(victronData.totalCurrent * 10), t = (int)(victronData.avgTemp * 10);
  d[0] = v & 0xFF; d[1] = v >> 8; d[2] = i & 0xFF; d[3] = i >> 8; d[4] = t & 0xFF; d[5] = t >> 8; d[6]=0; d[7]=0;
  sendCanFrame(0x356, d); delay(2);
  memset(d, 0, 8); 
  if(alarm_flags & 0x01) d[4] |= 0x80; if(alarm_flags & 0x40) d[4] |= 0x80; if(alarm_flags & 0x02) d[4] |= 0x40; if(alarm_flags & 0x08) d[4] |= 0x20; if(alarm_flags & 0x10) d[4] |= 0x10; 
  if(g_can_protocol == 2 && victronData.maxChargeCurrent < 0.1) d[4] |= 0x02; if(g_can_protocol == 2 && victronData.maxDischargeCurrent < 0.1) d[4] |= 0x01; 
  sendCanFrame(0x35A, d); delay(2);
  if(g_can_protocol == 1) { char n[] = "PYLON   "; sendCanFrame(0x35E, (uint8_t*)n); }
  else if(g_can_protocol == 2) { char n[] = "SMA     "; sendCanFrame(0x35E, (uint8_t*)n); }
  else { char n[] = "TOPBAND "; sendCanFrame(0x35E, (uint8_t*)n); } 
}

#ifdef MODE_SMART_WIFI
// ==========================================
// HISTORY, ENERGY PERSISTENCE & EXPORT (NVS namespace "h")
// Chart data: 4 types x 960 points (48h at 3min intervals), avg/min/max.
// Energy: today + rolling 7 days + monthly totals, all NVS-persistent.
// CSV export at /export endpoint (960 rows, all 4 chart types).
// All 4 chart avg arrays are persisted (power/volt/soc/temp, ~1920 bytes each).
// Min/max arrays are initialized from avg on load, rebuild with real data.
// Energy counters (ei/eo) track today's charge/discharge in kWh.
// Save interval: every 15 minutes + on OTA + on Save&Reboot.
// ==========================================
void loadHistory() {
  histStore.begin("h", true);
  energy_in_today = histStore.getFloat("ei", 0);
  energy_out_today = histStore.getFloat("eo", 0);
  if (histStore.getBytesLength("edi") == sizeof(energy_days_in)) {
    histStore.getBytes("edi", energy_days_in, sizeof(energy_days_in));
    histStore.getBytes("edo", energy_days_out, sizeof(energy_days_out));
  }
  if (histStore.getBytesLength("emi") == sizeof(energy_months_in)) {
    histStore.getBytes("emi", energy_months_in, sizeof(energy_months_in));
    histStore.getBytes("emo", energy_months_out, sizeof(energy_months_out));
  }
  historyIdx = histStore.getInt("hidx", 0);
  // V2.61: historyFilled distinguishes real 0 values from unwritten slots.
  // Migration: if key missing but data exists (hdat present), assume buffer already full (upgrade from V2.60).
  if (histStore.isKey("hfill")) {
    historyFilled = histStore.getInt("hfill", 0);
    if (historyFilled < 0) historyFilled = 0;
    if (historyFilled > HISTORY_LEN) historyFilled = HISTORY_LEN;
  } else {
    historyFilled = (histStore.getBytesLength("hdat") == HISTORY_LEN * sizeof(int16_t)) ? HISTORY_LEN : 0;
  }
  // Load all 4 chart avg arrays from NVS
  int16_t *avgArr[] = { powerHistory, voltHistory, socHistory, tempHistory };
  const char *avgKey[] = { "hdat", "hvlt", "hsoc", "htmp" };
  for (int k = 0; k < 4; k++) {
    size_t storedLen = histStore.getBytesLength(avgKey[k]);
    if (storedLen == HISTORY_LEN * sizeof(int16_t)) {
      histStore.getBytes(avgKey[k], avgArr[k], HISTORY_LEN * sizeof(int16_t));
    } else {
      for (int i = 0; i < HISTORY_LEN; i++) avgArr[k][i] = 0;
    }
  }
  // Initialize min/max to avg values (bands rebuild within one interval)
  for (int i = 0; i < HISTORY_LEN; i++) {
    powerMin[i] = powerHistory[i]; powerMax[i] = powerHistory[i];
    voltMin[i] = voltHistory[i]; voltMax[i] = voltHistory[i];
    socMin[i] = socHistory[i]; socMax[i] = socHistory[i];
    tempMin[i] = tempHistory[i]; tempMax[i] = tempHistory[i];
  }
  histStore.end();
}
void saveHistory(bool includeGraph) {
  histStore.begin("h", false);
  histStore.putFloat("ei", energy_in_today);
  histStore.putFloat("eo", energy_out_today);
  histStore.putBytes("edi", energy_days_in, sizeof(energy_days_in));
  histStore.putBytes("edo", energy_days_out, sizeof(energy_days_out));
  histStore.putBytes("emi", energy_months_in, sizeof(energy_months_in));
  histStore.putBytes("emo", energy_months_out, sizeof(energy_months_out));
  if(includeGraph) {
    histStore.putInt("hidx", historyIdx);
    histStore.putInt("hfill", historyFilled);
    histStore.putBytes("hdat", powerHistory, HISTORY_LEN * sizeof(int16_t));
    histStore.putBytes("hvlt", voltHistory, HISTORY_LEN * sizeof(int16_t));
    histStore.putBytes("hsoc", socHistory, HISTORY_LEN * sizeof(int16_t));
    histStore.putBytes("htmp", tempHistory, HISTORY_LEN * sizeof(int16_t));
  }
  histStore.end(); addToLog("Stats Saved", false);
}

// ==========================================
// V2.61: ALERT RING BUFFER FUNCTIONS
// ==========================================
// Storage: "gateway" namespace, keys "a_cnt", "a_hd", "a_dat" (blob).
// Size budget: 25 entries * sizeof(AlertEntry) = ~2.7 KB in NVS.

static uint32_t alertNowEpoch() {
  time_t now_t = time(nullptr);
  return (now_t > 1700000000UL) ? (uint32_t)now_t : 0;  // 0 = "no valid NTP"
}

void addAlertSrv(uint8_t sev, const String& msg) {
  if (msg.length() == 0) return;
  uint32_t ep = alertNowEpoch();
  uint32_t up = millis() / 1000;
  // Dedup: suppress if same message raised within the last 30s (scan last 5)
  int scan = g_alert_count < 5 ? g_alert_count : 5;
  for (int i = 1; i <= scan; i++) {
    int idx = (g_alert_head - i + ALERT_RING_SIZE) % ALERT_RING_SIZE;
    if (strncmp(g_alerts[idx].msg, msg.c_str(), ALERT_MSG_LEN - 1) == 0) {
      uint32_t prev_up = g_alerts[idx].uptime_s;
      if (up >= prev_up && (up - prev_up) < 30) return;
    }
  }
  AlertEntry& e = g_alerts[g_alert_head];
  e.ts = ep; e.uptime_s = up; e.sev = sev;
  strncpy(e.msg, msg.c_str(), ALERT_MSG_LEN - 1);
  e.msg[ALERT_MSG_LEN - 1] = 0;
  g_alert_head = (g_alert_head + 1) % ALERT_RING_SIZE;
  if (g_alert_count < ALERT_RING_SIZE) g_alert_count++;
  g_alerts_dirty = true;
}

void loadAlertsFromNvs() {
  preferences.begin("gateway", true);
  g_alert_count = preferences.getInt("a_cnt", 0);
  g_alert_head = preferences.getInt("a_hd", 0);
  if (g_alert_count < 0 || g_alert_count > ALERT_RING_SIZE) g_alert_count = 0;
  if (g_alert_head < 0 || g_alert_head >= ALERT_RING_SIZE) g_alert_head = 0;
  size_t expected = sizeof(g_alerts);
  if (g_alert_count > 0 && preferences.getBytesLength("a_dat") == expected) {
    preferences.getBytes("a_dat", g_alerts, expected);
  } else {
    memset(g_alerts, 0, sizeof(g_alerts));
    g_alert_count = 0; g_alert_head = 0;
  }
  preferences.end();
}

void saveAlertsToNvs() {
  preferences.begin("gateway", false);
  preferences.putInt("a_cnt", g_alert_count);
  preferences.putInt("a_hd", g_alert_head);
  preferences.putBytes("a_dat", g_alerts, sizeof(g_alerts));
  preferences.end();
  g_alerts_dirty = false;
  g_alerts_last_save = millis();
}

void clearAlertsAll() {
  memset(g_alerts, 0, sizeof(g_alerts));
  g_alert_count = 0; g_alert_head = 0;
  g_alerts_dirty = false;
  preferences.begin("gateway", false);
  preferences.remove("a_cnt");
  preferences.remove("a_hd");
  preferences.remove("a_dat");
  preferences.end();
  memset(&g_alert_state, 0, sizeof(g_alert_state));
  addToLog("Alerts cleared (RAM + NVS)", false);
}

// Called from main loop on Core 1. State-transition based (raises on rising edge,
// clears state flag on falling edge so alert re-raises if condition returns).
void alertDetectTick() {
  unsigned long now = millis();
  if (now - g_alerts_last_detect < 10000) return;
  g_alerts_last_detect = now;

  // --- System-level: no packs online ---
  bool no_packs_now = (victronData.activePacks == 0);
  if (no_packs_now && !g_alert_state.no_packs) addAlertSrv(2, "No BMS packs online");
  g_alert_state.no_packs = no_packs_now;

  // --- Per-BMS checks ---
  for (int i = 0; i < g_bms_count && i < MAX_BMS; i++) {
    bool online = bms[i].valid && (now - bms[i].last_seen < 120000);

    bool offline_now = !online;
    if (offline_now && !g_alert_state.bms_offline[i]) addAlertSrv(1, "BMS #" + String(i) + " offline");
    g_alert_state.bms_offline[i] = offline_now;
    if (!online) continue;

    float maxC = bms[i].maxCellV;
    float minC = bms[i].minCellV;
    float drift = (maxC > minC) ? (maxC - minC) : 0.0f;

    bool cm_near = (maxC >= g_safe_cell - 0.02f);
    if (cm_near && !g_alert_state.cell_max_near[i])
      addAlertSrv(1, "BMS #" + String(i) + " cell max " + String(maxC, 3) + "V near limit");
    g_alert_state.cell_max_near[i] = cm_near;

    bool cn_crit = (minC > 0 && minC <= 2.8f);
    if (cn_crit && !g_alert_state.cell_min_crit[i])
      addAlertSrv(2, "BMS #" + String(i) + " cell min " + String(minC, 3) + "V critically low");
    g_alert_state.cell_min_crit[i] = cn_crit;

    bool dr_near = (drift >= g_safe_drift - 0.03f);
    if (dr_near && !g_alert_state.drift_near[i])
      addAlertSrv(1, "BMS #" + String(i) + " cell drift " + String(drift, 3) + "V near limit");
    g_alert_state.drift_near[i] = dr_near;

    // Temperature: any sensor within 3C of discharge min
    bool tmp_near = false;
    for (int t = 0; t < bms[i].temp_count; t++) {
      if (bms[i].temps[t] <= g_td_min + 3.0f) { tmp_near = true; break; }
    }
    if (tmp_near && !g_alert_state.temp_near[i])
      addAlertSrv(1, "BMS #" + String(i) + " temperature approaching discharge min");
    g_alert_state.temp_near[i] = tmp_near;

    bool slow = (bms[i].soc <= 10);
    if (slow && !g_alert_state.soc_low[i])
      addAlertSrv(1, "BMS #" + String(i) + " SOC " + String(bms[i].soc) + "% critically low");
    g_alert_state.soc_low[i] = slow;
  }

  // --- System SOC ---
  bool sys_low_now = (victronData.activePacks > 0 && victronData.avgSOC <= 15.0f);
  if (sys_low_now && !g_alert_state.sys_soc_low)
    addAlertSrv(1, "System SOC " + String(victronData.avgSOC, 0) + "% low");
  g_alert_state.sys_soc_low = sys_low_now;

  // --- CAN bus error ---
  #ifdef MODE_SMART_WIFI
  bool can_err_now = (debug_can_status.indexOf("Err") >= 0 || debug_can_status.indexOf("FAIL") >= 0);
  if (can_err_now && !g_alert_state.can_error_state)
    addAlertSrv(2, "CAN bus error: " + debug_can_status);
  g_alert_state.can_error_state = can_err_now;

  // --- MQTT disconnect (if enabled and persistently failing) ---
  bool mq_fail_now = (g_mqtt_enable && !mqtt.connected() && mqtt_fail_count >= 3);
  if (mq_fail_now && !g_alert_state.mqtt_fail_state)
    addAlertSrv(1, "MQTT disconnected (" + String(mqtt_fail_count) + " failures)");
  g_alert_state.mqtt_fail_state = mq_fail_now;
  #endif

  // --- Throttled save: every 60s if dirty ---
  if (g_alerts_dirty && (now - g_alerts_last_save > 60000)) {
    saveAlertsToNvs();
  }
}

void calculateEnergy(float power_w) {
  unsigned long now = millis(); if (last_energy_calc == 0) { last_energy_calc = now; return; }
  double h = (now - last_energy_calc) / 3600000.0; last_energy_calc = now;
  double kw = power_w / 1000.0;
  if (kw > 0.001) energy_in_today += (kw * h); else if (kw < -0.001) energy_out_today += (fabs(kw) * h);
  struct tm t; if (getLocalTime(&t)) {
    // Day rollover: push today's totals into rolling 7-day array
    if (current_day != -1 && current_day != t.tm_mday) {
      for (int i = 6; i > 0; i--) { energy_days_in[i] = energy_days_in[i-1]; energy_days_out[i] = energy_days_out[i-1]; }
      energy_days_in[0] = energy_in_today; energy_days_out[0] = energy_out_today;
      // Month rollover: accumulate into monthly totals
      if (current_month != -1 && current_month != t.tm_mon) {
        for (int i = 11; i > 0; i--) { energy_months_in[i] = energy_months_in[i-1]; energy_months_out[i] = energy_months_out[i-1]; }
        energy_months_in[0] = 0; energy_months_out[0] = 0;
      }
      energy_months_in[0] += energy_in_today; energy_months_out[0] += energy_out_today;
      energy_in_today = 0; energy_out_today = 0;
      saveHistory(false);
      addToLog("Energy: day rollover, saved history", false);
    }
    current_day = t.tm_mday;
    current_month = t.tm_mon;
  }
}
#if HAS_SD_CARD
void initSD() {
  if (!g_sd_enable) return;
  SPI.begin(SD_SCLK, SD_MISO, SD_MOSI, SD_CS);
  if (SD.begin(SD_CS)) { sd_ok = true; sd_error_flag = false; } else { sd_ok = false; sd_error_flag = true; addToLog("SD: No Card", true); }
}
void writeLogToSD() {
  if (!g_sd_enable || !sd_ok) return;
  File f = SD.open("/log.csv", FILE_APPEND);
  if (f) { f.printf("%s;%.1f;%.0f;%.1f\n", getTimeStr().c_str(), victronData.avgSOC, victronData.totalPower, victronData.totalCurrent); f.close(); } else { sd_error_flag = true; }
}
void handleSDDownload() { if(!server.hasArg("file")) { server.send(400, "text/plain", "Bad Request"); return; } String path = server.arg("file"); if (!sd_ok || !SD.exists(path)) { server.send(404, "text/plain", "File Not Found"); return; } File f = SD.open(path); if (f) { server.streamFile(f, "application/octet-stream"); f.close(); } }
void handleSpyDownload() { if (!sd_ok) { server.send(404, "text/plain", "No SD"); return; } File f = SD.open("/spy.txt"); if (f) { server.streamFile(f, "text/plain"); f.close(); } else server.send(404, "text/plain", "No Spy Log"); }
void handleSDList() { if(!sd_ok) { server.send(200, "application/json", "[]"); return; } String out = "["; File root = SD.open("/"); File file = root.openNextFile(); bool first = true; while(file){ if(!first) out += ","; out += "{\"name\":\"" + String(file.name()) + "\",\"size\":" + String(file.size()) + "}"; first = false; file = root.openNextFile(); } out += "]"; server.send(200, "application/json", out); }
void handleSDDelete() { if(!sd_ok) { server.send(500, "text/plain", "No SD"); return; } if(server.hasArg("file")) { String f = server.arg("file"); if(SD.exists(f)) { SD.remove(f); server.send(200, "text/plain", "Deleted"); } else server.send(404, "text/plain", "Not Found"); } else if(server.hasArg("all")) { File root = SD.open("/"); File file = root.openNextFile(); while(file){ String n = file.name(); file = root.openNextFile(); SD.remove("/"+n); } server.send(200, "text/plain", "Wiped"); } else server.send(400, "text/plain", "Bad Args"); }
#else
void initSD() {}
void writeLogToSD() {}
#endif
// ==========================================
// MQTT & HOME ASSISTANT INTEGRATION
// MQTT publishes to {topic}/data every 5s with SOC, power, voltage, current,
// temperature, energy, alarm status, and optionally per-BMS cell data.
// Separate {topic}/alarm topic (retained) publishes on alarm state changes
// for HA automations. LWT: {topic}/status="offline" on unexpected disconnect.
// Reconnect uses exponential backoff: 5s -> 30s -> 60s.
// HA auto-discovery is re-sent on every MQTT connect to keep FW version
// and sensor config current after OTA updates.
// ==========================================
bool sendHaDiscovery() {
    if(!mqtt.connected()) return false;
    bool ok = true;
    String uid = g_dev_uid;
    String tp = "homeassistant/sensor/" + uid + "/";
    String btp = "homeassistant/binary_sensor/" + uid + "/";
    String dev = "\"dev\":{\"ids\":\"" + uid + "\",\"name\":\"Topband BMS (" + uid.substring(3) + ")\",\"mdl\":\"V" + String(FW_VERSION) + "\",\"mf\":\"DIY\"}";
    String stat_t = g_mqtt_topic + "/data";
    String av = "";
    String p1 = "{\"name\":\"SOC\",\"stat_t\":\"" + stat_t + "\",\"val_tpl\":\"{{value_json.soc}}\",\"unit_of_meas\":\"%\",\"dev_cla\":\"battery\",\"uniq_id\":\"" + uid + "_soc\"," + dev + "}";
    ok &= mqtt.publish((tp + "soc/config").c_str(), p1.c_str(), true);
    String p2 = "{\"name\":\"Power\",\"stat_t\":\"" + stat_t + "\",\"val_tpl\":\"{{value_json.p}}\",\"unit_of_meas\":\"W\",\"dev_cla\":\"power\",\"uniq_id\":\"" + uid + "_p\"," + dev + "}";
    ok &= mqtt.publish((tp + "p/config").c_str(), p2.c_str(), true);
    String p3 = "{\"name\":\"Voltage\",\"stat_t\":\"" + stat_t + "\",\"val_tpl\":\"{{value_json.v}}\",\"unit_of_meas\":\"V\",\"dev_cla\":\"voltage\",\"uniq_id\":\"" + uid + "_v\"," + dev + "}";
    ok &= mqtt.publish((tp + "v/config").c_str(), p3.c_str(), true);
    String p4 = "{\"name\":\"Current\",\"stat_t\":\"" + stat_t + "\",\"val_tpl\":\"{{value_json.i}}\",\"unit_of_meas\":\"A\",\"dev_cla\":\"current\",\"uniq_id\":\"" + uid + "_i\"," + dev + "}";
    ok &= mqtt.publish((tp + "i/config").c_str(), p4.c_str(), true);

    String p5 = "{\"name\":\"Active BMS\",\"stat_t\":\"" + stat_t + "\",\"val_tpl\":\"{{value_json.active}}\",\"unit_of_meas\":\"packs\",\"icon\":\"mdi:battery-multiple\",\"uniq_id\":\"" + uid + "_active\"," + dev + "}";
    ok &= mqtt.publish((tp + "active/config").c_str(), p5.c_str(), true);
    String p6 = "{\"name\":\"Max Charge Current\",\"stat_t\":\"" + stat_t + "\",\"val_tpl\":\"{{value_json.max_chg}}\",\"unit_of_meas\":\"A\",\"dev_cla\":\"current\",\"uniq_id\":\"" + uid + "_max_chg\"," + dev + "}";
    ok &= mqtt.publish((tp + "max_chg/config").c_str(), p6.c_str(), true);
    String p7 = "{\"name\":\"Max Discharge Current\",\"stat_t\":\"" + stat_t + "\",\"val_tpl\":\"{{value_json.max_dis}}\",\"unit_of_meas\":\"A\",\"dev_cla\":\"current\",\"uniq_id\":\"" + uid + "_max_dis\"," + dev + "}";
    ok &= mqtt.publish((tp + "max_dis/config").c_str(), p7.c_str(), true);
    String p8 = "{\"name\":\"Alarm\",\"stat_t\":\"" + stat_t + "\",\"val_tpl\":\"{{value_json.alarm}}\",\"icon\":\"mdi:alert\",\"uniq_id\":\"" + uid + "_alarm\"," + dev + "}";
    ok &= mqtt.publish((tp + "alarm/config").c_str(), p8.c_str(), true);
    String p9 = "{\"name\":\"Avg Temperature\",\"stat_t\":\"" + stat_t + "\",\"val_tpl\":\"{{value_json.avg_temp}}\",\"unit_of_meas\":\"\u00b0C\",\"dev_cla\":\"temperature\",\"uniq_id\":\"" + uid + "_avg_temp\"," + dev + "}";
    ok &= mqtt.publish((tp + "avg_temp/config").c_str(), p9.c_str(), true);
    String p10 = "{\"name\":\"SOH\",\"stat_t\":\"" + stat_t + "\",\"val_tpl\":\"{{value_json.soh}}\",\"unit_of_meas\":\"%\",\"icon\":\"mdi:battery-heart-variant\",\"uniq_id\":\"" + uid + "_soh\"," + dev + "}";
    ok &= mqtt.publish((tp + "soh/config").c_str(), p10.c_str(), true);
    String p11 = "{\"name\":\"Remaining Capacity\",\"stat_t\":\"" + stat_t + "\",\"val_tpl\":\"{{value_json.rem_cap}}\",\"unit_of_meas\":\"Ah\",\"icon\":\"mdi:battery-charging\",\"uniq_id\":\"" + uid + "_rem_cap\"," + dev + "}";
    ok &= mqtt.publish((tp + "rem_cap/config").c_str(), p11.c_str(), true);
    String p12 = "{\"name\":\"Total Capacity\",\"stat_t\":\"" + stat_t + "\",\"val_tpl\":\"{{value_json.total_cap}}\",\"unit_of_meas\":\"Ah\",\"icon\":\"mdi:battery\",\"uniq_id\":\"" + uid + "_total_cap\"," + dev + "}";
    ok &= mqtt.publish((tp + "total_cap/config").c_str(), p12.c_str(), true);
    String p13 = "{\"name\":\"Energy In Today\",\"stat_t\":\"" + stat_t + "\",\"val_tpl\":\"{{value_json.energy_in}}\",\"unit_of_meas\":\"kWh\",\"dev_cla\":\"energy\",\"uniq_id\":\"" + uid + "_energy_in\"," + dev + "}";
    ok &= mqtt.publish((tp + "energy_in/config").c_str(), p13.c_str(), true);
    String p14 = "{\"name\":\"Energy Out Today\",\"stat_t\":\"" + stat_t + "\",\"val_tpl\":\"{{value_json.energy_out}}\",\"unit_of_meas\":\"kWh\",\"dev_cla\":\"energy\",\"uniq_id\":\"" + uid + "_energy_out\"," + dev + "}";
    ok &= mqtt.publish((tp + "energy_out/config").c_str(), p14.c_str(), true);

    for (int i = 0; i < MAX_BMS; i++) {
      String idx = String(i);
      String t_soc = tp + "bms" + idx + "_soc/config";
      String t_v = tp + "bms" + idx + "_v/config";
      String t_i = tp + "bms" + idx + "_i/config";
      String t_min = tp + "bms" + idx + "_min_cell/config";
      String t_max = tp + "bms" + idx + "_max_cell/config";
      String t_on = btp + "bms" + idx + "_online/config";

      if (g_mqtt_full && i < g_bms_count) {
        String s_soc = "{\"name\":\"BMS " + idx + " SOC\",\"stat_t\":\"" + stat_t + "\",\"val_tpl\":\"{{value_json.bms[" + idx + "].soc | default(none)}}\",\"unit_of_meas\":\"%\",\"dev_cla\":\"battery\",\"uniq_id\":\"" + uid + "_bms" + idx + "_soc\"," + dev + "}";
        String s_v = "{\"name\":\"BMS " + idx + " Voltage\",\"stat_t\":\"" + stat_t + "\",\"val_tpl\":\"{{value_json.bms[" + idx + "].pack_v | default(none)}}\",\"unit_of_meas\":\"V\",\"dev_cla\":\"voltage\",\"uniq_id\":\"" + uid + "_bms" + idx + "_v\"," + dev + "}";
        String s_i = "{\"name\":\"BMS " + idx + " Current\",\"stat_t\":\"" + stat_t + "\",\"val_tpl\":\"{{value_json.bms[" + idx + "].current | default(none)}}\",\"unit_of_meas\":\"A\",\"dev_cla\":\"current\",\"uniq_id\":\"" + uid + "_bms" + idx + "_i\"," + dev + "}";
        String s_min = "{\"name\":\"BMS " + idx + " Min Cell\",\"stat_t\":\"" + stat_t + "\",\"val_tpl\":\"{{value_json.bms[" + idx + "].min_cell | default(none)}}\",\"unit_of_meas\":\"V\",\"dev_cla\":\"voltage\",\"uniq_id\":\"" + uid + "_bms" + idx + "_min_cell\"," + dev + "}";
        String s_max = "{\"name\":\"BMS " + idx + " Max Cell\",\"stat_t\":\"" + stat_t + "\",\"val_tpl\":\"{{value_json.bms[" + idx + "].max_cell | default(none)}}\",\"unit_of_meas\":\"V\",\"dev_cla\":\"voltage\",\"uniq_id\":\"" + uid + "_bms" + idx + "_max_cell\"," + dev + "}";
        String s_on = "{\"name\":\"BMS " + idx + " Online\",\"stat_t\":\"" + stat_t + "\",\"val_tpl\":\"{{ 'ON' if (value_json.bms[" + idx + "].online | default(false)) else 'OFF' }}\",\"pl_on\":\"ON\",\"pl_off\":\"OFF\",\"dev_cla\":\"connectivity\",\"uniq_id\":\"" + uid + "_bms" + idx + "_online\"," + dev + "}";
        ok &= mqtt.publish(t_soc.c_str(), s_soc.c_str(), true);
        ok &= mqtt.publish(t_v.c_str(), s_v.c_str(), true);
        ok &= mqtt.publish(t_i.c_str(), s_i.c_str(), true);
        ok &= mqtt.publish(t_min.c_str(), s_min.c_str(), true);
        ok &= mqtt.publish(t_max.c_str(), s_max.c_str(), true);
        ok &= mqtt.publish(t_on.c_str(), s_on.c_str(), true);
      } else {
        ok &= mqtt.publish(t_soc.c_str(), "", true);
        ok &= mqtt.publish(t_v.c_str(), "", true);
        ok &= mqtt.publish(t_i.c_str(), "", true);
        ok &= mqtt.publish(t_min.c_str(), "", true);
        ok &= mqtt.publish(t_max.c_str(), "", true);
        ok &= mqtt.publish(t_on.c_str(), "", true);
      }
    }
    return ok;
}

bool clearHaDiscovery() {
    if(!mqtt.connected()) return false;
    bool ok = true;
    String tp = "homeassistant/sensor/" + g_dev_uid + "/";
    String btp = "homeassistant/binary_sensor/" + g_dev_uid + "/";
    ok &= mqtt.publish((tp + "soc/config").c_str(), "", true);
    ok &= mqtt.publish((tp + "p/config").c_str(), "", true);
    ok &= mqtt.publish((tp + "v/config").c_str(), "", true);
    ok &= mqtt.publish((tp + "i/config").c_str(), "", true);
    ok &= mqtt.publish((tp + "active/config").c_str(), "", true);
    ok &= mqtt.publish((tp + "max_chg/config").c_str(), "", true);
    ok &= mqtt.publish((tp + "max_dis/config").c_str(), "", true);
    ok &= mqtt.publish((tp + "alarm/config").c_str(), "", true);
    ok &= mqtt.publish((tp + "avg_temp/config").c_str(), "", true);
    ok &= mqtt.publish((tp + "soh/config").c_str(), "", true);
    ok &= mqtt.publish((tp + "rem_cap/config").c_str(), "", true);
    ok &= mqtt.publish((tp + "total_cap/config").c_str(), "", true);
    ok &= mqtt.publish((tp + "energy_in/config").c_str(), "", true);
    ok &= mqtt.publish((tp + "energy_out/config").c_str(), "", true);
    for (int i = 0; i < MAX_BMS; i++) {
      String idx = String(i);
      ok &= mqtt.publish((tp + "bms" + idx + "_soc/config").c_str(), "", true);
      ok &= mqtt.publish((tp + "bms" + idx + "_v/config").c_str(), "", true);
      ok &= mqtt.publish((tp + "bms" + idx + "_i/config").c_str(), "", true);
      ok &= mqtt.publish((tp + "bms" + idx + "_min_cell/config").c_str(), "", true);
      ok &= mqtt.publish((tp + "bms" + idx + "_max_cell/config").c_str(), "", true);
      ok &= mqtt.publish((btp + "bms" + idx + "_online/config").c_str(), "", true);
    }
    return ok;
}

void handleHaDiscoverySend() {
  if (!checkAuth()) return;
  if (!g_mqtt_enable || g_mqtt_server == "") { server.send(400, "application/json", "{\"ok\":false,\"err\":\"mqtt disabled\"}"); return; }
  mqttReconnect();
  if (!mqtt.connected()) { server.send(500, "application/json", "{\"ok\":false,\"err\":\"mqtt not connected\"}"); return; }
  if (!sendHaDiscovery()) {
    addToLog("HA Discovery: publish failed", true);
    server.send(500, "application/json", "{\"ok\":false,\"err\":\"mqtt publish failed\"}");
    return;
  }
  addToLog("HA Discovery: sent", false);
  server.send(200, "application/json", "{\"ok\":true}");
}

void handleHaDiscoveryClear() {
  if (!checkAuth()) return;
  if (!g_mqtt_enable || g_mqtt_server == "") { server.send(400, "application/json", "{\"ok\":false,\"err\":\"mqtt disabled\"}"); return; }
  mqttReconnect();
  if (!mqtt.connected()) { server.send(500, "application/json", "{\"ok\":false,\"err\":\"mqtt not connected\"}"); return; }
  if (!clearHaDiscovery()) {
    addToLog("HA Discovery: clear publish failed", true);
    server.send(500, "application/json", "{\"ok\":false,\"err\":\"mqtt publish failed\"}");
    return;
  }
  addToLog("HA Discovery: cleared", false);
  server.send(200, "application/json", "{\"ok\":true}");
}

void mqttReconnect() {
  if (!g_mqtt_enable || g_mqtt_server == "") return;
  if (mqtt.connected()) { mqtt_fail_count = 0; mqtt_reconnect_interval = 5000; return; }
  if (WiFi.status() != WL_CONNECTED) return;
  unsigned long now = millis();
  if (now - last_mqtt_reconnect_attempt < mqtt_reconnect_interval) return;
  last_mqtt_reconnect_attempt = now;
  esp_task_wdt_reset();
  addToLog("MQTT: connecting to " + g_mqtt_server + ":" + String(g_mqtt_port) + " (attempt " + String(mqtt_fail_count + 1) + ")", false);
  String willTopic = g_mqtt_topic + "/status";
  if (mqtt.connect(g_dev_uid.c_str(), g_mqtt_user.c_str(), g_mqtt_pass.c_str(), willTopic.c_str(), 0, true, "offline")) {
    mqtt.publish((g_mqtt_topic + "/status").c_str(), "online", true);
    mqtt_fail_count = 0;
    mqtt_reconnect_interval = 5000;
    addToLog("MQTT: connected", false);
    // Auto-send HA discovery on every connect (updates FW version in HA)
    if (g_ha_enable) {
      if (sendHaDiscovery()) addToLog("HA: discovery sent (V" + String(FW_VERSION) + ")", false);
      else addToLog("HA: discovery send failed", true);
    }
  } else {
    mqtt_fail_count++;
    mqtt_reconnect_interval = mqtt_fail_count < 3 ? 5000 : mqtt_fail_count < 10 ? 30000 : 60000;
    addToLog("MQTT: connect failed (rc=" + String(mqtt.state()) + "), next in " + String(mqtt_reconnect_interval / 1000) + "s", true);
  }
  esp_task_wdt_reset();
}


// Build MQTT JSON under dataMutex, then release before network I/O
void sendMqttData() {
  if (g_mqtt_enable && mqtt.connected()) {
    // Snapshot data under mutex, then release before network I/O
    String json; json.reserve(g_mqtt_full ? 2048 : 512);
    String statusTopic = g_mqtt_topic + "/status";
    String dataTopic = g_mqtt_topic + "/data";
    if (dataMutex) xSemaphoreTake(dataMutex, pdMS_TO_TICKS(200));
    json = "{"; json += "\"soc\":" + String(victronData.avgSOC, 1) + ","; json += "\"p\":" + String(victronData.totalPower, 0) + ",";
    json += "\"i\":" + String(victronData.totalCurrent, 1) + ","; json += "\"v\":" + String(victronData.avgVoltage, 2) + ",";
    json += "\"alarm\":\"" + sys_error_msg + "\","; json += "\"active\":" + String(victronData.activePacks);
    json += ",\"avg_temp\":" + String(victronData.avgTemp, 1);
    json += ",\"soh\":" + String(victronData.avgSOH, 1);
    json += ",\"rem_cap\":" + String(victronData.remainCapacity, 0);
    json += ",\"total_cap\":" + String(victronData.totalCapacity, 0);
    json += ",\"energy_in\":" + String(energy_in_today, 2);
    json += ",\"energy_out\":" + String(energy_out_today, 2);
    json += ",\"max_chg\":" + String(victronData.maxChargeCurrent, 1) + ",\"max_dis\":" + String(victronData.maxDischargeCurrent, 1);
    json += ",\"fw\":\"" + String(FW_VERSION) + "\",\"uid\":\"" + g_dev_uid + "\",\"uptime\":" + String(millis() / 1000);
    if (g_mqtt_full) {
      json += ",\"bms\":[";
      for (int i = 0; i < g_bms_count; i++) {
        if (i > 0) json += ",";
        bool on = bms[i].valid && (millis() - bms[i].last_seen < 120000);
        json += "{\"id\":" + String(i) + ",\"online\":" + String(on ? "true" : "false");
        if (on) {
          json += ",\"soc\":" + String(bms[i].soc);
          json += ",\"current\":" + String(bms[i].current, 1);
          json += ",\"pack_v\":" + String(bms[i].voltage, 2);
          json += ",\"soh\":" + String(bms[i].soh);
          json += ",\"min_cell\":" + String(bms[i].minCellV, 3);
          json += ",\"max_cell\":" + String(bms[i].maxCellV, 3);
          json += ",\"cells\":[";
          for (int c = 0; c < bms[i].cell_count; c++) {
            if (c > 0) json += ",";
            json += String(bms[i].cells[c], 3);
          }
          json += "]";
        }
        json += "}";
      }
      json += "]";
    }
    json += "}";
    if (dataMutex) xSemaphoreGive(dataMutex);
    // Publish outside mutex - network I/O can take 100ms+
    mqtt.publish(statusTopic.c_str(), "online", true);
    mqtt.publish(dataTopic.c_str(), json.c_str());
    // Publish alarm state on separate topic (retained, for HA automations)
    static String prev_alarm_mqtt = "OK";
    String alarm_snap = sys_error_msg;
    if (alarm_snap != prev_alarm_mqtt) {
      String alarmTopic = g_mqtt_topic + "/alarm";
      String alarmJson = "{\"state\":\"" + alarm_snap + "\",\"flags\":" + String(alarm_flags) + ",\"safe_lock\":" + String(((alarm_flags & 0x02)||(alarm_flags & 0x10)||(alarm_flags & 0x40)||(alarm_flags & 0x80)) ? "true" : "false") + "}";
      mqtt.publish(alarmTopic.c_str(), alarmJson.c_str(), true);
      prev_alarm_mqtt = alarm_snap;
    }
  }
}

bool svcParseBmsId(int &id) {
  if (!server.hasArg("id")) return false;
  id = server.arg("id").toInt();
  if (id < 0 || id >= g_bms_count || id >= MAX_BMS) return false;
  return true;
}

float svcArgFloat(const char *name, float def) {
  if (!server.hasArg(name)) return def;
  return server.arg(name).toFloat();
}

void handleServiceBmsTimeGet() {
  if (!checkAuth()) return;
  int id;
  if (!svcParseBmsId(id)) { server.send(400, "application/json", "{\"ok\":false,\"err\":\"bad id\"}"); return; }

  TBFrame fr;
  String err;
  if (!tbRequestFrame((uint8_t)id, 0x4D, nullptr, 0, fr, err, true)) {
    server.send(500, "application/json", "{\"ok\":false,\"err\":\"" + jsonEscape(err) + "\"}");
    return;
  }
  if (fr.cid2 != 0x00) {
    server.send(500, "application/json", "{\"ok\":false,\"err\":\"BMS code 0x" + String(fr.cid2, HEX) + "\"}");
    return;
  }
  TBDate d;
  if (!tbParseDateFromFrame(fr, d)) {
    server.send(500, "application/json", "{\"ok\":false,\"err\":\"date parse failed\"}");
    return;
  }
  String j = "{";
  j += "\"ok\":true,";
  j += "\"id\":" + String(id) + ",";
  j += "\"date\":{\"year\":" + String(d.year) + ",\"month\":" + String(d.month) + ",\"day\":" + String(d.day) + ",\"hour\":" + String(d.hour) + ",\"minute\":" + String(d.minute) + ",\"second\":" + String(d.second) + "}";
  j += "}";
  server.send(200, "application/json", j);
}

void handleServiceBmsTimeSet() {
  if (!checkAuth()) return;
  int id;
  if (!svcParseBmsId(id)) { server.send(400, "application/json", "{\"ok\":false,\"err\":\"bad id\"}"); return; }

  TBDate d = {};
  if (server.hasArg("year")) {
    d.year = server.arg("year").toInt();
    d.month = server.arg("month").toInt();
    d.day = server.arg("day").toInt();
    d.hour = server.arg("hour").toInt();
    d.minute = server.arg("minute").toInt();
    d.second = server.arg("second").toInt();
  } else {
    struct tm t;
    if (!getLocalTime(&t)) {
      server.send(500, "application/json", "{\"ok\":false,\"err\":\"gateway time unavailable\"}");
      return;
    }
    d.year = t.tm_year + 1900;
    d.month = t.tm_mon + 1;
    d.day = t.tm_mday;
    d.hour = t.tm_hour;
    d.minute = t.tm_min;
    d.second = t.tm_sec;
  }

  uint8_t info7[7] = {
    (uint8_t)((d.year >> 8) & 0xFF), (uint8_t)(d.year & 0xFF),
    (uint8_t)d.month, (uint8_t)d.day, (uint8_t)d.hour, (uint8_t)d.minute, (uint8_t)d.second
  };

  TBFrame fr;
  String err;
  bool ok = tbRequestFrame((uint8_t)id, 0x4E, info7, 7, fr, err, true);
  if ((!ok || fr.cid2 != 0x00)) {
    // Fallback for variants requiring BMS id in payload.
    uint8_t info8[8] = {
      (uint8_t)id,
      (uint8_t)((d.year >> 8) & 0xFF), (uint8_t)(d.year & 0xFF),
      (uint8_t)d.month, (uint8_t)d.day, (uint8_t)d.hour, (uint8_t)d.minute, (uint8_t)d.second
    };
    ok = tbRequestFrame((uint8_t)id, 0x4E, info8, 8, fr, err, true);
  }

  if (!ok) {
    server.send(500, "application/json", "{\"ok\":false,\"err\":\"" + jsonEscape(err) + "\"}");
    return;
  }
  if (fr.cid2 != 0x00) {
    server.send(500, "application/json", "{\"ok\":false,\"err\":\"BMS code 0x" + String(fr.cid2, HEX) + "\"}");
    return;
  }

  String j = "{";
  j += "\"ok\":true,";
  j += "\"id\":" + String(id) + ",";
  j += "\"set\":{\"year\":" + String(d.year) + ",\"month\":" + String(d.month) + ",\"day\":" + String(d.day) + ",\"hour\":" + String(d.hour) + ",\"minute\":" + String(d.minute) + ",\"second\":" + String(d.second) + "}";
  j += "}";
  server.send(200, "application/json", j);
}

void handleServiceAutoCfgApply() {
  if (!checkAuth()) return;
  int id;
  if (!svcParseBmsId(id)) { server.send(400, "application/json", "{\"ok\":false,\"err\":\"bad id\"}"); return; }
  float buf_cell_mv = svcArgFloat("buf_cell_mv", 20.0f);
  float buf_pack_v = svcArgFloat("buf_pack_v", 0.30f);
  float buf_temp_c = svcArgFloat("buf_temp_c", 2.0f);
  float buf_curr_pct = svcArgFloat("buf_curr_pct", 10.0f);

  TBFrame fr;
  String err;
  if (!tbRequestFrame((uint8_t)id, 0x47, nullptr, 0, fr, err, true) || fr.cid2 != 0x00) {
    server.send(500, "application/json", "{\"ok\":false,\"err\":\"system parameter read failed\"}");
    return;
  }

  TBSysParam sp;
  if (!tbParseSystemParameterFromFrame(fr, sp)) {
    server.send(500, "application/json", "{\"ok\":false,\"err\":\"system parameter parse failed\"}");
    return;
  }
  if (!tbSystemParamPlausible(sp)) {
    server.send(500, "application/json", "{\"ok\":false,\"err\":\"system parameter out of plausible range\"}");
    return;
  }

  float safe_cell, safe_volt, tc_min, tc_max, td_min, td_max, chg_a, dis_a, cvl_v;
  tbBuildAutoCfgSuggestion(sp, buf_cell_mv, buf_pack_v, buf_temp_c, buf_curr_pct,
                           safe_cell, safe_volt, tc_min, tc_max, td_min, td_max, chg_a, dis_a, cvl_v);

  g_expert_mode = true;
  g_safe_cell = safe_cell;
  g_safe_volt = safe_volt;
  g_tc_min = tc_min;
  g_tc_max = tc_max;
  g_td_min = td_min;
  g_td_max = td_max;
  g_charge_amps = chg_a;
  g_discharge_amps = dis_a;
  g_cvl_voltage = cvl_v;

  String j = "{";
  j += "\"ok\":true,";
  j += "\"id\":" + String(id) + ",";
  j += "\"applied\":{";
  j += "\"s_cel\":" + String(g_safe_cell, 3) + ",";
  j += "\"s_vol\":" + String(g_safe_volt, 2) + ",";
  j += "\"t_c_min\":" + String(g_tc_min, 1) + ",";
  j += "\"t_c_max\":" + String(g_tc_max, 1) + ",";
  j += "\"t_d_min\":" + String(g_td_min, 1) + ",";
  j += "\"t_d_max\":" + String(g_td_max, 1) + ",";
  j += "\"chg\":" + String(g_charge_amps, 1) + ",";
  j += "\"dis\":" + String(g_discharge_amps, 1) + ",";
  j += "\"cvl\":" + String(g_cvl_voltage, 2);
  j += "}}";
  server.send(200, "application/json", j);
}

void handleServiceBmsDiag() {
  if (!checkAuth()) return;
  int id;
  if (!svcParseBmsId(id)) { server.send(400, "application/json", "{\"ok\":false,\"err\":\"bad id\"}"); return; }

  float buf_cell_mv = svcArgFloat("buf_cell_mv", 20.0f);
  float buf_pack_v = svcArgFloat("buf_pack_v", 0.30f);
  float buf_temp_c = svcArgFloat("buf_temp_c", 2.0f);
  float buf_curr_pct = svcArgFloat("buf_curr_pct", 10.0f);

  TBFrame fr_manu, fr_date, fr_alarm, fr_sys, fr_hist, fr_aval;
  String err_manu, err_date, err_alarm, err_sys, err_hist, err_aval;

  bool manu_ok = tbRequestFrame((uint8_t)id, 0x51, nullptr, 0, fr_manu, err_manu, true) && fr_manu.cid2 == 0x00;
  bool date_ok = tbRequestFrame((uint8_t)id, 0x4D, nullptr, 0, fr_date, err_date, true) && fr_date.cid2 == 0x00;
  uint8_t alarm_req[1] = {(uint8_t)id};
  bool alarm_ok = tbRequestFrame((uint8_t)id, 0x44, alarm_req, 1, fr_alarm, err_alarm, true) && fr_alarm.cid2 == 0x00;
  bool sys_ok = tbRequestFrame((uint8_t)id, 0x47, nullptr, 0, fr_sys, err_sys, true) && fr_sys.cid2 == 0x00;
  uint8_t hist_req[2] = {0x00, (uint8_t)id};  // first event
  bool hist_ok = tbRequestFrame((uint8_t)id, 0x4B, hist_req, 2, fr_hist, err_hist, true) && fr_hist.cid2 == 0x00;
  uint8_t aval_req[1] = {(uint8_t)id};
  bool aval_ok = tbRequestFrame((uint8_t)id, 0x42, aval_req, 1, fr_aval, err_aval, true) && fr_aval.cid2 == 0x00;

  String hw = "", sw = "", mid = "";
  TBDate bms_date = {};
  TBAlarmInfoParsed ai = {};
  TBSysParam sp = {};
  TBDate hist_date = {};
  int hist_event = -1;
  uint64_t hist_status = 0;
  float hist_i = 0, hist_v = 0, hist_rem = 0;
  TBAnalogDiag ad = {};

  if (manu_ok) manu_ok = tbParseManufacturerFromFrame(fr_manu, hw, sw, mid);
  if (date_ok) date_ok = tbParseDateFromFrame(fr_date, bms_date);
  if (alarm_ok) {
    alarm_ok = tbParseAlarmInfoFromFrame(fr_alarm, ai);
    if (alarm_ok) {
      g_bms_alarm_bits[id] = ai.status_bits;
      g_bms_alarm_status_count[id] = ai.status_count;
      g_bms_alarm_last_seen[id] = millis();
    }
  }
  if (sys_ok) {
    sys_ok = tbParseSystemParameterFromFrame(fr_sys, sp);
    if (sys_ok && !tbSystemParamPlausible(sp)) { sys_ok = false; err_sys = "system parameter out of plausible range"; }
  }
  if (hist_ok) hist_ok = tbParseHistoricalFirstFromFrame(fr_hist, hist_date, hist_event, hist_status, hist_i, hist_v, hist_rem);
  if (aval_ok) aval_ok = tbParseAnalogDiagFromFrame(fr_aval, ad);

  float sug_cell = 0, sug_volt = 0, sug_tcmin = 0, sug_tcmax = 0, sug_tdmin = 0, sug_tdmax = 0, sug_chg = 0, sug_dis = 0, sug_cvl = 0;
  if (sys_ok) {
    tbBuildAutoCfgSuggestion(sp, buf_cell_mv, buf_pack_v, buf_temp_c, buf_curr_pct,
                             sug_cell, sug_volt, sug_tcmin, sug_tcmax, sug_tdmin, sug_tdmax, sug_chg, sug_dis, sug_cvl);
  }

  String j = "{";
  j += "\"ok\":true,";
  j += "\"id\":" + String(id) + ",";

  j += "\"manufacturer\":{\"ok\":" + String(manu_ok ? "true" : "false");
  if (manu_ok) {
    j += ",\"hw\":\"" + jsonEscape(hw) + "\",\"sw\":\"" + jsonEscape(sw) + "\",\"id\":\"" + jsonEscape(mid) + "\"";
  } else {
    j += ",\"err\":\"" + jsonEscape(err_manu) + "\"";
  }
  j += "},";

  j += "\"date\":{\"ok\":" + String(date_ok ? "true" : "false");
  if (date_ok) {
    j += ",\"year\":" + String(bms_date.year) + ",\"month\":" + String(bms_date.month) + ",\"day\":" + String(bms_date.day) + ",\"hour\":" + String(bms_date.hour) + ",\"minute\":" + String(bms_date.minute) + ",\"second\":" + String(bms_date.second);
  } else {
    j += ",\"err\":\"" + jsonEscape(err_date) + "\"";
  }
  j += "},";

  j += "\"analog\":{\"ok\":" + String(aval_ok ? "true" : "false");
  if (aval_ok) {
    j += ",\"id\":" + String(ad.id);
    j += ",\"cells\":" + String(ad.cell_count);
    j += ",\"temps\":" + String(ad.temp_count);
    j += ",\"pack_v\":" + String(ad.pack_v, 2);
    j += ",\"current\":" + String(ad.current_a, 2);
    j += ",\"soc\":" + String(ad.soc);
    j += ",\"soh\":" + String(ad.soh);
    j += ",\"min_cell\":" + String(ad.min_cell_v, 3);
    j += ",\"max_cell\":" + String(ad.max_cell_v, 3);
    j += ",\"rem_ah\":" + String(ad.rem_ah, 2);
    j += ",\"full_ah\":" + String(ad.full_ah, 2);
  } else {
    j += ",\"err\":\"" + jsonEscape(err_aval) + "\"";
  }
  j += "},";

  j += "\"alarm\":{\"ok\":" + String(alarm_ok ? "true" : "false");
  if (alarm_ok) {
    j += ",\"status_count\":" + String(ai.status_count);
    j += ",\"status_hex\":\"" + u64Hex(ai.status_bits) + "\"";
    j += ",\"critical\":" + String(tbHasCriticalAlarm(ai.status_bits) ? "true" : "false");
    j += ",\"active\":[";
    bool first = true;
    for (uint8_t b = 0; b < 64; b++) {
      if (ai.status_bits & (1ULL << b)) {
        if (!first) j += ",";
        j += "{\"bit\":" + String(b) + ",\"name\":\"" + jsonEscape(String(tbAlarmBitName(b))) + "\"}";
        first = false;
      }
    }
    j += "]";
  } else {
    j += ",\"err\":\"" + jsonEscape(err_alarm) + "\"";
  }
  j += "},";

  j += "\"system\":{\"ok\":" + String(sys_ok ? "true" : "false");
  if (sys_ok) {
    j += ",\"cell_high_v\":" + String(sp.cell_high_v, 3);
    j += ",\"cell_low_v\":" + String(sp.cell_low_v, 3);
    j += ",\"cell_under_v\":" + String(sp.cell_under_v, 3);
    j += ",\"module_high_v\":" + String(sp.module_high_v, 2);
    j += ",\"module_low_v\":" + String(sp.module_low_v, 2);
    j += ",\"module_under_v\":" + String(sp.module_under_v, 2);
    j += ",\"charge_high_t\":" + String(sp.charge_high_t, 1);
    j += ",\"charge_low_t\":" + String(sp.charge_low_t, 1);
    j += ",\"discharge_high_t\":" + String(sp.discharge_high_t, 1);
    j += ",\"discharge_low_t\":" + String(sp.discharge_low_t, 1);
    j += ",\"charge_current_max_a\":" + String(sp.charge_current_max_a, 1);
    j += ",\"discharge_current_max_a\":" + String(sp.discharge_current_max_a, 1);
  } else {
    j += ",\"err\":\"" + jsonEscape(err_sys) + "\"";
  }
  j += "},";

  j += "\"historical\":{\"ok\":" + String(hist_ok ? "true" : "false");
  if (hist_ok) {
    j += ",\"year\":" + String(hist_date.year) + ",\"month\":" + String(hist_date.month) + ",\"day\":" + String(hist_date.day) + ",\"hour\":" + String(hist_date.hour) + ",\"minute\":" + String(hist_date.minute) + ",\"second\":" + String(hist_date.second);
    j += ",\"event\":" + String(hist_event);
    j += ",\"status_hex\":\"" + u64Hex(hist_status) + "\"";
    j += ",\"pack_v\":" + String(hist_v, 2);
    j += ",\"current\":" + String(hist_i, 2);
    j += ",\"remain_ah\":" + String(hist_rem, 2);
  } else {
    j += ",\"err\":\"" + jsonEscape(err_hist) + "\"";
  }
  j += "},";

  j += "\"autocfg\":{\"ok\":" + String(sys_ok ? "true" : "false");
  if (sys_ok) {
    j += ",\"buffer\":{\"cell_mv\":" + String(buf_cell_mv, 1) + ",\"pack_v\":" + String(buf_pack_v, 2) + ",\"temp_c\":" + String(buf_temp_c, 1) + ",\"curr_pct\":" + String(buf_curr_pct, 1) + "}";
    j += ",\"suggest\":{\"s_cel\":" + String(sug_cell, 3) + ",\"s_vol\":" + String(sug_volt, 2) + ",\"t_c_min\":" + String(sug_tcmin, 1) + ",\"t_c_max\":" + String(sug_tcmax, 1) + ",\"t_d_min\":" + String(sug_tdmin, 1) + ",\"t_d_max\":" + String(sug_tdmax, 1) + ",\"chg\":" + String(sug_chg, 1) + ",\"dis\":" + String(sug_dis, 1) + ",\"cvl\":" + String(sug_cvl, 2) + "}";
  }
  j += "}";

  j += "}";
  server.send(200, "application/json", j);
}

// ==========================================
// WEB ENDPOINT HANDLERS
// All handlers call checkAuth() first (cookie session check, returns 401/302
// if not authenticated). Settings are saved to NVS
// and the device reboots. handleData() uses chunked HTTP streaming
// with a mutex snapshot pattern to avoid blocking Core 0.
// ==========================================
void handleSim() { if (!checkAuth()) return; if (server.hasArg("act")) simulation_active = (server.arg("act") == "1"); server.send(200); }
void handleChartCfg() {
  if (!checkAuth(true)) return;
  if (server.hasArg("c1")) { g_chart1 = constrain(server.arg("c1").toInt(), 0, 3); }
  if (server.hasArg("c2")) { g_chart2 = constrain(server.arg("c2").toInt(), 0, 3); }
  preferences.begin("gateway", false);
  preferences.putInt("chart1", g_chart1); preferences.putInt("chart2", g_chart2);
  preferences.end();
  server.send(200, "application/json", "{\"ok\":true}");
}

// V2.61: GET /alerts -> JSON array of last 25 alerts (newest first)
void handleAlertsGet() {
  if (!checkAuth()) return;
  String j = "["; bool first = true;
  // Walk back from head (newest last written) to oldest still in buffer
  for (int i = 1; i <= g_alert_count; i++) {
    int idx = (g_alert_head - i + ALERT_RING_SIZE) % ALERT_RING_SIZE;
    if (!first) j += ",";
    first = false;
    // Escape message for JSON (backslash, quote, control chars)
    String m; m.reserve(ALERT_MSG_LEN + 16);
    const char* p = g_alerts[idx].msg;
    for (int k = 0; k < ALERT_MSG_LEN && p[k]; k++) {
      char ch = p[k];
      if (ch == '"' || ch == '\\') { m += '\\'; m += ch; }
      else if (ch == '\n') m += " ";
      else if ((uint8_t)ch < 0x20) m += ' ';
      else m += ch;
    }
    j += "{\"ts\":" + String(g_alerts[idx].ts);
    j += ",\"up\":" + String(g_alerts[idx].uptime_s);
    j += ",\"sev\":" + String((int)g_alerts[idx].sev);
    j += ",\"msg\":\"" + m + "\"}";
  }
  j += "]";
  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "application/json", j);
}

// V2.61: POST/GET /alerts/clear -> wipe RAM + NVS
void handleAlertsClear() {
  if (!checkAuth()) return;
  clearAlertsAll();
  server.send(200, "application/json", "{\"ok\":true}");
}

void handleSave() {
  if (!checkAuth()) return;
  saveHistory(true); delay(100);
  if (g_alerts_dirty) saveAlertsToNvs();   // V2.61: flush alerts before reboot
  preferences.begin("gateway", false); 
  if (server.hasArg("cnt")) g_bms_count = server.arg("cnt").toInt();
  if (server.hasArg("cells")) g_force_cell_count = server.arg("cells").toInt();
  if (server.hasArg("soc_mode")) g_soc_source_mode = server.arg("soc_mode").toInt();
  if (server.hasArg("chart1")) g_chart1 = constrain(server.arg("chart1").toInt(), 0, 3);
  if (server.hasArg("chart2")) g_chart2 = constrain(server.arg("chart2").toInt(), 0, 3);
  if (g_soc_source_mode < 0 || g_soc_source_mode > 2) g_soc_source_mode = 2;
  g_maint_charge_mode = server.hasArg("maint_en");
  if (server.hasArg("maint_v")) g_maint_target_v = server.arg("maint_v").toFloat();
  if (g_maint_target_v < 40.0f) g_maint_target_v = 40.0f;
  if (g_maint_target_v > 65.0f) g_maint_target_v = 65.0f;
  g_auto_balance_enable = server.hasArg("ab_en");
  g_easy_mode = server.hasArg("easy");
  g_expert_mode = server.hasArg("exp");
  if (g_easy_mode) g_expert_mode = false;
  g_setup_done = true;
  if(g_expert_mode) {
      if(server.hasArg("chg")) g_charge_amps = server.arg("chg").toFloat();
      if(server.hasArg("dis")) g_discharge_amps = server.arg("dis").toFloat();
      if(server.hasArg("cvl")) g_cvl_voltage = server.arg("cvl").toFloat();
      if(server.hasArg("s_vol")) g_safe_volt = server.arg("s_vol").toFloat();
      if(server.hasArg("s_cel")) g_safe_cell = server.arg("s_cel").toFloat();
      if(server.hasArg("t_c_min")) g_tc_min = server.arg("t_c_min").toFloat();
      if(server.hasArg("t_c_max")) g_tc_max = server.arg("t_c_max").toFloat();
      if(server.hasArg("t_d_min")) g_td_min = server.arg("t_d_min").toFloat();
      if(server.hasArg("t_d_max")) g_td_max = server.arg("t_d_max").toFloat();
      if(server.hasArg("t_mode")) g_temp_mode = server.arg("t_mode").toInt();
      if(server.hasArg("s_drift")) g_safe_drift = server.arg("s_drift").toFloat();
  }  // Defaults when expert mode is off
  if (!g_expert_mode) {
    g_easy_mode = false;
    g_charge_amps = 30.0f;
    g_discharge_amps = 30.0f;
    g_cvl_voltage = 52.5f;
    g_safe_volt = 53.25f;
    g_safe_cell = 3.55f;
    g_safe_drift = 0.20f;
    g_tc_min = 5.0f;
    g_tc_max = 50.0f;
    g_td_min = -20.0f;
    g_td_max = 60.0f;
    g_maint_target_v = 51.0f;
  }
  g_sd_enable = server.hasArg("sd_en"); g_sd_spy = server.hasArg("spy"); g_serial_debug = server.hasArg("debug");
  preferences.putBool("sd_en", g_sd_enable); preferences.putBool("debug", g_serial_debug); preferences.putBool("spy", g_sd_spy);
  if(server.hasArg("theme")) g_theme_id = server.arg("theme").toInt();
  if(server.hasArg("can_proto")) g_can_protocol = server.arg("can_proto").toInt(); 
  g_mqtt_enable = server.hasArg("mq_en"); g_victron_enable = server.hasArg("vic_en");
  g_mqtt_full = server.hasArg("mq_full"); g_ha_enable = server.hasArg("ha_en"); 
  if (server.hasArg("mq_ip")) g_mqtt_server = server.arg("mq_ip");
  if (server.hasArg("mq_pt")) g_mqtt_port = server.arg("mq_pt").toInt();
  if (server.hasArg("mq_us")) g_mqtt_user = server.arg("mq_us");
  if (server.hasArg("mq_pw") && server.arg("mq_pw").length() > 0) g_mqtt_pass = server.arg("mq_pw");
  if (server.hasArg("mq_top") && server.arg("mq_top").length() > 0) g_mqtt_topic = server.arg("mq_top");
  if (server.hasArg("ntp_svr")) g_ntp_server = server.arg("ntp_svr");
  if (server.hasArg("tz_off")) g_timezone_offset = server.arg("tz_off").toInt();
  preferences.putInt("cnt", g_bms_count); preferences.putInt("cells", g_force_cell_count);
  preferences.putFloat("chg", g_charge_amps); preferences.putFloat("dis", g_discharge_amps);
  preferences.putBool("exp", g_expert_mode); preferences.putFloat("cvl", g_cvl_voltage);
  preferences.putInt("soc_mode", g_soc_source_mode);
  preferences.putInt("chart1", g_chart1);
  preferences.putInt("chart2", g_chart2);
  preferences.putBool("maint_en", g_maint_charge_mode);
  preferences.putFloat("maint_v", g_maint_target_v);
  preferences.putBool("ab_en", g_auto_balance_enable);
  preferences.putBool("easy", g_easy_mode); preferences.putBool("setupd", g_setup_done);
  preferences.putFloat("s_vol", g_safe_volt); preferences.putFloat("s_cel", g_safe_cell);
  preferences.putFloat("t_c_min", g_tc_min); preferences.putFloat("t_c_max", g_tc_max);
  preferences.putFloat("t_d_min", g_td_min); preferences.putFloat("t_d_max", g_td_max);
  preferences.putInt("t_mode", g_temp_mode); preferences.putFloat("s_drift", g_safe_drift);
  preferences.putBool("mq_en", g_mqtt_enable); preferences.putBool("mq_full", g_mqtt_full); preferences.putBool("vic_en", g_victron_enable); 
  preferences.putBool("ha_en", g_ha_enable); preferences.putInt("theme", g_theme_id); preferences.putInt("can_proto", g_can_protocol);
  preferences.putString("mq_ip", g_mqtt_server); preferences.putInt("mq_pt", g_mqtt_port);
  preferences.putString("mq_us", g_mqtt_user); preferences.putString("mq_pw", g_mqtt_pass);
  preferences.putString("mq_top", g_mqtt_topic);
  preferences.putString("ntp", g_ntp_server); preferences.putInt("tz", g_timezone_offset);
  // Board & pin settings
  if (server.hasArg("board_type")) {
    int new_board = constrain(server.arg("board_type").toInt(), 0, 2);
    g_board_type = new_board;
    preferences.putInt("board_type", g_board_type);
    if (new_board < 2) {
      applyBoardPreset(new_board);
    } else {
      // Custom: read pin values from form
      if (server.hasArg("p_rs_tx")) g_pin_rs485_tx = constrain(server.arg("p_rs_tx").toInt(), 0, 48);
      if (server.hasArg("p_rs_rx")) g_pin_rs485_rx = constrain(server.arg("p_rs_rx").toInt(), 0, 48);
      if (server.hasArg("p_rs_dir")) g_pin_rs485_dir = constrain(server.arg("p_rs_dir").toInt(), -1, 48);
      if (server.hasArg("p_can_tx")) g_pin_can_tx = constrain(server.arg("p_can_tx").toInt(), 0, 48);
      if (server.hasArg("p_can_rx")) g_pin_can_rx = constrain(server.arg("p_can_rx").toInt(), 0, 48);
      if (server.hasArg("p_led")) g_pin_led = constrain(server.arg("p_led").toInt(), -1, 48);
    }
    preferences.putInt("p_rs_tx", g_pin_rs485_tx); preferences.putInt("p_rs_rx", g_pin_rs485_rx);
    preferences.putInt("p_rs_dir", g_pin_rs485_dir); preferences.putInt("p_can_tx", g_pin_can_tx);
    preferences.putInt("p_can_rx", g_pin_can_rx); preferences.putInt("p_led", g_pin_led);
  }
  // Auth settings
  g_auth_enable = server.hasArg("auth_en");
  if (server.hasArg("auth_user") && server.arg("auth_user").length() > 0) g_auth_user = server.arg("auth_user");
  if (server.hasArg("auth_pw") && server.arg("auth_pw").length() > 0) g_auth_hash = sha256Hash(server.arg("auth_pw"));
  if (!g_auth_enable) g_auth_hash = "";
  preferences.putBool("auth_en", g_auth_enable);
  preferences.putString("auth_user", g_auth_user);
  preferences.putString("auth_hash", g_auth_hash);
  preferences.end(); delay(100);
  String rp = R"RP(<!DOCTYPE html><html><head><meta charset="UTF-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>Rebooting</title><style>
:root{--bg:#f3f4f6;--bg2:#fff;--tx:#1f2937;--tx2:#6b7280;--acc:#059669;--r:12px}
@media(prefers-color-scheme:dark){:root{--bg:#0f172a;--bg2:#1a1a2e;--tx:#e2e8f0;--tx2:#94a3b8}}
*{box-sizing:border-box;margin:0;padding:0}body{background:var(--bg);color:var(--tx);font-family:-apple-system,sans-serif;display:flex;justify-content:center;align-items:center;min-height:100vh}
.b{background:var(--bg2);border-radius:var(--r);padding:40px;text-align:center;max-width:400px;width:90%}
h2{font-size:16px;font-weight:500;margin-bottom:8px}
.s{color:var(--tx2);font-size:13px;margin-bottom:16px}
.bar{width:100%;height:4px;background:rgba(128,128,128,0.2);border-radius:2px;overflow:hidden}
.bar div{height:100%;background:var(--acc);border-radius:2px;width:0%;transition:width 0.5s}
.c{font-size:12px;color:var(--tx2);margin-top:12px}
</style></head><body><div class="b"><h2>Settings saved</h2><div class="s">Device is rebooting...</div><div class="bar"><div id="br"></div></div><div class="c" id="ct">Waiting for reboot...</div></div>
<script>var n=0,b=document.getElementById('br'),c=document.getElementById('ct');var iv=setInterval(function(){n++;b.style.width=Math.min(n*8,95)+'%';c.textContent='Waiting for reboot... ('+n+'s)';fetch('/data',{signal:AbortSignal.timeout(2000)}).then(function(){clearInterval(iv);b.style.width='100%';c.textContent='Loading dashboard...';location.href='/'}).catch(function(){})},3000)</script></body></html>)RP";
  server.send(200, "text/html", rp);
  delay(500); ESP.restart();
}
// /data endpoint: JSON with victron, config, status, energy, history, bms arrays.
// Takes a fast snapshot (<1ms) of shared data under dataMutex, then streams
// without holding the lock. Accepts ?c1=N&c2=N for live chart type switching.
void handleData() {
  if (!checkAuth(true)) return;
  // Snapshot shared data under mutex (fast, <1ms)
  VictronType vSnap;
  BMSData bSnap[MAX_BMS];
  uint8_t alarmSnap = alarm_flags;
  String alarmMsg = sys_error_msg;
  int bmsCnt = g_bms_count;
  if (dataMutex) xSemaphoreTake(dataMutex, pdMS_TO_TICKS(500));
  memcpy(&vSnap, &victronData, sizeof(VictronType));
  memcpy(bSnap, bms, sizeof(BMSData) * MAX_BMS);
  alarmSnap = alarm_flags;
  alarmMsg = sys_error_msg;
  if (dataMutex) xSemaphoreGive(dataMutex);
  // Now stream without holding mutex
  server.setContentLength(CONTENT_LENGTH_UNKNOWN);
  server.send(200, "application/json", "");
  
  // Helper: build + send a chunk, then free
  String c;
  c.reserve(512);

  // Victron section
  c = "{ \"victron\": {";
  c += "\"active\":" + String(vSnap.activePacks) + ",\"total_amps\":" + String(vSnap.totalCurrent, 1);
  c += ",\"total_power\":" + String(vSnap.totalPower, 0) + ",\"avg_soc\":" + String(vSnap.avgSOC, 1);
  c += ",\"avg_soh\":" + String(vSnap.avgSOH, 1) + ",\"voltage\":" + String(vSnap.avgVoltage, 2);
  c += ",\"rem_cap\":" + String(vSnap.remainCapacity, 0) + ",\"total_cap\":" + String(vSnap.totalCapacity, 0);
  c += ",\"avg_temp\":" + String(vSnap.avgTemp, 1);
  c += ",\"max_chg\":" + String(vSnap.maxChargeCurrent, 1) + ",\"max_dis\":" + String(vSnap.maxDischargeCurrent, 1);
  c += ",\"dyn_cvl\":" + String(g_runtime_cvl > 0.1f ? g_runtime_cvl : g_cvl_voltage, 2);
  c += ",\"proto_ccl\":" + String(g_runtime_proto_ccl_cap, 1) + ",\"proto_dcl\":" + String(g_runtime_proto_dcl_cap, 1);
  c += ",\"proto_uv_hit\":" + String(g_runtime_proto_uv_hit ? "true" : "false");
  c += ",\"maint_en\":" + String(g_maint_charge_mode ? "true" : "false") + ",\"maint_v\":" + String(g_maint_target_v, 2);
  c += ",\"autobal_due\":" + String(g_runtime_auto_balance_due ? "true" : "false");
  float conf_chg = bmsCnt * g_charge_amps;
  float conf_dis = bmsCnt * g_discharge_amps;
  c += ",\"cfg_chg\":" + String(conf_chg, 1) + ",\"cfg_dis\":" + String(conf_dis, 1);
  c += "}, ";
  server.sendContent(c);

  // Config section
  c = "\"config\": {";
  c += "\"cnt\":" + String(bmsCnt) + ",\"cells\":" + String(g_force_cell_count);
  c += ",\"chg\":" + String(g_charge_amps) + ",\"dis\":" + String(g_discharge_amps);
  c += ",\"cvl\":" + String(g_cvl_voltage) + ",\"exp\":" + String(g_expert_mode ? "true" : "false");
  c += ",\"s_vol\":" + String(g_safe_volt) + ",\"s_cel\":" + String(g_safe_cell);
  c += ",\"t_c_min\":" + String(g_tc_min) + ",\"t_c_max\":" + String(g_tc_max);
  c += ",\"t_d_min\":" + String(g_td_min) + ",\"t_d_max\":" + String(g_td_max);
  c += ",\"t_mode\":" + String(g_temp_mode) + ",\"s_drift\":" + String(g_safe_drift);
  c += ",\"debug\":" + String(g_serial_debug ? "true" : "false") + ",\"spy\":" + String(g_sd_spy ? "true" : "false");
  c += ",\"can_proto\":" + String(g_can_protocol) + ",\"theme\":" + String(g_theme_id);
  c += ",\"ntp\":\"" + g_ntp_server + "\",\"tz\":" + String(g_timezone_offset);
  c += ",\"sd\":" + String(g_sd_enable ? "true" : "false") + ",\"vic_en\":" + String(g_victron_enable ? "true" : "false");
  c += ",\"mq_en\":" + String(g_mqtt_enable ? "true" : "false") + ",\"mq_full\":" + String(g_mqtt_full ? "true" : "false");
  c += ",\"ha_en\":" + String(g_ha_enable ? "true" : "false") + ",\"mq_ip\":\"" + g_mqtt_server + "\"";
  c += ",\"mq_pt\":" + String(g_mqtt_port) + ",\"mq_us\":\"" + g_mqtt_user + "\"";
  c += ",\"mq_pw_set\":" + String(g_mqtt_pass.length() > 0 ? "true" : "false");
  c += ",\"mq_top\":\"" + g_mqtt_topic + "\"";
  c += ",\"mq_connected\":" + String(mqtt.connected() ? "true" : "false");
  c += ",\"fw\":\"" + String(FW_VERSION) + "\",\"dev_uid\":\"" + g_dev_uid + "\"";
  c += ",\"soc_mode\":" + String(g_soc_source_mode);
  c += ",\"maint_en\":" + String(g_maint_charge_mode ? "true" : "false") + ",\"maint_v\":" + String(g_maint_target_v, 2);
  c += ",\"ab_en\":" + String(g_auto_balance_enable ? "true" : "false") + ",\"ab_last\":" + String(g_auto_balance_last_ts);
  c += ",\"easy\":" + String(g_easy_mode ? "true" : "false") + ",\"setup_done\":" + String(g_setup_done ? "true" : "false");
  c += ",\"chart1\":" + String(g_chart1) + ",\"chart2\":" + String(g_chart2);
  c += ",\"auth_en\":" + String(g_auth_enable ? "true" : "false") + ",\"auth_user\":\"" + jsonEscape(g_auth_user) + "\"";
  c += ",\"board_type\":" + String(g_board_type);
  c += ",\"pins\":{\"rs_tx\":" + String(g_pin_rs485_tx) + ",\"rs_rx\":" + String(g_pin_rs485_rx);
  c += ",\"rs_dir\":" + String(g_pin_rs485_dir) + ",\"can_tx\":" + String(g_pin_can_tx);
  c += ",\"can_rx\":" + String(g_pin_can_rx) + ",\"led\":" + String(g_pin_led) + "}";
  c += ",\"dummy\":0}, ";
  server.sendContent(c);

  // Status section
  struct tm t; unsigned long ts = 0; if(getLocalTime(&t)) { time_t t_now; time(&t_now); ts = (unsigned long)t_now; }
  c = "\"ts\":" + String(ts);
  c += ",\"wifi_ip\":\"" + WiFi.localIP().toString() + "\"";
  c += ",\"wifi_rssi\":" + String(WiFi.RSSI());
  c += ",\"wifi_host\":\"" + String(HOSTNAME) + "\"";
  c += ",\"uptime_ms\":" + String(millis());
  c += ",\"free_heap\":" + String(ESP.getFreeHeap());
  c += ",\"min_heap\":" + String(heap_min);
  c += ",\"chip_temp\":" + String(temperatureRead(), 1);
  c += ",\"sketch_size\":" + String(ESP.getSketchSize());
  c += ",\"sketch_free\":" + String(ESP.getFreeSketchSpace());
  c += ",\"flash_size\":" + String(ESP.getFlashChipSize());
  c += ",\"chip_model\":\"" + String(ESP.getChipModel()) + "\"";
  c += ",\"chip_rev\":" + String(ESP.getChipRevision());
  c += ",\"chip_cores\":" + String(ESP.getChipCores());
  c += ",\"cpu_mhz\":" + String(ESP.getCpuFreqMHz());
  c += ",\"boot_reason\":\"" + g_boot_reason + "\"";
  c += ",\"mqtt_fails\":" + String(mqtt_fail_count);
  String l = debug_log; l.replace("\"", "'"); l.replace("\n", "");
  c += ",\"log\":\"" + l + "\", ";
  l = "";  // free debug_log copy
  if(!g_victron_enable) c += "\"can_status\":\"DISABLED\",\"can_error\":false, ";
  else c += "\"can_status\":\"" + debug_can_status + "\",\"can_error\":" + String(can_error_flag ? "true" : "false") + ", ";
  #if HAS_SD_CARD
  c += "\"sd_ok\":" + String(sd_ok ? "true" : "false") + ",\"sd_err\":" + String(sd_error_flag ? "true" : "false") + ", ";
  #else
  c += "\"sd_ok\":false,\"sd_err\":false, ";
  #endif
  c += "\"safe_lock\":" + String(((alarmSnap & 0x02) || (alarmSnap & 0x10) || (alarmSnap & 0x40) || (alarmSnap & 0x80)) ? "true" : "false");
  c += ",\"alarm\":\"" + jsonEscape(alarmMsg) + "\", ";
  server.sendContent(c);

  // Energy section
  c = "\"energy\":{\"in\":" + String(energy_in_today, 2) + ",\"out\":" + String(energy_out_today, 2);
  // 7-day rolling totals
  float week_in = energy_in_today, week_out = energy_out_today;
  c += ",\"days_in\":["; for(int i=0;i<7;i++){if(i)c+=",";c+=String(energy_days_in[i],1);week_in+=energy_days_in[i];}
  c += "],\"days_out\":["; for(int i=0;i<7;i++){if(i)c+=",";c+=String(energy_days_out[i],1);week_out+=energy_days_out[i];}
  c += "],\"week_in\":" + String(week_in, 1) + ",\"week_out\":" + String(week_out, 1);
  c += ",\"month_in\":" + String(energy_months_in[0] + energy_in_today, 1);
  c += ",\"month_out\":" + String(energy_months_out[0] + energy_out_today, 1);
  c += "}, ";
  server.sendContent(c);

  // History - two configurable charts, downsampled, chunked
  auto sendHist = [&](const char* name, int16_t* src) {
    server.sendContent(String("\"") + name + "\":[");
    const int DS = 5;
    int startIdx = historyIdx;
    bool hFirst = true;
    c = "";
    int cnt = 0;
    for (int i = 0; i < HISTORY_LEN; i += DS) {
      int32_t sum = 0;
      for (int k = 0; k < DS && (i + k) < HISTORY_LEN; k++) sum += src[(startIdx + i + k) % HISTORY_LEN];
      if (!hFirst) c += ",";
      c += String((int16_t)(sum / DS));
      hFirst = false;
      if (++cnt >= 32) { server.sendContent(c); c = ""; cnt = 0; }
    }
    if (c.length() > 0) server.sendContent(c);
    server.sendContent("], ");
  };
  int16_t* chartSrc[] = { powerHistory, voltHistory, socHistory, tempHistory };
  int16_t* chartMin[] = { powerMin, voltMin, socMin, tempMin };
  int16_t* chartMax[] = { powerMax, voltMax, socMax, tempMax };
  int ci1 = g_chart1 < 4 ? g_chart1 : 0, ci2 = g_chart2 < 4 ? g_chart2 : 1;
  if (server.hasArg("c1")) ci1 = constrain(server.arg("c1").toInt(), 0, 3);
  if (server.hasArg("c2")) ci2 = constrain(server.arg("c2").toInt(), 0, 3);
  sendHist("hist1", chartSrc[ci1]);
  sendHist("h1mn", chartMin[ci1]);
  sendHist("h1mx", chartMax[ci1]);
  sendHist("hist2", chartSrc[ci2]);
  sendHist("h2mn", chartMin[ci2]);
  sendHist("h2mx", chartMax[ci2]);
  // V2.61: hstart = index in downsampled array (192 points) where real data begins.
  // Points before this are unwritten zeros from boot, client must skip them.
  {
    const int DS = 5;
    int unfilled = HISTORY_LEN - historyFilled;
    int hstart = (unfilled + DS - 1) / DS;  // round up so we never show partial-empty downsampled cells
    if (hstart < 0) hstart = 0;
    if (hstart > HISTORY_LEN/DS) hstart = HISTORY_LEN/DS;
    server.sendContent("\"hstart\":" + String(hstart) + ", ");
  }

  // BMS array - one BMS per chunk
  server.sendContent("\"bms\":[");
  for (int i = 0; i < bmsCnt; i++) {
    if (i > 0) server.sendContent(",");
    bool on = (bSnap[i].valid && (millis() - bSnap[i].last_seen < 120000)) || simulation_active;
    c = "{\"id\":" + String(i) + ",\"online\":" + String(on ? "true" : "false");
    if (on) {
      c += ",\"soc\":" + String(bSnap[i].soc) + ",\"current\":" + String(bSnap[i].current, 1);
      c += ",\"pack_v\":" + String(bSnap[i].voltage, 2) + ",\"soh\":" + String(bSnap[i].soh);
      c += ",\"min_cell\":" + String(bSnap[i].minCellV, 3) + ",\"max_cell\":" + String(bSnap[i].maxCellV, 3);
      c += ",\"avg_temp\":" + String(bSnap[i].avgTemp, 1);
      c += ",\"cycles\":" + String(bSnap[i].cycles);
      c += ",\"temps\":[";
      for (int t = 0; t < bSnap[i].temp_count; t++) {
        if (t > 0) c += ",";
        String lbl = "T" + String(t + 1); if (t == 4) lbl = "MOS"; if (t == 5) lbl = "ENV"; if (t == 6) lbl = "BAL";
        c += "{\"val\":" + String(bSnap[i].temps[t], 1) + ",\"lbl\":\"" + lbl + "\"}";
      }
      c += "],\"cells\":[";
      for (int cc = 0; cc < bSnap[i].cell_count; cc++) { if (cc > 0) c += ","; c += String(bSnap[i].cells[cc], 3); }
      c += "]";
    }
    c += ",\"stats\":{\"polls\":" + String(g_bms_stats[i].polls) + ",\"ok\":" + String(g_bms_stats[i].ok);
    c += ",\"timeout\":" + String(g_bms_stats[i].timeouts) + ",\"err\":" + String(g_bms_stats[i].errors);
    c += ",\"spikes\":" + String(g_bms_stats[i].spikes) + "}";
    c += "}";
    server.sendContent(c);
  }
  server.sendContent("], ");
  // V2.61: embed alert ring buffer (newest first)
  server.sendContent("\"alerts\":[");
  {
    bool first = true;
    for (int i = 1; i <= g_alert_count; i++) {
      int idx = (g_alert_head - i + ALERT_RING_SIZE) % ALERT_RING_SIZE;
      if (!first) server.sendContent(",");
      first = false;
      String m; m.reserve(ALERT_MSG_LEN + 16);
      const char* p = g_alerts[idx].msg;
      for (int k = 0; k < ALERT_MSG_LEN && p[k]; k++) {
        char ch = p[k];
        if (ch == '"' || ch == '\\') { m += '\\'; m += ch; }
        else if ((uint8_t)ch < 0x20) m += ' ';
        else m += ch;
      }
      String ae = "{\"ts\":" + String(g_alerts[idx].ts)
        + ",\"up\":" + String(g_alerts[idx].uptime_s)
        + ",\"sev\":" + String((int)g_alerts[idx].sev)
        + ",\"msg\":\"" + m + "\"}";
      server.sendContent(ae);
    }
  }
  server.sendContent("] }");
  server.sendContent("");  // signal end of chunked transfer
}

// ==========================================
// OTA FIRMWARE UPDATE
// Standalone page at /update (GET) + upload handler (POST).
// saveHistory(true) is called before update begins to preserve chart data.
// ==========================================
void handleOTAPage() {
  if (!checkAuth()) return;
  String h = R"rawliteral(<!DOCTYPE html><html><head><meta charset="UTF-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>Firmware Update</title><style>
:root{--bg:#f3f4f6;--bg2:#fff;--bg3:#f3f4f6;--tx:#1f2937;--tx2:#6b7280;--tx3:#9ca3af;--bd:rgba(0,0,0,0.15);--acc:#059669;--acc2:#047857;--r:8px;--r2:12px}
@media(prefers-color-scheme:dark){:root{--bg:#0f172a;--bg2:#1a1a2e;--bg3:#16213e;--tx:#e2e8f0;--tx2:#94a3b8;--tx3:#64748b;--bd:rgba(255,255,255,0.12)}}
*{box-sizing:border-box;margin:0;padding:0}body{background:var(--bg);color:var(--tx);font-family:-apple-system,BlinkMacSystemFont,'Segoe UI',sans-serif;display:flex;justify-content:center;align-items:center;min-height:100vh;padding:20px}
.box{background:var(--bg2);border:0.5px solid var(--bd);border-radius:var(--r2);padding:30px;max-width:450px;width:100%;text-align:center}
h2{font-size:16px;font-weight:500;margin-bottom:4px}
.ver{color:var(--tx3);font-size:12px;margin-bottom:20px}
.drop{border:1.5px dashed var(--bd);border-radius:var(--r);padding:40px 20px;margin:16px 0;cursor:pointer;transition:all 0.2s}
.drop:hover,.drop.over{border-color:var(--acc);background:rgba(5,150,105,0.05)}
.drop input{display:none}.drop label{cursor:pointer;color:var(--tx2);font-size:13px}
.obtn{background:var(--acc);color:#fff;border:none;padding:10px 24px;border-radius:var(--r);font-size:13px;cursor:pointer;width:100%;margin-top:12px;font-weight:500}
.obtn:hover{background:var(--acc2)}.obtn:disabled{background:var(--bg3);color:var(--tx3);cursor:not-allowed}
.prog{width:100%;height:6px;border-radius:3px;background:var(--bg3);margin:16px 0 8px;overflow:hidden;display:none}
.prog div{height:100%;background:var(--acc);border-radius:3px;width:0%;transition:width 0.3s}
.msg{font-size:12px;min-height:20px;margin-top:8px}
.msg.ok{color:var(--acc)}.msg.err{color:#ef4444}.msg.info{color:#3b82f6}
a.back{color:var(--tx3);font-size:12px;text-decoration:none;display:inline-block;margin-top:16px}a.back:hover{color:var(--tx2)}
</style></head><body>
<div class="box">
<h2>Firmware Update</h2>
<div class="ver" id="ota_ver"></div>
<div class="drop" id="drop">
<input type="file" id="file" accept=".bin">
<label for="file" id="flbl">Select .bin file or drag and drop here</label>
</div>
<div class="prog" id="prog"><div id="bar"></div></div>
<div class="msg" id="msg"></div>
<button class="obtn" id="obtn" onclick="doUpload()" disabled>Upload firmware</button>
<br><a class="back" href="/">&larr; Back to dashboard</a>
</div>
<script>
var fileData=null;
var dp=document.getElementById('drop');
var fi=document.getElementById('file');
dp.ondragover=function(ev){ev.preventDefault();dp.classList.add('over');};
dp.ondragleave=function(){dp.classList.remove('over');};
dp.ondrop=function(ev){ev.preventDefault();dp.classList.remove('over');if(ev.dataTransfer.files.length)pickFile(ev.dataTransfer.files[0]);};
fi.onchange=function(){if(fi.files.length)pickFile(fi.files[0]);};
function pickFile(f){
  if(!f.name.endsWith('.bin')){showMsg('Only .bin files allowed','err');return;}
  fileData=f;
  document.getElementById('flbl').innerText=f.name+' ('+Math.round(f.size/1024)+' KB)';
  document.getElementById('obtn').disabled=false;
  showMsg('Ready to upload','info');
}
function showMsg(t,c){var m=document.getElementById('msg');m.className='msg '+(c?c:'');m.innerText=t;}
function doUpload(){
  if(!fileData)return;
  var xhr=new XMLHttpRequest();
  var fd=new FormData();
  fd.append('update',fileData);
  document.getElementById('obtn').disabled=true;
  document.getElementById('prog').style.display='block';
  showMsg('Uploading...','info');
  var uploadDone=false;
  xhr.upload.onprogress=function(ev){if(ev.lengthComputable){var p=Math.round(ev.loaded/ev.total*100);document.getElementById('bar').style.width=p+'%';showMsg('Upload: '+p+'%','info');if(p>=100)uploadDone=true;}};
  xhr.onload=function(){showMsg('Update successful! Rebooting...','ok');pollBoot();};
  xhr.onerror=function(){if(uploadDone){showMsg('Update successful! Rebooting...','ok');pollBoot();}else{showMsg('Connection error','err');document.getElementById('obtn').disabled=false;}};
  function pollBoot(){var n=0;var iv=setInterval(function(){n++;showMsg('Waiting for reboot... ('+n+'s)','info');fetch('/data',{signal:AbortSignal.timeout(2000)}).then(function(){clearInterval(iv);location.href='/'}).catch(function(){})},3000)}
  xhr.open('POST','/update');
  xhr.send(fd);
}
fetch('/data').then(function(r){return r.json();}).then(function(d){if(d.config&&d.config.fw)document.getElementById('ota_ver').innerText='Current version: V'+d.config.fw;});
</script></body></html>)rawliteral";
  server.sendHeader("Cache-Control", "public, max-age=600");
  server.send(200, "text/html", h);
}

void handleOTAUpload() {
  if (!checkAuth()) return;
  HTTPUpload& upload = server.upload();
  if (upload.status == UPLOAD_FILE_START) {
    saveHistory(true);
    if (g_alerts_dirty) saveAlertsToNvs();   // V2.61
    addToLog("OTA: Start " + upload.filename, false);
    if (!Update.begin(UPDATE_SIZE_UNKNOWN)) {
      addToLog("OTA: Begin failed", true);
    }
  } else if (upload.status == UPLOAD_FILE_WRITE) {
    esp_task_wdt_reset();
    if (Update.write(upload.buf, upload.currentSize) != upload.currentSize) {
      addToLog("OTA: Write failed", true);
    }
  } else if (upload.status == UPLOAD_FILE_END) {
    if (Update.end(true)) {
      addToLog("OTA: Success, rebooting", false);
    } else {
      addToLog("OTA: End failed", true);
    }
  }
}

void handleOTADone() {
  if (!checkAuth()) return;
  if (Update.hasError()) {
    server.send(500, "text/plain", "Update failed");
  } else {
    String rp = R"RP(<!DOCTYPE html><html><head><meta charset="UTF-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>Rebooting</title><style>
:root{--bg:#f3f4f6;--bg2:#fff;--tx:#1f2937;--tx2:#6b7280;--acc:#059669;--r:12px}
@media(prefers-color-scheme:dark){:root{--bg:#0f172a;--bg2:#1a1a2e;--tx:#e2e8f0;--tx2:#94a3b8}}
*{box-sizing:border-box;margin:0;padding:0}body{background:var(--bg);color:var(--tx);font-family:-apple-system,sans-serif;display:flex;justify-content:center;align-items:center;min-height:100vh}
.b{background:var(--bg2);border-radius:var(--r);padding:40px;text-align:center;max-width:400px;width:90%}
h2{font-size:16px;font-weight:500;margin-bottom:8px}.s{color:var(--tx2);font-size:13px;margin-bottom:16px}
.bar{width:100%;height:4px;background:rgba(128,128,128,0.2);border-radius:2px;overflow:hidden}.bar div{height:100%;background:var(--acc);border-radius:2px;width:0%;transition:width 0.5s}
.c{font-size:12px;color:var(--tx2);margin-top:12px}
</style></head><body><div class="b"><h2>Firmware updated</h2><div class="s">Device is rebooting...</div><div class="bar"><div id="br"></div></div><div class="c" id="ct">Waiting for reboot...</div></div>
<script>var n=0,b=document.getElementById('br'),c=document.getElementById('ct');var iv=setInterval(function(){n++;b.style.width=Math.min(n*5,95)+'%';c.textContent='Waiting for reboot... ('+n+'s)';fetch('/data',{signal:AbortSignal.timeout(2000)}).then(function(){clearInterval(iv);b.style.width='100%';c.textContent='Loading dashboard...';location.href='/'}).catch(function(){})},3000)</script></body></html>)RP";
    server.send(200, "text/html", rp);
    delay(1000);
    ESP.restart();
  }
}

// ==========================================
// SETTINGS BACKUP & RESTORE
// Backup exports all NVS settings as JSON (excludes MQTT password and auth hash).
// Restore imports JSON, writes to NVS, and reboots.
// ==========================================
void handleBackup() {
  if (!checkAuth()) return;
  preferences.begin("gateway", true);
  String j = "{\n";
  j += "  \"_info\": \"TopBand BMS Gateway Settings Backup\",\n";
  j += "  \"_fw\": \"" + String(FW_VERSION) + "\",\n";
  j += "  \"_uid\": \"" + g_dev_uid + "\",\n";
  j += "  \"cnt\": " + String(preferences.getInt("cnt", 2)) + ",\n";
  j += "  \"cells\": " + String(preferences.getInt("cells", 0)) + ",\n";
  j += "  \"chg\": " + String(preferences.getFloat("chg", 30.0), 1) + ",\n";
  j += "  \"dis\": " + String(preferences.getFloat("dis", 30.0), 1) + ",\n";
  j += "  \"cvl\": " + String(preferences.getFloat("cvl", 52.5), 2) + ",\n";
  j += "  \"exp\": " + String(preferences.getBool("exp", true) ? "true" : "false") + ",\n";
  j += "  \"s_vol\": " + String(preferences.getFloat("s_vol", 53.25), 2) + ",\n";
  j += "  \"s_cel\": " + String(preferences.getFloat("s_cel", 3.55), 3) + ",\n";
  j += "  \"s_drift\": " + String(preferences.getFloat("s_drift", 0.20), 2) + ",\n";
  j += "  \"t_c_min\": " + String(preferences.getFloat("t_c_min", 5.0), 1) + ",\n";
  j += "  \"t_c_max\": " + String(preferences.getFloat("t_c_max", 50.0), 1) + ",\n";
  j += "  \"t_d_min\": " + String(preferences.getFloat("t_d_min", -20.0), 1) + ",\n";
  j += "  \"t_d_max\": " + String(preferences.getFloat("t_d_max", 60.0), 1) + ",\n";
  j += "  \"t_mode\": " + String(preferences.getInt("t_mode", 0)) + ",\n";
  j += "  \"soc_mode\": " + String(preferences.getInt("soc_mode", 2)) + ",\n";
  j += "  \"maint_en\": " + String(preferences.getBool("maint_en", false) ? "true" : "false") + ",\n";
  j += "  \"maint_v\": " + String(preferences.getFloat("maint_v", 51.0), 2) + ",\n";
  j += "  \"ab_en\": " + String(preferences.getBool("ab_en", true) ? "true" : "false") + ",\n";
  j += "  \"vic_en\": " + String(preferences.getBool("vic_en", true) ? "true" : "false") + ",\n";
  j += "  \"can_proto\": " + String(preferences.getInt("can_proto", 0)) + ",\n";
  j += "  \"mq_en\": " + String(preferences.getBool("mq_en", false) ? "true" : "false") + ",\n";
  j += "  \"mq_full\": " + String(preferences.getBool("mq_full", false) ? "true" : "false") + ",\n";
  j += "  \"ha_en\": " + String(preferences.getBool("ha_en", false) ? "true" : "false") + ",\n";
  j += "  \"mq_ip\": \"" + jsonEscape(preferences.getString("mq_ip", "")) + "\",\n";
  j += "  \"mq_pt\": " + String(preferences.getInt("mq_pt", 1883)) + ",\n";
  j += "  \"mq_us\": \"" + jsonEscape(preferences.getString("mq_us", "")) + "\",\n";
  j += "  \"mq_top\": \"" + jsonEscape(preferences.getString("mq_top", "Topband/BMS")) + "\",\n";
  j += "  \"ntp\": \"" + jsonEscape(preferences.getString("ntp", "pool.ntp.org")) + "\",\n";
  j += "  \"tz\": " + String(preferences.getInt("tz", 1)) + ",\n";
  j += "  \"chart1\": " + String(preferences.getInt("chart1", 0)) + ",\n";
  j += "  \"chart2\": " + String(preferences.getInt("chart2", 1)) + ",\n";
  j += "  \"auth_en\": " + String(preferences.getBool("auth_en", false) ? "true" : "false") + ",\n";
  j += "  \"auth_user\": \"" + jsonEscape(preferences.getString("auth_user", "admin")) + "\",\n";
  j += "  \"board_type\": " + String(preferences.getInt("board_type", 0)) + ",\n";
  j += "  \"p_rs_tx\": " + String(preferences.getInt("p_rs_tx", 17)) + ",\n";
  j += "  \"p_rs_rx\": " + String(preferences.getInt("p_rs_rx", 18)) + ",\n";
  j += "  \"p_rs_dir\": " + String(preferences.getInt("p_rs_dir", 21)) + ",\n";
  j += "  \"p_can_tx\": " + String(preferences.getInt("p_can_tx", 15)) + ",\n";
  j += "  \"p_can_rx\": " + String(preferences.getInt("p_can_rx", 16)) + ",\n";
  j += "  \"p_led\": " + String(preferences.getInt("p_led", 38)) + "\n";
  j += "}";
  preferences.end();
  server.sendHeader("Content-Disposition", "attachment; filename=topband_backup.json");
  server.send(200, "application/json", j);
  addToLog("Settings backup downloaded", false);
}

void handleRestore() {
  if (!checkAuth()) return;
  if (!server.hasArg("plain")) {
    server.send(400, "application/json", "{\"ok\":false,\"err\":\"no body\"}");
    return;
  }
  String body = server.arg("plain");
  if (body.length() < 10 || body.indexOf("cnt") < 0) {
    server.send(400, "application/json", "{\"ok\":false,\"err\":\"invalid json\"}");
    return;
  }
  // Simple JSON value extractors
  auto getInt = [&](const char* key, int def) -> int {
    String k = String("\"") + key + "\":";
    int p = body.indexOf(k);
    if (p < 0) return def;
    p += k.length();
    while (p < body.length() && body[p] == ' ') p++;
    return body.substring(p).toInt();
  };
  auto getFloat = [&](const char* key, float def) -> float {
    String k = String("\"") + key + "\":";
    int p = body.indexOf(k);
    if (p < 0) return def;
    p += k.length();
    while (p < body.length() && body[p] == ' ') p++;
    return body.substring(p).toFloat();
  };
  auto getBool = [&](const char* key, bool def) -> bool {
    String k = String("\"") + key + "\":";
    int p = body.indexOf(k);
    if (p < 0) return def;
    p += k.length();
    while (p < body.length() && body[p] == ' ') p++;
    return body.substring(p, p + 4) == "true";
  };
  auto getStr = [&](const char* key, String def) -> String {
    String k = String("\"") + key + "\": \"";
    int p = body.indexOf(k);
    if (p < 0) { k = String("\"") + key + "\":\""; p = body.indexOf(k); }
    if (p < 0) return def;
    p += k.length();
    int e = body.indexOf("\"", p);
    if (e < 0) return def;
    return body.substring(p, e);
  };

  preferences.begin("gateway", false);
  preferences.putInt("cnt", getInt("cnt", 2));
  preferences.putInt("cells", getInt("cells", 0));
  preferences.putFloat("chg", getFloat("chg", 30.0));
  preferences.putFloat("dis", getFloat("dis", 30.0));
  preferences.putFloat("cvl", getFloat("cvl", 52.5));
  preferences.putBool("exp", getBool("exp", true));
  preferences.putFloat("s_vol", getFloat("s_vol", 53.25));
  preferences.putFloat("s_cel", getFloat("s_cel", 3.55));
  preferences.putFloat("s_drift", getFloat("s_drift", 0.20));
  preferences.putFloat("t_c_min", getFloat("t_c_min", 5.0));
  preferences.putFloat("t_c_max", getFloat("t_c_max", 50.0));
  preferences.putFloat("t_d_min", getFloat("t_d_min", -20.0));
  preferences.putFloat("t_d_max", getFloat("t_d_max", 60.0));
  preferences.putInt("t_mode", getInt("t_mode", 0));
  preferences.putInt("soc_mode", getInt("soc_mode", 2));
  preferences.putBool("maint_en", getBool("maint_en", false));
  preferences.putFloat("maint_v", getFloat("maint_v", 51.0));
  preferences.putBool("ab_en", getBool("ab_en", true));
  preferences.putBool("vic_en", getBool("vic_en", true));
  preferences.putInt("can_proto", getInt("can_proto", 0));
  preferences.putBool("mq_en", getBool("mq_en", false));
  preferences.putBool("mq_full", getBool("mq_full", false));
  preferences.putBool("ha_en", getBool("ha_en", false));
  preferences.putString("mq_ip", getStr("mq_ip", ""));
  preferences.putInt("mq_pt", getInt("mq_pt", 1883));
  preferences.putString("mq_us", getStr("mq_us", ""));
  preferences.putString("mq_top", getStr("mq_top", "Topband/BMS"));
  preferences.putString("ntp", getStr("ntp", "pool.ntp.org"));
  preferences.putInt("tz", getInt("tz", 1));
  preferences.putInt("chart1", getInt("chart1", 0));
  preferences.putInt("chart2", getInt("chart2", 1));
  preferences.putBool("auth_en", getBool("auth_en", false));
  preferences.putString("auth_user", getStr("auth_user", "admin"));
  // Pin configuration
  preferences.putInt("board_type", getInt("board_type", 0));
  preferences.putInt("p_rs_tx", getInt("p_rs_tx", 17));
  preferences.putInt("p_rs_rx", getInt("p_rs_rx", 18));
  preferences.putInt("p_rs_dir", getInt("p_rs_dir", 21));
  preferences.putInt("p_can_tx", getInt("p_can_tx", 15));
  preferences.putInt("p_can_rx", getInt("p_can_rx", 16));
  preferences.putInt("p_led", getInt("p_led", 38));
  // Note: auth_hash intentionally excluded from restore for security
  preferences.putBool("setupd", true);
  preferences.end();

  addToLog("Settings restored from backup", false);
  server.send(200, "application/json", "{\"ok\":true}");
  delay(500);
  ESP.restart();
}

// ==========================================
// HISTORY EXPORT (CSV download of chart data)
// Exports all 4 chart types as CSV with timestamps.
// Each row = one 3-minute interval, 960 rows = 48h.
// ==========================================
void handleExport() {
  if (!checkAuth()) return;
  server.sendHeader("Content-Disposition", "attachment; filename=\"topband_history.csv\"");
  server.sendHeader("Cache-Control", "no-cache");
  server.setContentLength(CONTENT_LENGTH_UNKNOWN);
  server.send(200, "text/csv", "");
  server.sendContent("index,minutes_ago,power_w,voltage_v,soc_pct,temperature_c\n");
  String buf; buf.reserve(512);
  int cnt = 0;
  for (int i = 0; i < HISTORY_LEN; i++) {
    int idx = (historyIdx + i) % HISTORY_LEN;
    int mins_ago = (HISTORY_LEN - i) * 3;
    float pw = (float)powerHistory[idx];
    float vt = (float)voltHistory[idx] * 0.01f;
    float sc = (float)socHistory[idx] * 0.1f;
    float tp = (float)tempHistory[idx] * 0.1f;
    buf += String(i) + "," + String(mins_ago) + "," + String(pw, 0) + "," + String(vt, 2) + "," + String(sc, 1) + "," + String(tp, 1) + "\n";
    if (++cnt >= 40) { server.sendContent(buf); buf = ""; cnt = 0; }
  }
  if (buf.length() > 0) server.sendContent(buf);
  server.sendContent("");
}

void handleServicePage() {
  if (!checkAuth()) return;
  String h = R"SVCHTML(<!DOCTYPE html><html><head><meta charset="UTF-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>BMS Diagnostics</title><style>
:root{--bg:#f3f4f6;--bg2:#fff;--bg3:#f3f4f6;--tx:#1f2937;--tx2:#6b7280;--tx3:#9ca3af;--bd:rgba(0,0,0,0.15);--bd2:rgba(0,0,0,0.3);--acc:#059669;--acc2:#047857;--accbg:#dcfce7;--acctx:#166534;--red:#f87171;--redbg:#fee2e2;--redtx:#991b1b;--amber:#fbbf24;--amberbg:#fef3c7;--ambertx:#92400e;--r:8px;--r2:12px;--mono:'SF Mono',Consolas,monospace}
@media(prefers-color-scheme:dark){:root{--bg:#0f172a;--bg2:#1a1a2e;--bg3:#16213e;--tx:#e2e8f0;--tx2:#94a3b8;--tx3:#64748b;--bd:rgba(255,255,255,0.12);--bd2:rgba(255,255,255,0.25);--accbg:rgba(22,101,52,0.3);--acctx:#4ade80;--redbg:rgba(153,27,27,0.3);--redtx:#fca5a5;--amberbg:rgba(146,64,14,0.3);--ambertx:#fbbf24}}
*{box-sizing:border-box;margin:0;padding:0}body{background:var(--bg);color:var(--tx);font-family:-apple-system,BlinkMacSystemFont,'Segoe UI',sans-serif;padding:20px}
.gw{max-width:700px;margin:0 auto}
.top{background:var(--bg2);border:0.5px solid var(--bd);border-bottom:2px solid var(--acc);border-radius:var(--r2) var(--r2) 0 0;padding:12px 16px;display:flex;justify-content:space-between;align-items:center}
.top h1{font-size:15px;font-weight:500}.top .v{font-size:11px;color:var(--tx3)}
.ct{background:var(--bg2);border:0.5px solid var(--bd);border-top:none;border-radius:0 0 var(--r2) var(--r2);padding:16px}
.sl{font-size:10px;color:var(--tx3);text-transform:uppercase;letter-spacing:0.06em;margin:14px 0 8px;padding-bottom:5px;border-bottom:0.5px solid var(--bd)}
.sl:first-child{margin-top:0}
.row{display:flex;justify-content:space-between;font-size:12px;padding:4px 0;border-bottom:0.5px solid var(--bd)}
.row .k{color:var(--tx2)}.row .vl{font-weight:500;font-family:var(--mono);font-size:11px}
.row .vl.ok{color:var(--acctx)}.row .vl.er{color:var(--redtx)}.row .vl.wa{color:var(--ambertx)}
.ctr{display:flex;gap:8px;align-items:center;margin-bottom:12px;flex-wrap:wrap}
select,input{padding:7px 9px;border:0.5px solid var(--bd2);border-radius:var(--r);background:var(--bg3);color:var(--tx);font-size:12px;outline:none}
select:focus,input:focus{border-color:var(--acc)}
.bp{background:var(--acc);color:#fff;border:none;padding:8px 16px;border-radius:var(--r);font-size:12px;font-weight:500;cursor:pointer}
.bp:hover{background:var(--acc2)}.bp:disabled{background:var(--bg3);color:var(--tx3);cursor:not-allowed}
.bo{background:transparent;border:0.5px solid var(--bd2);padding:7px 14px;border-radius:var(--r);font-size:12px;cursor:pointer;color:var(--tx)}
.bo:hover{background:var(--bg3)}
.msg{font-size:12px;padding:8px 12px;border-radius:var(--r);margin:8px 0}
.msg.ok{background:var(--accbg);color:var(--acctx)}.msg.er{background:var(--redbg);color:var(--redtx)}.msg.ld{background:var(--bg3);color:var(--tx3)}
a.back{color:var(--tx3);font-size:12px;text-decoration:none;display:inline-block;margin-top:14px}a.back:hover{color:var(--tx2)}
.pill{font-size:10px;padding:2px 8px;border-radius:10px;font-weight:500}
.pill-g{background:var(--accbg);color:var(--acctx)}.pill-r{background:var(--redbg);color:var(--redtx)}.pill-y{background:var(--amberbg);color:var(--ambertx)}
.alarm-list{margin-top:6px}.alarm-list span{display:inline-block;font-size:11px;background:var(--bg3);padding:2px 8px;border-radius:4px;margin:2px 4px 2px 0}
</style></head><body>
<div class="gw">
<div class="top"><h1>BMS Diagnostics <span class="v" id="sfw"></span></h1></div>
<div class="ct">
<div class="ctr">
<select id="sid"></select>
<button class="bp" id="rbtn" onclick="rd()">Read diagnostics</button>
<button class="bo" onclick="st()">Sync time</button>
</div>
<div id="stt" class="msg ld" style="display:none"></div>
<div id="out"></div>
<a class="back" href="/">&larr; Back to dashboard</a>
</div></div>
<script>
function $(i){return document.getElementById(i)}
function msg(t,c){var e=$('stt');e.style.display='block';e.className='msg '+(c||'ld');e.textContent=t}
fetch('/data').then(function(r){return r.json()}).then(function(d){
  var c=d.config||{};var sel=$('sid');
  for(var i=0;i<(c.cnt||2);i++){var o=document.createElement('option');o.value=i;o.textContent='BMS #'+i;sel.appendChild(o)}
  $('sfw').textContent='V'+(c.fw||'');
});
function rd(){
  var id=$('sid').value;msg('Reading BMS '+id+'...');$('rbtn').disabled=true;
  fetch('/svc/bms?id='+id).then(function(r){return r.json()}).then(function(d){
    $('rbtn').disabled=false;
    if(!d.ok){msg('Error: '+(d.err||'unknown'),'er');return}
    msg('BMS '+id+' data loaded','ok');
    var h='';
    if(d.manufacturer&&d.manufacturer.ok){
      h+='<div class="sl">Manufacturer</div>';
      h+='<div class="row"><span class="k">Hardware</span><span class="vl">'+esc(d.manufacturer.hw)+'</span></div>';
      h+='<div class="row"><span class="k">Software</span><span class="vl">'+esc(d.manufacturer.sw)+'</span></div>';
      h+='<div class="row"><span class="k">ID</span><span class="vl">'+esc(d.manufacturer.id)+'</span></div>';
    }
    if(d.date&&d.date.ok){
      h+='<div class="sl">BMS clock</div>';
      h+='<div class="row"><span class="k">Date/Time</span><span class="vl">'+d.date.year+'-'+pad(d.date.month)+'-'+pad(d.date.day)+' '+pad(d.date.hour)+':'+pad(d.date.minute)+':'+pad(d.date.second)+'</span></div>';
    }
    if(d.analog&&d.analog.ok){
      var a=d.analog;
      h+='<div class="sl">Analog data (0x42)</div>';
      h+='<div class="row"><span class="k">Pack voltage</span><span class="vl">'+a.pack_v.toFixed(2)+' V</span></div>';
      h+='<div class="row"><span class="k">Current</span><span class="vl">'+a.current.toFixed(2)+' A</span></div>';
      h+='<div class="row"><span class="k">SOC / SOH</span><span class="vl">'+a.soc+'% / '+a.soh+'%</span></div>';
      h+='<div class="row"><span class="k">Capacity</span><span class="vl">'+a.rem_ah.toFixed(2)+' / '+a.full_ah.toFixed(2)+' Ah</span></div>';
      h+='<div class="row"><span class="k">Cells / Temps</span><span class="vl">'+a.cells+' / '+a.temps+'</span></div>';
      h+='<div class="row"><span class="k">Cell range</span><span class="vl">'+a.min_cell.toFixed(3)+' .. '+a.max_cell.toFixed(3)+' V</span></div>';
    }
    if(d.system&&d.system.ok){
      var s=d.system;
      h+='<div class="sl">System parameters (0x47)</div>';
      h+='<div class="row"><span class="k">Cell high / low / under</span><span class="vl">'+s.cell_high_v.toFixed(3)+' / '+s.cell_low_v.toFixed(3)+' / '+s.cell_under_v.toFixed(3)+' V</span></div>';
      h+='<div class="row"><span class="k">Module high / low / under</span><span class="vl">'+s.module_high_v.toFixed(2)+' / '+s.module_low_v.toFixed(2)+' / '+s.module_under_v.toFixed(2)+' V</span></div>';
      h+='<div class="row"><span class="k">Charge current max</span><span class="vl">'+s.charge_current_max_a.toFixed(1)+' A</span></div>';
      h+='<div class="row"><span class="k">Discharge current max</span><span class="vl">'+s.discharge_current_max_a.toFixed(1)+' A</span></div>';
      h+='<div class="row"><span class="k">Charge temp range</span><span class="vl">'+s.charge_low_t.toFixed(1)+' .. '+s.charge_high_t.toFixed(1)+' C</span></div>';
      h+='<div class="row"><span class="k">Discharge temp range</span><span class="vl">'+s.discharge_low_t.toFixed(1)+' .. '+s.discharge_high_t.toFixed(1)+' C</span></div>';
    }
    if(d.alarm&&d.alarm.ok){
      h+='<div class="sl">Alarm status (0x44)</div>';
      h+='<div class="row"><span class="k">Critical</span><span class="vl '+(d.alarm.critical?'er':'ok')+'">'+(d.alarm.critical?'YES':'No')+'</span></div>';
      h+='<div class="row"><span class="k">Status bits</span><span class="vl">0x'+d.alarm.status_hex.replace(/^0+/,'')+'</span></div>';
      if(d.alarm.active&&d.alarm.active.length>0){
        h+='<div class="alarm-list">';d.alarm.active.forEach(function(a){h+='<span>'+esc(a.name)+'</span>'});h+='</div>';
      }
    }
    if(d.historical&&d.historical.ok){
      var hi=d.historical;
      h+='<div class="sl">Last historical event (0x4B)</div>';
      h+='<div class="row"><span class="k">Date</span><span class="vl">'+hi.year+'-'+pad(hi.month)+'-'+pad(hi.day)+' '+pad(hi.hour)+':'+pad(hi.minute)+'</span></div>';
      h+='<div class="row"><span class="k">Event / Status</span><span class="vl">Type '+hi.event+' / 0x'+hi.status_hex.replace(/^0+/,'')+'</span></div>';
      h+='<div class="row"><span class="k">Pack / Current / Remain</span><span class="vl">'+hi.pack_v.toFixed(2)+' V / '+hi.current.toFixed(2)+' A / '+hi.remain_ah.toFixed(2)+' Ah</span></div>';
    }
    if(d.autocfg&&d.autocfg.ok&&d.autocfg.suggest){
      var sg=d.autocfg.suggest;
      h+='<div class="sl">Auto-config suggestion</div>';
      h+='<div class="row"><span class="k">Safe cell / pack</span><span class="vl">'+sg.s_cel.toFixed(3)+' V / '+sg.s_vol.toFixed(2)+' V</span></div>';
      h+='<div class="row"><span class="k">Charge / Discharge</span><span class="vl">'+sg.chg.toFixed(1)+' / '+sg.dis.toFixed(1)+' A</span></div>';
      h+='<div class="row"><span class="k">CVL</span><span class="vl">'+sg.cvl.toFixed(2)+' V</span></div>';
      h+='<div class="row"><span class="k">Temp charge</span><span class="vl">'+sg.t_c_min.toFixed(1)+' .. '+sg.t_c_max.toFixed(1)+' C</span></div>';
      h+='<div class="row"><span class="k">Temp discharge</span><span class="vl">'+sg.t_d_min.toFixed(1)+' .. '+sg.t_d_max.toFixed(1)+' C</span></div>';
      h+='<div style="margin-top:10px"><button class="bp" onclick="acf()">Apply to gateway</button></div>';
    }
    $('out').innerHTML=h;
  }).catch(function(e){$('rbtn').disabled=false;msg('Fetch error: '+e.message,'er')});
}
function st(){
  var id=$('sid').value;msg('Setting BMS '+id+' time...');
  fetch('/svc/bms/time/set?id='+id).then(function(r){return r.json()}).then(function(d){
    if(d.ok){msg('BMS '+id+' time synced','ok');rd()}else msg('Failed: '+(d.err||''),'er');
  }).catch(function(e){msg('Error: '+e.message,'er')});
}
function acf(){
  var id=$('sid').value;if(!confirm('Apply auto-config from BMS '+id+' to gateway? You must Save and Reboot afterwards.'))return;
  msg('Applying auto-config from BMS '+id+'...');
  fetch('/svc/autocfg/apply?id='+id).then(function(r){return r.json()}).then(function(d){
    if(d.ok){msg('Auto-config applied. Go to Battery tab and Save and Reboot to persist.','ok')}else msg('Failed: '+(d.err||''),'er');
  }).catch(function(e){msg('Error: '+e.message,'er')});
}
function pad(n){return String(n).padStart(2,'0')}
function esc(s){if(!s)return'';var d=document.createElement('div');d.textContent=s;return d.innerHTML}
</script></body></html>)SVCHTML";
  server.sendHeader("Cache-Control", "public, max-age=600");
  server.send(200, "text/html", h);
}

// ==========================================
// MAIN WEB UI (single-page app served as raw literal)
// Glassmorphism sidebar layout with 5 pages:
//   Dashboard (read-only), Battery, Network, General, Alerts.
// Sidebar navigation with SVG icons, frosted glass cards.
// Light + dark mode via CSS variables + localStorage toggle.
// Polls /data every 2.5s, charts render on <canvas>.
// Form fields use same name attributes as /save POST endpoint.
// Alert tracking uses localStorage for "new since last visit" badge.
// ==========================================
void handleRoot() {
  if (!checkAuth()) return;
  String h = R"RAWUI(
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1,viewport-fit=cover">
<title>TopBand BMS Gateway</title>
<style>
/* ============================================================
   TOPBAND BMS GATEWAY V2.63 - Glassmorphism Sidebar UI
   Light + Dark mode, responsive sidebar, frosted glass cards.
   ============================================================ */

/* --- DESIGN TOKENS: LIGHT MODE --- */
:root{
--bg:#f0f2f5;--bg2:#ffffff;--bg3:#f8f9fb;
--tx:#0f172a;--tx2:#64748b;--tx3:#94a3b8;
--bd:rgba(0,0,0,0.07);--bd2:rgba(0,0,0,0.15);
--acc:#059669;--acc2:#047857;--accbg:#ecfdf5;--acctx:#065f46;
--red:#ef4444;--redbg:#fef2f2;--redtx:#991b1b;--redbd:#fca5a5;
--amber:#f59e0b;--amberbg:#fffbeb;--ambertx:#92400e;--amberbd:#fcd34d;
--blue:#3b82f6;--bluebg:#eff6ff;--bluetx:#1e40af;
--glass:rgba(255,255,255,0.75);--blur:blur(14px);
--sh:0 1px 4px rgba(0,0,0,0.05);--sh2:0 6px 20px rgba(0,0,0,0.07);
--nav:#0e1726;--navtx:rgba(255,255,255,0.55);--navhi:#fff;
--r:10px;--r2:14px;--mono:ui-monospace,'SF Mono',Consolas,monospace}

/* --- DESIGN TOKENS: DARK MODE --- */
@media(prefers-color-scheme:dark){:root:not(.lt){
--bg:#080e1a;--bg2:rgba(15,23,42,0.65);--bg3:rgba(22,33,55,0.5);
--tx:#e2e8f0;--tx2:#94a3b8;--tx3:#64748b;
--bd:rgba(255,255,255,0.06);--bd2:rgba(255,255,255,0.12);
--glass:rgba(15,23,42,0.55);
--accbg:rgba(5,150,105,0.15);--acctx:#34d399;
--redbg:rgba(239,68,68,0.12);--redtx:#fca5a5;--redbd:rgba(239,68,68,0.35);
--amberbg:rgba(245,158,11,0.12);--ambertx:#fcd34d;--amberbd:rgba(245,158,11,0.35);
--bluebg:rgba(59,130,246,0.12);--bluetx:#93c5fd;
--sh:0 1px 4px rgba(0,0,0,0.3);--sh2:0 6px 24px rgba(0,0,0,0.3);
--nav:#060b14;--navtx:rgba(255,255,255,0.4)}}
:root.dk{
--bg:#080e1a;--bg2:rgba(15,23,42,0.65);--bg3:rgba(22,33,55,0.5);
--tx:#e2e8f0;--tx2:#94a3b8;--tx3:#64748b;
--bd:rgba(255,255,255,0.06);--bd2:rgba(255,255,255,0.12);
--glass:rgba(15,23,42,0.55);
--accbg:rgba(5,150,105,0.15);--acctx:#34d399;
--redbg:rgba(239,68,68,0.12);--redtx:#fca5a5;--redbd:rgba(239,68,68,0.35);
--amberbg:rgba(245,158,11,0.12);--ambertx:#fcd34d;--amberbd:rgba(245,158,11,0.35);
--bluebg:rgba(59,130,246,0.12);--bluetx:#93c5fd;
--sh:0 1px 4px rgba(0,0,0,0.3);--sh2:0 6px 24px rgba(0,0,0,0.3);
--nav:#060b14;--navtx:rgba(255,255,255,0.4)}

/* --- RESET + BASE --- */
*{box-sizing:border-box;margin:0;padding:0}
body{font-family:-apple-system,BlinkMacSystemFont,'Segoe UI',system-ui,sans-serif;background:var(--bg);color:var(--tx)}

/* --- APP SHELL: SIDEBAR + MAIN --- */
.app{display:grid;grid-template-columns:56px 1fr;min-height:100vh}
@media(max-width:768px){.app{grid-template-columns:1fr;grid-template-rows:auto 1fr}}

/* --- SIDEBAR (always dark) --- */
.nav{background:var(--nav);display:flex;flex-direction:column;align-items:center;padding:12px 0;gap:4px;border-right:1px solid rgba(255,255,255,0.06);z-index:10}
@media(max-width:768px){.nav{flex-direction:row;justify-content:space-around;order:1;padding:env(safe-area-inset-top,0) 8px 0;border-right:none;border-bottom:1px solid rgba(255,255,255,0.06);position:sticky;top:0}}
.nav .logo{width:32px;height:32px;margin-bottom:12px;opacity:0.7}
@media(max-width:768px){.nav .logo{display:none}}
.ni{display:flex;flex-direction:column;align-items:center;gap:2px;padding:10px 0;width:100%;cursor:pointer;color:var(--navtx);transition:color 0.15s;text-decoration:none;position:relative}
.ni svg{width:20px;height:20px;stroke:currentColor;fill:none;stroke-width:1.5;stroke-linecap:round;stroke-linejoin:round}
.ni span{font-size:8px;letter-spacing:0.04em;text-transform:uppercase;font-weight:500}
.ni:hover,.ni.on{color:var(--navhi)}
.ni.on::before{content:'';position:absolute;left:0;top:8px;bottom:8px;width:2px;background:var(--acc);border-radius:0 2px 2px 0}
@media(max-width:768px){.ni.on::before{left:8px;right:8px;top:0;bottom:auto;width:auto;height:2px;border-radius:0 0 2px 2px}}
.ni .badge{position:absolute;top:6px;right:10px;background:var(--red);color:#fff;font-size:8px;font-weight:700;min-width:14px;height:14px;border-radius:7px;display:flex;align-items:center;justify-content:center;padding:0 3px}
@media(max-width:768px){.ni .badge{top:4px;right:50%;transform:translateX(14px)}}

/* --- MAIN CONTENT --- */
.main{overflow-y:auto;max-height:100vh}
@media(max-width:768px){.main{max-height:none;overflow:visible}}

/* --- TOPBAR --- */
.top{display:flex;align-items:center;justify-content:space-between;padding:10px 20px;background:var(--glass);backdrop-filter:var(--blur);border-bottom:1px solid var(--bd);position:sticky;top:0;z-index:5;flex-wrap:wrap;gap:6px}
.top h1{font-size:13px;font-weight:600;letter-spacing:-0.01em}
.top .rt{display:flex;align-items:center;gap:8px}
.clock{font-size:11px;color:var(--tx3);font-family:var(--mono)}
.thm{background:var(--bg3);border:1px solid var(--bd);border-radius:6px;padding:4px 8px;cursor:pointer;font-size:12px;color:var(--tx2);line-height:1;transition:background 0.15s}
.thm:hover{background:var(--bd)}
.pls{display:flex;gap:3px}
.pl{font-size:9px;padding:2px 7px;border-radius:8px;font-weight:600;letter-spacing:0.02em}
.pl-g{background:var(--accbg);color:var(--acctx)}
.pl-y{background:var(--amberbg);color:var(--ambertx)}
.pl-r{background:var(--redbg);color:var(--redtx)}

/* --- PAGES --- */
.pg{display:none;padding:20px;max-width:1400px}
.pg.on{display:block}

/* --- ALERT BANNER --- */
.ab{display:none;align-items:center;gap:10px;padding:10px 14px;border-radius:var(--r);margin-bottom:14px;font-size:12px;background:var(--amberbg);color:var(--ambertx);border:1px solid var(--amberbd)}
.ab.err{background:var(--redbg);color:var(--redtx);border-color:var(--redbd)}

/* --- GLASS CARDS (metric cards, BMS cards, form panels) --- */
.mc{background:var(--glass);backdrop-filter:var(--blur);border-radius:var(--r);padding:14px 16px;border:1px solid var(--bd);box-shadow:var(--sh);transition:box-shadow 0.2s,transform 0.15s}
.mc:hover{box-shadow:var(--sh2);transform:translateY(-1px)}
.mc .l{font-size:9px;color:var(--tx3);text-transform:uppercase;letter-spacing:0.06em;margin-bottom:4px;font-weight:600}
.mc .n{font-size:22px;font-weight:600;letter-spacing:-0.02em}.mc .n.sm{font-size:15px;font-weight:500}
.mc .s{font-size:10px;color:var(--tx2);margin-top:3px}

/* --- METRIC GRIDS --- */
.mg{display:grid;grid-template-columns:repeat(auto-fit,minmax(170px,1fr));gap:10px;margin-bottom:14px}
.mg5{display:grid;grid-template-columns:repeat(auto-fit,minmax(145px,1fr));gap:10px;margin-bottom:14px}
@media(max-width:700px){.mg,.mg5{grid-template-columns:repeat(2,1fr)}.mc-w{grid-column:span 2}}

/* --- SECTION LABELS --- */
.sl{font-size:9px;color:var(--tx3);text-transform:uppercase;letter-spacing:0.08em;margin:18px 0 10px;padding-bottom:6px;border-bottom:1px solid var(--bd);font-weight:600}

/* --- CHART CONTAINERS --- */
.cp{background:var(--glass);backdrop-filter:var(--blur);border-radius:var(--r);height:clamp(160px,18vw,280px);position:relative;overflow:hidden;margin:6px 0 0;border:1px solid var(--bd)}
.cp canvas{width:100%!important;height:100%!important}

/* --- CHART + BMS GRIDS (always stretch to fill) --- */
.cg{display:grid;grid-template-columns:1fr 1fr;gap:10px;margin-bottom:14px}
@media(max-width:700px){.cg{grid-template-columns:1fr}}
.bg{display:grid;grid-template-columns:repeat(auto-fit,minmax(250px,1fr));gap:10px}
@media(max-width:600px){.bg{grid-template-columns:1fr}}
.bc{border:1px solid var(--bd);border-radius:var(--r);padding:14px 16px;border-left:3px solid var(--acc);background:var(--glass);backdrop-filter:var(--blur);box-shadow:var(--sh);transition:box-shadow 0.2s}
.bc:hover{box-shadow:var(--sh2)}
.bc.bcr{border-left-color:var(--red)}.bc.bcy{border-left-color:var(--amber)}.bc.bco{border-left-color:var(--tx3)}
.bc .hd{display:flex;justify-content:space-between;align-items:center;margin-bottom:8px}
.bc .hd .nm{font-weight:600;font-size:13px}
.bc .st{display:flex;gap:10px;font-size:11px;color:var(--tx2);flex-wrap:wrap}
.bc .st b{color:var(--tx);font-weight:600}
.cell-bar{display:flex;gap:2px;margin-top:8px;align-items:flex-end;height:clamp(36px,5vw,64px)}
.cell-bar .cb{flex:1;background:var(--acc);border-radius:3px 3px 0 0;min-width:0;transition:height 0.3s;opacity:0.8}
.cell-bar .cb.lo{background:var(--blue);opacity:1}.cell-bar .cb.hi{background:var(--red);opacity:1}
.cell-lbl{display:flex;justify-content:space-between;font-size:9px;color:var(--tx3);margin-top:3px}

/* --- EVENTS LIST --- */
.evl{max-height:160px;overflow-y:auto}
.evi{display:flex;justify-content:space-between;font-size:11px;padding:5px 0;border-bottom:1px solid var(--bd)}
.evi .ts{color:var(--tx3);font-family:var(--mono);white-space:nowrap;margin-left:8px;font-size:10px}

/* --- FORM ELEMENTS --- */
.fg{display:grid;grid-template-columns:1fr 1fr;gap:10px 14px}
@media(max-width:500px){.fg{grid-template-columns:1fr}}
.fg .fu{grid-column:1/-1}
.fg label{display:block;font-size:11px;color:var(--tx2);margin-bottom:4px;font-weight:500}
.fg input,.fg select{width:100%;padding:8px 10px;border:1px solid var(--bd2);border-radius:var(--r);background:var(--bg3);color:var(--tx);font-size:12px;outline:none;transition:border-color 0.15s}
.fg input:focus,.fg select:focus{border-color:var(--acc);box-shadow:0 0 0 3px var(--accbg)}
.fg input[readonly]{opacity:0.5}
.cr{display:flex;align-items:center;font-size:12px;padding:4px 0;cursor:pointer;margin-bottom:6px}
.cr input[type=checkbox]{width:16px;height:16px;accent-color:var(--acc);margin-right:10px;flex-shrink:0}

/* --- BUTTONS --- */
.bp{background:var(--acc);color:#fff;border:none;padding:10px 22px;border-radius:var(--r);font-size:12px;font-weight:600;cursor:pointer;transition:background 0.15s}
.bp:hover{background:var(--acc2)}
.bo{background:transparent;border:1px solid var(--bd2);padding:8px 16px;border-radius:var(--r);font-size:12px;cursor:pointer;color:var(--tx);transition:all 0.15s}
.bo:hover{background:var(--bg3);border-color:var(--acc)}

/* --- INFO ROWS / NOTES --- */
.ir{display:flex;justify-content:space-between;font-size:12px;padding:6px 0;border-bottom:1px solid var(--bd)}
.ir .k{color:var(--tx2)}.ir .vl{font-weight:600}
.nt{font-size:11px;color:var(--tx3);margin-top:6px}
.ib{font-size:11px;color:var(--tx2);background:var(--bg3);border:1px solid var(--bd);border-radius:var(--r);padding:10px 14px;margin-top:8px;line-height:1.6}
.bpr{background:var(--bg3);border-radius:var(--r);padding:12px 14px;margin-top:10px;border:1px solid var(--bd)}
.bpr .row{display:flex;justify-content:space-between;font-size:12px;padding:3px 0}
.bpr .row .k{color:var(--tx2)}.bpr .row .vl{font-weight:600}

/* --- BATTERY TAB ELEMENTS --- */
.bd{border:1px solid var(--bd);border-radius:var(--r);padding:14px;margin-bottom:12px;background:var(--glass);backdrop-filter:var(--blur)}
.bd.bdy{border-left:3px solid var(--amberbd)}.bd.bdr{border-left:3px solid var(--redbd)}
.bd .bh{display:flex;justify-content:space-between;align-items:center;margin-bottom:10px}
.bd .bh .nm{font-weight:600;font-size:14px}
.tg{display:grid;grid-template-columns:repeat(7,1fr);gap:6px;margin-top:8px}
@media(max-width:500px){.tg{grid-template-columns:repeat(4,1fr)}}
.tc{background:var(--bg3);border-radius:var(--r);padding:8px 6px;text-align:center;border:1px solid var(--bd);transition:all 0.15s}
.tc .tv{font-size:14px;font-weight:600;margin-bottom:2px}
.tc .tl{font-size:9px;color:var(--tx3);text-transform:uppercase;font-weight:500}
.tc.tok .tv{color:var(--acctx)}
.tc.twa{border-color:var(--amberbd);background:var(--amberbg)}.tc.twa .tv{color:var(--ambertx)}
.tc.ter{border-color:var(--redbd);background:var(--redbg)}.tc.ter .tv{color:var(--redtx)}

/* --- SAVE BAR (for config pages) --- */
.foot{padding:12px 20px;text-align:right;background:var(--glass);backdrop-filter:var(--blur);border-top:1px solid var(--bd);position:sticky;bottom:0;display:none}
.foot.vis{display:block}

/* --- ALERTS PAGE --- */
.al-item{display:flex;align-items:flex-start;gap:12px;padding:12px 14px;border:1px solid var(--bd);border-radius:var(--r);margin-bottom:8px;background:var(--glass);backdrop-filter:var(--blur);border-left:3px solid var(--tx3)}
.al-item.al-err{border-left-color:var(--red)}.al-item.al-warn{border-left-color:var(--amber)}.al-item.al-info{border-left-color:var(--acc)}
.al-item.al-new{box-shadow:var(--sh2)}
.al-item .al-t{font-size:12px;font-weight:500;flex:1}.al-item .al-ts{font-size:10px;color:var(--tx3);font-family:var(--mono);white-space:nowrap}
.al-item .al-d{font-size:11px;color:var(--tx2);margin-top:2px}
.al-empty{text-align:center;color:var(--tx3);padding:40px 0;font-size:13px}

/* --- DEBUG LOG --- */
.dlog{font-family:var(--mono);font-size:11px;max-height:120px;overflow-y:auto;background:var(--bg3);padding:8px;border-radius:var(--r);margin-top:10px;display:none;border:1px solid var(--bd)}
</style>
</head>
<body>
<div class="app">

<!-- ==========================================
     SIDEBAR NAVIGATION
     5 items: Dashboard, Battery, Network, General, Alerts
     Always dark background, icons + micro-labels.
     Active indicator: left accent bar (desktop), bottom bar (mobile).
     ========================================== -->
<nav class="nav">
<svg class="logo" viewBox="0 0 32 32" fill="none" stroke="rgba(255,255,255,0.5)" stroke-width="1.5"><rect x="6" y="4" width="20" height="24" rx="3"/><line x1="12" y1="1" x2="12" y2="4"/><line x1="20" y1="1" x2="20" y2="4"/><line x1="10" y1="12" x2="22" y2="12"/><line x1="10" y1="17" x2="22" y2="17"/><line x1="10" y1="22" x2="22" y2="22"/></svg>
<a class="ni on" data-p="d" onclick="nav(this)"><svg viewBox="0 0 24 24"><rect x="3" y="3" width="7" height="7" rx="1"/><rect x="14" y="3" width="7" height="4" rx="1"/><rect x="3" y="14" width="7" height="4" rx="1"/><rect x="14" y="11" width="7" height="7" rx="1"/></svg><span>Dash</span></a>
<a class="ni" data-p="b" onclick="nav(this)"><svg viewBox="0 0 24 24"><rect x="3" y="6" width="18" height="14" rx="2"/><line x1="7" y1="3" x2="7" y2="6"/><line x1="17" y1="3" x2="17" y2="6"/><line x1="7" y1="11" x2="11" y2="11"/><line x1="9" y1="9" x2="9" y2="13"/><line x1="13" y1="11" x2="17" y2="11"/></svg><span>Battery</span></a>
<a class="ni" data-p="n" onclick="nav(this)"><svg viewBox="0 0 24 24"><path d="M5 12.55a11 11 0 0 1 14.08 0"/><path d="M1.42 9a16 16 0 0 1 21.16 0"/><path d="M8.53 16.11a6 6 0 0 1 6.95 0"/><circle cx="12" cy="20" r="1"/></svg><span>Network</span></a>
<a class="ni" data-p="g" onclick="nav(this)"><svg viewBox="0 0 24 24"><circle cx="12" cy="12" r="3"/><path d="M19.4 15a1.65 1.65 0 0 0 .33 1.82l.06.06a2 2 0 1 1-2.83 2.83l-.06-.06a1.65 1.65 0 0 0-1.82-.33 1.65 1.65 0 0 0-1 1.51V21a2 2 0 0 1-4 0v-.09A1.65 1.65 0 0 0 9 19.4a1.65 1.65 0 0 0-1.82.33l-.06.06a2 2 0 1 1-2.83-2.83l.06-.06A1.65 1.65 0 0 0 4.68 15a1.65 1.65 0 0 0-1.51-1H3a2 2 0 0 1 0-4h.09A1.65 1.65 0 0 0 4.6 9a1.65 1.65 0 0 0-.33-1.82l-.06-.06a2 2 0 1 1 2.83-2.83l.06.06A1.65 1.65 0 0 0 9 4.68a1.65 1.65 0 0 0 1-1.51V3a2 2 0 0 1 4 0v.09a1.65 1.65 0 0 0 1 1.51 1.65 1.65 0 0 0 1.82-.33l.06-.06a2 2 0 1 1 2.83 2.83l-.06.06A1.65 1.65 0 0 0 19.4 9a1.65 1.65 0 0 0 1.51 1H21a2 2 0 0 1 0 4h-.09a1.65 1.65 0 0 0-1.51 1z"/></svg><span>General</span></a>
<a class="ni" data-p="a" onclick="nav(this)"><svg viewBox="0 0 24 24"><path d="M18 8A6 6 0 0 0 6 8c0 7-3 9-3 9h18s-3-2-3-9"/><path d="M13.73 21a2 2 0 0 1-3.46 0"/></svg><span>Alerts</span><span class="badge" id="abadge" style="display:none">0</span></a>
</nav>

<!-- ==========================================
     MAIN CONTENT AREA
     ========================================== -->
<div class="main">

<!-- TOPBAR: status pills, alert indicator, user, theme, clock -->
<div class="top">
<h1>TopBand BMS Gateway</h1>
<div class="rt">
<div class="pls"><span class="pl pl-g" id="pb">BMS 0x</span><span class="pl pl-g" id="pc">CAN</span><span class="pl pl-g" id="pm">MQTT</span><span class="pl pl-g" id="pw">WiFi</span></div>
<span class="clock" id="hck">--:--</span>
<span id="hauth" style="display:none;font-size:11px;color:var(--tx2)"><span id="hauser"></span> <a href="/logout" style="color:var(--acc);text-decoration:none">Logout</a></span>
<button class="thm" id="thm" onclick="tgT()" title="Toggle theme">&#9790;</button>
</div>
</div>

<!-- FORM wraps all config pages (Battery, Network, General) -->
<form id="sf" action="/save" method="POST"><input type="hidden" name="exp" value="1"><input type="hidden" name="theme" value="0">

<!-- ==========================================
     PAGE: DASHBOARD (read-only live data)
     ========================================== -->
<div class="pg on" id="p-d">
<div class="ab" id="dab"><span>&#9888;</span><span id="dam"></span></div>
<div class="mg">
<div class="mc mc-w"><div class="l">SOC</div><div class="n" id="ds" style="color:var(--acc)">--%</div><div class="s" id="dss">-- / -- Ah</div></div>
<div class="mc"><div class="l">Power</div><div class="n" id="dp">-- W</div><div class="s" id="dst">Idle</div></div>
<div class="mc"><div class="l">Current</div><div class="n" id="dc">-- A</div><div class="s" id="dcs">Limit: --</div></div>
<div class="mc mc-w"><div class="l">Voltage</div><div class="n" id="dv">-- V</div><div class="s" id="dvs">CVL: --</div></div>
<div class="mc mc-w"><div class="l">Energy today</div><div class="n sm" id="dei" style="color:var(--acc)">-- kWh</div><div class="s" id="deo">Out: -- kWh</div></div>
</div>
<div class="mg5">
<div class="mc"><div class="l">Cell min</div><div class="n sm" id="dcn">-- V</div><div class="s" id="dcni">--</div></div>
<div class="mc"><div class="l">Cell max</div><div class="n sm" id="dcx">-- V</div><div class="s" id="dcxi">--</div></div>
<div class="mc mc-w"><div class="l">Cell drift</div><div class="n sm" id="dcd">-- V</div><div class="s" id="dcds">Limit: --</div></div>
<div class="mc"><div class="l">Temperature</div><div class="n sm" id="dtp">-- C</div><div class="s" id="dtps">SOH: --%</div></div>
<div class="mc"><div class="l">Runtime est.</div><div class="n sm" id="drt">--</div><div class="s" id="drts">&nbsp;</div></div>
</div>
<div class="cg">
<div><div style="margin-bottom:4px"><span style="font-size:11px;color:var(--tx3)" id="ch1lbl">Power (48h)</span></div><div class="cp"><canvas id="ch1"></canvas></div></div>
<div><div style="margin-bottom:4px"><span style="font-size:11px;color:var(--tx3)" id="ch2lbl">Voltage (48h)</span></div><div class="cp"><canvas id="ch2"></canvas></div></div>
</div>
<div class="sl">BMS packs</div>
<div class="bg" id="bcon"></div>
</div>

<!-- ==========================================
     PAGE: BATTERY (config + diagnostics)
     ========================================== -->
<div class="pg" id="p-b">
<div class="sl">Battery configuration</div>
<div class="fg"><div><label>BMS count</label><input type="number" name="cnt" id="i_cnt" min="1" max="16"></div><div><label>Cells per BMS (0 = auto)</label><input type="number" name="cells" id="i_cells" min="0" max="32"></div><div><label>SOC source</label><select name="soc_mode" id="i_soc_mode"><option value="2">Hybrid (recommended)</option><option value="1">Raw BMS</option><option value="0">Calculated</option></select></div></div>
<div class="sl" style="margin-top:16px">Temperature sensors</div>
<div id="btmp"><div style="color:var(--tx3);font-size:12px">Waiting for data...</div></div>
<div class="sl" style="margin-top:16px">BMS reported limits (read-only)</div>
<div id="bpro" class="bpr"><div style="color:var(--tx3);font-size:12px">Loading...</div></div>
<div class="nt">Hardware limits from BMS protocol 0x47.</div>
<div style="margin-top:10px"><button type="button" class="bo" onclick="$('svcSec').style.display=$('svcSec').style.display==='none'?'block':'none'">BMS diagnostics</button></div>
<div id="svcSec" style="display:none;margin-top:12px;border:1px solid var(--bd);border-radius:var(--r);padding:16px;background:var(--glass);backdrop-filter:var(--blur)">
<div style="font-size:13px;font-weight:500;margin-bottom:8px">BMS Diagnostics</div>
<div style="display:flex;gap:8px;align-items:center;flex-wrap:wrap;margin-bottom:10px">
<select id="svcId" style="padding:7px 9px;border:1px solid var(--bd2);border-radius:var(--r);background:var(--bg3);color:var(--tx);font-size:12px"></select>
<button type="button" class="bp" id="svcRdBtn" onclick="svcRd()">Read diagnostics</button>
<button type="button" class="bo" onclick="svcTm()">Sync time</button>
</div>
<div id="svcMsg" class="nt" style="display:none"></div>
<div id="svcOut"></div>
</div>
<div class="sl" style="margin-top:16px">Gateway limits</div>
<div style="margin-bottom:12px"><button type="button" class="bo" id="acbtn" onclick="acBatt()">Auto-configure from BMS</button><span id="acst" class="nt" style="margin-left:10px"></span></div>
<div class="fg">
<div><label>Charge current (A per pack)</label><input type="number" step="0.1" name="chg" id="i_chg"></div>
<div><label>Discharge current (A per pack)</label><input type="number" step="0.1" name="dis" id="i_dis"></div>
<div><label>Charge voltage CVL (V)</label><input type="number" step="0.01" name="cvl" id="i_cvl"></div>
<div><label>Max pack voltage (V)</label><input type="number" step="0.01" name="s_vol" id="i_svol"></div>
<div><label>Max cell voltage (V)</label><input type="number" step="0.01" name="s_cel" id="i_scel"></div>
<div><label>Max cell drift (V)</label><input type="number" step="0.01" name="s_drift" id="i_sdrift"></div>
</div>
<div class="sl" style="margin-top:16px">Temperature cutoff (C)</div>
<div class="fg">
<div><label>Charge min</label><input type="number" step="0.1" name="t_c_min" id="i_tcmin"></div>
<div><label>Charge max</label><input type="number" step="0.1" name="t_c_max" id="i_tcmax"></div>
<div><label>Discharge min</label><input type="number" step="0.1" name="t_d_min" id="i_tdmin"></div>
<div><label>Discharge max</label><input type="number" step="0.1" name="t_d_max" id="i_tdmax"></div>
<div><label>Evaluation mode</label><select name="t_mode" id="i_tmode"><option value="0">Hottest sensor (safe)</option><option value="1">Average</option></select></div>
</div>
<div class="sl" style="margin-top:16px">Charge management</div>
<div class="fg">
<div class="fu"><label class="cr"><input type="checkbox" name="maint_en" id="i_maint_en"><span>Maintenance charge mode</span></label></div>
<div><label>Maintenance target (V)</label><input type="number" step="0.1" name="maint_v" id="i_maint_v"></div>
<div class="fu"><label class="cr"><input type="checkbox" name="ab_en" id="i_ab_en"><span>Auto balance watchdog (30 days)</span></label></div>
</div>
<div class="sl" style="margin-top:16px">Energy</div>
<div class="mg5">
<div class="mc"><div class="l">Today in</div><div class="n sm" id="bet_i" style="color:var(--acc)">--</div><div class="s" id="bet_o">Out: --</div></div>
<div class="mc"><div class="l">Week (7d) in</div><div class="n sm" id="bew_i">--</div><div class="s" id="bew_o">Out: --</div></div>
<div class="mc"><div class="l">Month in</div><div class="n sm" id="bem_i">--</div><div class="s" id="bem_o">Out: --</div></div>
</div>
<div style="font-size:10px;color:var(--tx3);margin-top:2px" id="bedays"></div>
</div>

<!-- ==========================================
     PAGE: NETWORK (WiFi, CAN, MQTT, HA)
     ========================================== -->
<div class="pg" id="p-n">
<div class="sl">WiFi</div>
<div class="fg" style="grid-template-columns:1fr 1fr 1fr"><div><label>IP address</label><div style="font-size:13px;font-weight:500;padding:7px 0" id="nip">--</div></div><div><label>Signal</label><div style="font-size:13px;font-weight:500;padding:7px 0" id="nrs">--</div></div><div><label>Hostname</label><div style="font-size:13px;font-weight:500;padding:7px 0" id="nhn">--</div></div></div>
<div class="sl" style="margin-top:16px">CAN-Bus</div>
<div class="fg"><div class="fu"><label class="cr"><input type="checkbox" name="vic_en" id="i_vic_en"><span>Enable CAN-Bus</span></label></div><div><label>Protocol</label><select name="can_proto" id="i_can_proto"><option value="0">Standard (Victron)</option><option value="1">Pylontech Strict</option><option value="2">SMA Sunny Island</option></select></div></div>
<div class="sl" style="margin-top:16px">MQTT</div>
<div class="fg">
<div class="fu"><label class="cr"><input type="checkbox" name="mq_en" id="i_mq_en"><span>Enable MQTT</span></label></div>
<div><label>Broker IP</label><input type="text" name="mq_ip" id="i_mq_ip" placeholder="192.168.1.x"></div>
<div><label>Port</label><input type="number" name="mq_pt" id="i_mq_pt"></div>
<div><label>User</label><input type="text" name="mq_us" id="i_mq_us"></div>
<div><label>Password</label><input type="password" name="mq_pw" id="i_mq_pw" placeholder="(unchanged)"></div>
<div><label>Topic</label><input type="text" name="mq_top" id="i_mq_top" placeholder="Topband/BMS"></div>
<div><label>Device ID</label><input type="text" id="nuid" readonly></div>
<div class="fu"><label class="cr"><input type="checkbox" name="mq_full" id="i_mq_full"><span>Include per-BMS cell data</span></label></div>
</div>
<div class="sl" style="margin-top:16px">Home Assistant</div>
<div class="fg"><div class="fu"><label class="cr"><input type="checkbox" name="ha_en" id="i_ha_en"><span>Auto-discovery</span></label></div><div class="fu" style="display:flex;gap:8px"><button type="button" class="bo" onclick="haS()">Send discovery</button><button type="button" class="bo" onclick="haC()">Clear discovery</button></div></div>
<div id="has" class="nt"></div>
</div>

<!-- ==========================================
     PAGE: GENERAL (preferences, security, device info, OTA)
     ========================================== -->
<div class="pg" id="p-g">
<div class="sl">Hardware</div>
<div class="fg">
<div><label>Board</label><select name="board_type" id="i_board" onchange="tgPins()"><option value="0">Waveshare ESP32-S3</option><option value="1">LilyGo T-CAN485</option><option value="2">Custom (set pins manually)</option></select></div>
</div>
<div id="pinFields" style="display:none;margin-top:8px">
<div class="fg" style="grid-template-columns:repeat(3,1fr)">
<div><label>RS485 TX</label><input type="number" name="p_rs_tx" id="i_p_rs_tx" min="0" max="48"></div>
<div><label>RS485 RX</label><input type="number" name="p_rs_rx" id="i_p_rs_rx" min="0" max="48"></div>
<div><label>RS485 DIR (-1=none)</label><input type="number" name="p_rs_dir" id="i_p_rs_dir" min="-1" max="48"></div>
<div><label>CAN TX</label><input type="number" name="p_can_tx" id="i_p_can_tx" min="0" max="48"></div>
<div><label>CAN RX</label><input type="number" name="p_can_rx" id="i_p_can_rx" min="0" max="48"></div>
<div><label>LED (-1=none)</label><input type="number" name="p_led" id="i_p_led" min="-1" max="48"></div>
</div>
</div>
<div id="pinInfo" class="ib" style="margin-top:8px"></div>
<div class="sl" style="margin-top:16px">Preferences</div>
<div class="fg"><div class="fu" style="display:flex;gap:14px"><div style="flex:1"><label>Chart left</label><select name="chart1" id="i_ch1" onchange="svCh();chDirty=true;upd()"><option value="0">Power</option><option value="1">Voltage</option><option value="2">SOC</option><option value="3">Temperature</option></select></div><div style="flex:1"><label>Chart right</label><select name="chart2" id="i_ch2" onchange="svCh();chDirty=true;upd()"><option value="0">Power</option><option value="1">Voltage</option><option value="2">SOC</option><option value="3">Temperature</option></select></div></div>
<div class="ib" style="grid-column:1/-1">The dashboard shows two charts covering the last 48 hours. A new data point is recorded every 3 minutes, capturing the average, minimum, and maximum values over that interval. The shaded area behind the line shows the min/max range. Chart data is saved to flash memory every 15 minutes and survives reboots and firmware updates. Chart selection is saved instantly and applies to all devices.</div>
<div class="fu" style="display:flex;gap:14px"><div style="flex:1"><label>NTP server</label><input type="text" name="ntp_svr" id="i_ntp"></div><div style="flex:1;max-width:140px"><label>Timezone (GMT+)</label><input type="number" name="tz_off" id="i_tz"></div></div></div>
<div class="sl" style="margin-top:16px">Security</div>
<div class="fg">
<div class="fu"><label class="cr"><input type="checkbox" name="auth_en" id="i_auth_en" onchange="tgAuth()"><span>Enable web authentication</span></label></div>
<div id="authFields" style="display:none;grid-column:1/-1"><div class="fg"><div><label>Username</label><input type="text" name="auth_user" id="i_auth_user" maxlength="32" autocomplete="username"></div><div><label>New password</label><input type="password" name="auth_pw" id="i_auth_pw" placeholder="(unchanged)" autocomplete="new-password"></div></div></div>
</div>
<div class="sl" style="margin-top:16px">Diagnostics</div>
<div class="fg"><div class="fu"><label class="cr"><input type="checkbox" name="debug" id="i_debug"><span>Serial debug output</span></label></div></div>
<div class="dlog" id="dlog"></div>
<div class="sl" style="margin-top:16px">Device</div>
<div class="ir"><span class="k">Firmware</span><span class="vl" id="gfw">--</span></div>
<div class="ir"><span class="k">Device ID</span><span class="vl" id="guid">--</span></div>
<div class="ir"><span class="k">Chip</span><span class="vl" id="gchip">--</span></div>
<div class="ir"><span class="k">Chip temperature</span><span class="vl" id="gctemp">--</span></div>
<div class="ir"><span class="k">Flash usage</span><span class="vl" id="gflash">--</span></div>
<div class="ir"><span class="k">Free heap</span><span class="vl" id="ghp">--</span></div>
<div class="ir"><span class="k">Uptime</span><span class="vl" id="gup">--</span></div>
<div class="ir"><span class="k">Last boot reason</span><span class="vl" id="gboot">--</span></div>
<div class="ir"><span class="k">GPIO pins</span><span class="vl" id="gpins">--</span></div>
<div class="sl" style="margin-top:16px">Maintenance</div>
<div style="display:flex;gap:10px;flex-wrap:wrap;margin-top:8px;align-items:center"><button type="button" class="bo" onclick="$('otaSec').style.display=$('otaSec').style.display==='none'?'block':'none'">Firmware update</button><a href="/backup" class="bo" style="text-decoration:none;display:inline-block">Download backup</a><button type="button" class="bo" id="rbtn" onclick="$('rfile').click()">Restore backup</button><input type="file" id="rfile" accept=".json" style="display:none" onchange="doRestore(this)"><a href="/export" class="bo" style="text-decoration:none;display:inline-block">Export history (CSV)</a></div>
<div id="rmsg" class="nt"></div>
<div id="otaSec" style="display:none;margin-top:12px;border:1px solid var(--bd);border-radius:var(--r);padding:16px;background:var(--glass);backdrop-filter:var(--blur)">
<div style="font-size:13px;font-weight:500;margin-bottom:8px">OTA Firmware Update</div>
<div id="otaDrop" style="border:1.5px dashed var(--bd2);border-radius:var(--r);padding:24px;text-align:center;cursor:pointer;color:var(--tx2);font-size:12px;transition:border-color 0.2s">
<input type="file" id="otaFile" accept=".bin" style="display:none"><label for="otaFile" style="cursor:pointer" id="otaLbl">Select .bin file or drag and drop</label></div>
<div id="otaProg" style="display:none;margin-top:10px"><div style="width:100%;height:4px;background:var(--bg3);border-radius:2px;overflow:hidden"><div id="otaBar" style="height:100%;background:var(--acc);width:0%;transition:width 0.3s;border-radius:2px"></div></div></div>
<div id="otaMsg" class="nt" style="margin-top:6px"></div>
<button type="button" class="bp" id="otaBtn" disabled style="margin-top:10px;opacity:0.5" onclick="otaUp()">Upload firmware</button>
<div class="nt">ESP32 reboots automatically. Settings are preserved.</div>
</div>
</div>

<!-- ==========================================
     PAGE: ALERTS (event history with "new" tracking)
     ========================================== -->
<div class="pg" id="p-a">
<div style="display:flex;justify-content:space-between;align-items:center"><div class="sl" style="margin:0;border:none;padding:0">System alerts</div><div style="display:flex;gap:8px;align-items:center"><span class="al-cnt" id="acnt" style="font-size:10px;color:var(--tx3)"></span><button type="button" class="bo" onclick="clearAlertsSrv()">Clear</button></div></div>
<div id="alertList" style="margin-top:10px"><div class="al-empty">No alerts</div></div>
</div>

</form>
<!-- Save bar: visible on Battery, Network, General pages -->
<div class="foot" id="sbar"><button type="submit" form="sf" class="bp">Save and reboot</button></div>
</div>
</div>

<!-- ==========================================
     JAVASCRIPT
     Navigation, theme, data polling, charts, alerts.
     All element IDs match backend /data and /save endpoints.
     ========================================== -->
<script>
/* --- GLOBALS --- */
var D=null,I=false,prL=false,chDirty=true,lastPwr=0;
var reqC1=0,reqC2=1;  // Track what chart types were actually requested (matches data)
var lastBmsRender=0;  // Throttle BMS card rebuilds (ms timestamp)
var alertLog=[];  // V2.61: populated from server /data D.alerts (NVS-backed ring buffer of last 25)
var lastAlertTs=parseInt(localStorage.getItem('tbLastAlert')||'0');
/* V2.61: POST /alerts/clear to wipe RAM + NVS on the gateway */
function clearAlertsSrv(){
if(!confirm('Clear all alerts? This will wipe the stored alert history on the device.'))return;
fetch('/alerts/clear',{method:'POST'}).then(function(r){return r.json()}).then(function(d){
if(d&&d.ok){alertLog=[];updAlertPage();updAlertBadge()}}).catch(function(){
/* fallback to GET if POST blocked */
fetch('/alerts/clear').then(function(){alertLog=[];updAlertPage();updAlertBadge()})})}

function $(i){return document.getElementById(i)}
function fV(v){return v!=null?v.toFixed(2):'--'}
function fV3(v){return v!=null?v.toFixed(3):'--'}
function fA(v){return v!=null?v.toFixed(1):'--'}
function tS(ts){if(!ts)return'--';var d=new Date(ts*1000);return d.getHours().toString().padStart(2,'0')+':'+d.getMinutes().toString().padStart(2,'0')}
function uT(ms){var s=Math.floor(ms/1000),m=Math.floor(s/60),h=Math.floor(m/60),d=Math.floor(h/24);if(d>0)return d+'d '+h%24+'h';if(h>0)return h+'h '+m%60+'m';return m+'m'}
function tCl(v,mn,mx){if(v==null)return'tok';if(v>=mx-3||v<=mn+2)return'ter';if(v>=mx-8||v<=mn+5)return'twa';return'tok'}
function pd(n){return String(n).padStart(2,'0')}
function esc(s){if(!s)return'';var d=document.createElement('div');d.textContent=s;return d.innerHTML}

/* --- SIDEBAR NAVIGATION --- */
function nav(el){
document.querySelectorAll('.ni').forEach(function(x){x.classList.remove('on')});
document.querySelectorAll('.pg').forEach(function(x){x.classList.remove('on')});
el.classList.add('on');
var pg=$('p-'+el.dataset.p);if(pg)pg.classList.add('on');
// Show save bar only on config pages
var isConfig=el.dataset.p!=='d'&&el.dataset.p!=='a';
$('sbar').classList.toggle('vis',isConfig);
// Mark alerts as seen when visiting alerts page
if(el.dataset.p==='a'){lastAlertTs=Math.floor(Date.now()/1000);localStorage.setItem('tbLastAlert',String(lastAlertTs));updAlertBadge()}
// Redraw charts when switching to dashboard
if(el.dataset.p==='d'){chDirty=true;drAll()}
}

/* --- THEME TOGGLE --- */
setInterval(function(){var d=new Date();$('hck').textContent=d.getHours().toString().padStart(2,'0')+':'+d.getMinutes().toString().padStart(2,'0')},5000);
var dk=localStorage.getItem('theme')==='dark'||(localStorage.getItem('theme')===null&&matchMedia('(prefers-color-scheme:dark)').matches);
function apT(){var r=document.documentElement;if(dk){r.classList.add('dk');r.classList.remove('lt')}else{r.classList.add('lt');r.classList.remove('dk')}$('thm').textContent=dk?'\u2600':'\u263E'}
function tgT(){dk=!dk;localStorage.setItem('theme',dk?'dark':'light');apT()}apT();
function tgAuth(){var on=$('i_auth_en').checked;$('authFields').style.display=on?'block':'none'}
function tgPins(){
var bt=parseInt($('i_board').value);
$('pinFields').style.display=bt===2?'block':'none';
var presets={0:{tx:17,rx:18,dir:21,ctx:15,crx:16,led:38,info:'Waveshare ESP32-S3: RS485 via DIR pin (GPIO 21)'},1:{tx:22,rx:21,dir:-1,ctx:27,crx:26,led:4,info:'LilyGo T-CAN485: RS485 via EN/CB pins, SD card slot'},2:{tx:0,rx:0,dir:-1,ctx:0,crx:0,led:-1,info:'Set GPIO pins for your board. DIR=-1 if your RS485 transceiver has auto-direction. LED=-1 to disable.'}};
var p=presets[bt]||presets[0];
if(bt<2){$('i_p_rs_tx').value=p.tx;$('i_p_rs_rx').value=p.rx;$('i_p_rs_dir').value=p.dir;$('i_p_can_tx').value=p.ctx;$('i_p_can_rx').value=p.crx;$('i_p_led').value=p.led}
$('pinInfo').textContent=p.info}
function svCh(){fetch('/chartcfg?c1='+$('i_ch1').value+'&c2='+$('i_ch2').value)}
window.onresize=function(){chDirty=true;drAll()};

/* --- ALERT TRACKING (V2.61: alertLog populated from server NVS ring buffer) --- */
/* Legacy no-op kept in case any extension script still calls it */
function addAlert(sev,msg,ts){return}
function updAlertBadge(){
var cnt=0;for(var i=0;i<alertLog.length;i++){if(alertLog[i].ts>lastAlertTs)cnt++}
var b=$('abadge');if(cnt>0){b.textContent=cnt;b.style.display='flex'}else{b.style.display='none'}
}
function updAlertPage(){
var c=$('alertList');if(!c)return;
var ac=$('acnt');if(ac){var ec=0,wc=0,ic=0;alertLog.forEach(function(a){if(a.sev==='err')ec++;else if(a.sev==='warn')wc++;else ic++});ac.textContent=alertLog.length>0?(ec+' errors, '+wc+' warnings, '+ic+' info'):''}
if(alertLog.length===0){c.innerHTML='<div class="al-empty">No alerts</div>';return}
var h='';alertLog.forEach(function(a){
var cls='al-item';if(a.sev==='err')cls+=' al-err';else if(a.sev==='warn')cls+=' al-warn';else cls+=' al-info';
if(a.ts>lastAlertTs)cls+=' al-new';
var tstr;
if(a.ts&&a.ts>1700000000){var d=new Date(a.ts*1000);tstr=pd(d.getHours())+':'+pd(d.getMinutes())+':'+pd(d.getSeconds())}
else if(a.up){/* V2.61: no NTP when raised, show as "Xh Ym uptime" */
var u=a.up;var hh=Math.floor(u/3600),mm=Math.floor((u%3600)/60);tstr=(hh>0?hh+'h '+mm+'m':mm+'m')+' up'}
else tstr='--';
h+='<div class="'+cls+'"><div style="flex:1"><div class="al-t">'+esc(a.msg)+'</div></div><span class="al-ts">'+tstr+'</span></div>'
});c.innerHTML=h
}

/* --- CHART RENDERING (V2.61: uses hstart instead of zero-filter heuristic) --- */
var CM=['Power','Voltage','SOC','Temperature'];var CU=['W','V','%','C'];var CC=['#059669','#3b82f6','#8b5cf6','#f59e0b'];
var CSC=[1,0.01,0.1,0.1];
function drAll(){if(!D||!chDirty)return;chDirty=false;
var c1=reqC1,c2=reqC2;
$('ch1lbl').textContent=CM[c1]+' (48h)';$('ch2lbl').textContent=CM[c2]+' (48h)';
var hs=D.hstart||0;
drLine('ch1',D.hist1,c1,D.h1mn,D.h1mx,hs);
drLine('ch2',D.hist2,c2,D.h2mn,D.h2mx,hs)}
function drLine(id,data,typ,dmin,dmax,skip){var c=$(id);if(!c||!data||data.length<2)return;
var w=c.parentElement.clientWidth,h=c.parentElement.clientHeight||180;if(w<10||h<10)return;
c.width=w;c.height=h;var ctx=c.getContext('2d');
var sc=CSC[typ]||1,col=CC[typ]||'#059669',unit=CU[typ]||'';
var pad=40,bt=24,tp=8;
skip=skip||0;if(skip>=data.length-1)skip=0;
/* V2.61: trim leading unfilled slots (reported by server via hstart) */
var src=skip>0?data.slice(skip):data;
var srcMn=(skip>0&&dmin)?dmin.slice(skip):dmin;
var srcMx=(skip>0&&dmax)?dmax.slice(skip):dmax;
if(src.length<2)return;
var pv=[],pmn=[],pmx=[];
for(var i=0;i<src.length;i++){pv.push(src[i]*sc);pmn.push(srcMn?srcMn[i]*sc:src[i]*sc);pmx.push(srcMx?srcMx[i]*sc:src[i]*sc)}
/* Y-axis range from all real points (no more zero-filter hack) */
var mn=Infinity,mx=-Infinity,hasData=false;
for(i=0;i<pv.length;i++){
var lo=pmn[i]<pv[i]?pmn[i]:pv[i];var hi2=pmx[i]>pv[i]?pmx[i]:pv[i];
if(lo<mn)mn=lo;if(hi2>mx)mx=hi2;hasData=true}
if(!hasData)return;
if(mn===mx){if(typ===2){mn=0;mx=100}else if(typ===1){mn=mn-2;mx=mx+2}else{mn=mn-50;mx=mx+50}}
var rng=mx-mn;if(rng<0.5)rng=0.5;mn-=rng*0.08;mx+=rng*0.08;rng=mx-mn;
var dh=h-bt-tp;
function gy(v){return tp+dh-((v-mn)/rng*dh)}
ctx.strokeStyle='rgba(128,128,128,0.08)';ctx.lineWidth=0.5;
for(var s=0;s<=3;s++){var v2=mn+(rng*(s/3)),y=gy(v2);ctx.beginPath();ctx.moveTo(pad,y);ctx.lineTo(w,y);ctx.stroke();
ctx.fillStyle='var(--tx3)';ctx.font='10px sans-serif';ctx.textAlign='right';
var lbl=(typ===1||typ===3)?v2.toFixed(1):Math.round(v2);ctx.fillText(lbl,pad-4,y+3)}
if(typ===0&&mn<0&&mx>0){var y0=gy(0);ctx.setLineDash([3,3]);ctx.strokeStyle='rgba(128,128,128,0.2)';ctx.beginPath();ctx.moveTo(pad,y0);ctx.lineTo(w,y0);ctx.stroke();ctx.setLineDash([])}
var sx=(w-pad)/(pv.length-1);
/* Min/max band */
if(srcMn&&srcMx){var cr2=parseInt(col.slice(1,3),16),cg=parseInt(col.slice(3,5),16),cb=parseInt(col.slice(5,7),16);
ctx.beginPath();for(i=0;i<pv.length;i++){var x=pad+i*sx;if(i===0)ctx.moveTo(x,gy(pmx[i]));else ctx.lineTo(x,gy(pmx[i]))}
for(i=pv.length-1;i>=0;i--){ctx.lineTo(pad+i*sx,gy(pmn[i]))}ctx.closePath();
ctx.fillStyle='rgba('+cr2+','+cg+','+cb+',0.06)';ctx.fill()}
/* Area fill */
var grd=ctx.createLinearGradient(0,tp,0,h-bt);
if(typ===0){grd.addColorStop(0,'rgba(5,150,105,0.08)');grd.addColorStop(0.5,'rgba(5,150,105,0.01)');grd.addColorStop(0.5,'rgba(217,119,6,0.01)');grd.addColorStop(1,'rgba(217,119,6,0.08)')}
else{var r2=parseInt(col.slice(1,3),16),g3=parseInt(col.slice(3,5),16),b3=parseInt(col.slice(5,7),16);grd.addColorStop(0,'rgba('+r2+','+g3+','+b3+',0.06)');grd.addColorStop(1,'rgba('+r2+','+g3+','+b3+',0.01)')}
var baseY=typ===0&&mn<0?gy(0):h-bt;
ctx.beginPath();ctx.moveTo(pad,baseY);ctx.lineTo(pad,gy(pv[0]));
for(i=1;i<pv.length;i++){ctx.lineTo(pad+i*sx,gy(pv[i]))}
ctx.lineTo(pad+(pv.length-1)*sx,baseY);ctx.fillStyle=grd;ctx.fill();
/* Main line */
ctx.beginPath();ctx.moveTo(pad,gy(pv[0]));for(i=1;i<pv.length;i++){ctx.lineTo(pad+i*sx,gy(pv[i]))}
ctx.strokeStyle=col;ctx.lineWidth=1.5;ctx.stroke();
/* Last point marker */
var lastIdx=pv.length-1;
var lx=pad+lastIdx*sx,ly=gy(pv[lastIdx]);
ctx.beginPath();ctx.arc(lx,ly,3,0,Math.PI*2);ctx.fillStyle=col;ctx.fill();
ctx.beginPath();ctx.arc(lx,ly,5,0,Math.PI*2);ctx.strokeStyle=col;ctx.lineWidth=1;ctx.globalAlpha=0.3;ctx.stroke();ctx.globalAlpha=1;
ctx.fillStyle=col;ctx.font='10px sans-serif';ctx.textAlign='right';
var nv=pv[lastIdx];ctx.fillText((typ===1||typ===3?nv.toFixed(1):Math.round(nv))+' '+unit,lx-8,ly-8);
/* Time axis labels (adjusted for trimmed length) */
ctx.fillStyle='rgba(128,128,128,0.4)';ctx.font='9px sans-serif';ctx.textAlign='center';
var ts2=D.ts||0;var minPer=15;
for(i=0;i<pv.length;i+=Math.max(1,Math.floor(pv.length/6))){var t=new Date((ts2-((pv.length-1-i)*minPer*60))*1000);ctx.fillText(String(t.getHours()).padStart(2,'0')+':00',pad+i*sx,h-4)}}

/* V2.61: ResizeObserver + delayed redraws fix mobile rendering race */
var _chartObs=null;
function initChartObserver(){if(_chartObs||!window.ResizeObserver)return;
_chartObs=new ResizeObserver(function(){chDirty=true;drAll()});
var cps=document.querySelectorAll('.cp');for(var i=0;i<cps.length;i++){_chartObs.observe(cps[i])}}

/* --- HELPER FUNCTIONS (HA, AutoCfg, Restore, OTA) --- */
function haS(){var s=$('has');s.textContent='Sending...';fetch('/ha/discovery/send').then(function(r){return r.json()}).then(function(d){s.textContent=d.ok?'Discovery sent':'Failed: '+(d.err||'');s.style.color=d.ok?'var(--acctx)':'var(--redtx)'}).catch(function(e){s.textContent='Error';s.style.color='var(--redtx)'})}
function haC(){var s=$('has');s.textContent='Clearing...';fetch('/ha/discovery/clear').then(function(r){return r.json()}).then(function(d){s.textContent=d.ok?'Cleared':'Failed: '+(d.err||'');s.style.color=d.ok?'var(--acctx)':'var(--redtx)'}).catch(function(e){s.textContent='Error';s.style.color='var(--redtx)'})}
function acBatt(){var s=$('acst');s.textContent='Reading BMS...';s.style.color='var(--tx2)';$('acbtn').disabled=true;
fetch('/svc/autocfg/apply?id=0').then(function(r){return r.json()}).then(function(d){$('acbtn').disabled=false;
if(!d.ok||!d.applied){s.textContent='Failed: '+(d.err||'');s.style.color='var(--redtx)';return}
var a=d.applied;
if(a.s_vol!=null)$('i_svol').value=a.s_vol;if(a.s_cel!=null)$('i_scel').value=a.s_cel;
if(a.chg!=null)$('i_chg').value=a.chg;if(a.dis!=null)$('i_dis').value=a.dis;
if(a.cvl!=null)$('i_cvl').value=a.cvl;
if(a.t_c_min!=null)$('i_tcmin').value=a.t_c_min;if(a.t_c_max!=null)$('i_tcmax').value=a.t_c_max;
if(a.t_d_min!=null)$('i_tdmin').value=a.t_d_min;if(a.t_d_max!=null)$('i_tdmax').value=a.t_d_max;
s.textContent='Applied. Save and Reboot to persist.';s.style.color='var(--acctx)'}).catch(function(e){$('acbtn').disabled=false;s.textContent='Error';s.style.color='var(--redtx)'})}
function doRestore(inp){if(!inp.files||!inp.files[0])return;var f=inp.files[0];var rm=$('rmsg');rm.textContent='Reading '+f.name+'...';rm.style.color='var(--tx2)';
var rd=new FileReader();rd.onload=function(){try{var j=JSON.parse(rd.result);if(!j.cnt){rm.textContent='Invalid backup file';rm.style.color='var(--redtx)';return}
if(!confirm('Restore settings from this backup? Device will reboot.'))return;rm.textContent='Restoring...';
fetch('/restore',{method:'POST',headers:{'Content-Type':'application/json'},body:rd.result}).then(function(r){return r.json()}).then(function(d){if(d.ok){rm.textContent='Restored! Rebooting...';rm.style.color='var(--acctx)';setTimeout(function(){location.reload()},5000)}else{rm.textContent='Failed: '+(d.err||'');rm.style.color='var(--redtx)'}}).catch(function(e){rm.textContent='Rebooting...';rm.style.color='var(--acctx)';setTimeout(function(){location.reload()},5000)})}catch(e){rm.textContent='Invalid JSON file';rm.style.color='var(--redtx)'}};rd.readAsText(f);inp.value=''}
var otaData=null;
(function(){var dp=$('otaDrop'),fi=$('otaFile');if(!dp||!fi)return;dp.ondragover=function(e){e.preventDefault();dp.style.borderColor='var(--acc)'};dp.ondragleave=function(){dp.style.borderColor='var(--bd2)'};dp.ondrop=function(e){e.preventDefault();dp.style.borderColor='var(--bd2)';if(e.dataTransfer.files.length)otaPick(e.dataTransfer.files[0])};fi.onchange=function(){if(fi.files.length)otaPick(fi.files[0])}})();
function otaPick(f){if(!f.name.endsWith('.bin')){$('otaMsg').textContent='Only .bin files';$('otaMsg').style.color='var(--redtx)';return}otaData=f;$('otaLbl').textContent=f.name+' ('+Math.round(f.size/1024)+' KB)';$('otaBtn').disabled=false;$('otaBtn').style.opacity='1';$('otaMsg').textContent='Ready';$('otaMsg').style.color='var(--acctx)'}
function otaUp(){if(!otaData)return;var xhr=new XMLHttpRequest();var fd=new FormData();fd.append('update',otaData);$('otaBtn').disabled=true;$('otaProg').style.display='block';$('otaMsg').textContent='Uploading...';$('otaMsg').style.color='var(--tx2)';var done=false;xhr.upload.onprogress=function(e){if(e.lengthComputable){var p=Math.round(e.loaded/e.total*100);$('otaBar').style.width=p+'%';$('otaMsg').textContent='Upload: '+p+'%';if(p>=100)done=true}};xhr.onload=function(){$('otaMsg').textContent='Update successful! Rebooting...';$('otaMsg').style.color='var(--acctx)';pollReboot()};xhr.onerror=function(){if(done){$('otaMsg').textContent='Update successful! Rebooting...';$('otaMsg').style.color='var(--acctx)';pollReboot()}else{$('otaMsg').textContent='Connection error';$('otaMsg').style.color='var(--redtx)';$('otaBtn').disabled=false}};xhr.open('POST','/update');xhr.send(fd)}
function pollReboot(){var n=0;var iv=setInterval(function(){n++;$('otaMsg').textContent='Waiting for reboot... ('+n+'s)';fetch('/data',{signal:AbortSignal.timeout(2000)}).then(function(){clearInterval(iv);location.href='/'}).catch(function(){})},3000)}

/* --- BMS DIAGNOSTICS --- */
function svcMsg(t,c){var e=$('svcMsg');e.style.display='block';e.textContent=t;e.style.color=c||'var(--tx2)'}
function svcRd(){var id=$('svcId').value;svcMsg('Reading BMS '+id+'...');$('svcRdBtn').disabled=true;
fetch('/svc/bms?id='+id).then(function(r){return r.json()}).then(function(d){$('svcRdBtn').disabled=false;if(!d.ok){svcMsg('Error: '+(d.err||''),'var(--redtx)');return}svcMsg('BMS '+id+' loaded','var(--acctx)');
var h='';
if(d.manufacturer&&d.manufacturer.ok){h+='<div class="sl">Manufacturer</div>';h+='<div class="ir"><span class="k">Hardware</span><span class="vl">'+esc(d.manufacturer.hw)+'</span></div>';h+='<div class="ir"><span class="k">Software</span><span class="vl">'+esc(d.manufacturer.sw)+'</span></div>';h+='<div class="ir"><span class="k">ID</span><span class="vl">'+esc(d.manufacturer.id)+'</span></div>'}
if(d.date&&d.date.ok){h+='<div class="sl">BMS clock</div>';h+='<div class="ir"><span class="k">Date/Time</span><span class="vl">'+d.date.year+'-'+pd(d.date.month)+'-'+pd(d.date.day)+' '+pd(d.date.hour)+':'+pd(d.date.minute)+':'+pd(d.date.second)+'</span></div>'}
if(d.analog&&d.analog.ok){var a=d.analog;h+='<div class="sl">Analog (0x42)</div>';h+='<div class="ir"><span class="k">Pack / Current</span><span class="vl">'+a.pack_v.toFixed(2)+' V / '+a.current.toFixed(2)+' A</span></div>';h+='<div class="ir"><span class="k">SOC / SOH</span><span class="vl">'+a.soc+'% / '+a.soh+'%</span></div>';h+='<div class="ir"><span class="k">Capacity</span><span class="vl">'+a.rem_ah.toFixed(2)+' / '+a.full_ah.toFixed(2)+' Ah</span></div>';h+='<div class="ir"><span class="k">Cell range</span><span class="vl">'+a.min_cell.toFixed(3)+' .. '+a.max_cell.toFixed(3)+' V</span></div>'}
if(d.system&&d.system.ok){var s=d.system;h+='<div class="sl">System params (0x47)</div>';h+='<div class="ir"><span class="k">Cell H/L/U</span><span class="vl">'+s.cell_high_v.toFixed(3)+' / '+s.cell_low_v.toFixed(3)+' / '+s.cell_under_v.toFixed(3)+' V</span></div>';h+='<div class="ir"><span class="k">Module H/L/U</span><span class="vl">'+s.module_high_v.toFixed(2)+' / '+s.module_low_v.toFixed(2)+' / '+s.module_under_v.toFixed(2)+' V</span></div>';h+='<div class="ir"><span class="k">Charge/Disch max</span><span class="vl">'+s.charge_current_max_a.toFixed(1)+' / '+s.discharge_current_max_a.toFixed(1)+' A</span></div>';h+='<div class="ir"><span class="k">Charge temp</span><span class="vl">'+s.charge_low_t.toFixed(1)+' .. '+s.charge_high_t.toFixed(1)+' C</span></div>';h+='<div class="ir"><span class="k">Discharge temp</span><span class="vl">'+s.discharge_low_t.toFixed(1)+' .. '+s.discharge_high_t.toFixed(1)+' C</span></div>'}
if(d.alarm&&d.alarm.ok){h+='<div class="sl">Alarms (0x44)</div>';h+='<div class="ir"><span class="k">Critical</span><span class="vl" style="color:'+(d.alarm.critical?'var(--redtx)':'var(--acctx)')+'">'+(d.alarm.critical?'YES':'No')+'</span></div>';if(d.alarm.active&&d.alarm.active.length>0){h+='<div style="margin-top:6px;display:flex;flex-wrap:wrap;gap:4px">';d.alarm.active.forEach(function(a){h+='<span style="font-size:11px;background:var(--bg3);padding:2px 8px;border-radius:4px">'+esc(a.name)+'</span>'});h+='</div>'}}
if(d.autocfg&&d.autocfg.ok&&d.autocfg.suggest){var sg=d.autocfg.suggest;h+='<div class="sl">Auto-config suggestion</div>';h+='<div class="ir"><span class="k">Safe cell/pack</span><span class="vl">'+sg.s_cel.toFixed(3)+' / '+sg.s_vol.toFixed(2)+' V</span></div>';h+='<div class="ir"><span class="k">Chg/Dis</span><span class="vl">'+sg.chg.toFixed(1)+' / '+sg.dis.toFixed(1)+' A, CVL '+sg.cvl.toFixed(2)+' V</span></div>';h+='<div style="margin-top:8px"><button type="button" class="bp" onclick="svcAcf()">Apply to gateway</button></div>'}
$('svcOut').innerHTML=h}).catch(function(e){$('svcRdBtn').disabled=false;svcMsg('Error: '+e.message,'var(--redtx)')})}
function svcTm(){var id=$('svcId').value;svcMsg('Syncing BMS '+id+' time...');fetch('/svc/bms/time/set?id='+id).then(function(r){return r.json()}).then(function(d){if(d.ok){svcMsg('Time synced','var(--acctx)');svcRd()}else svcMsg('Failed','var(--redtx)')}).catch(function(e){svcMsg('Error','var(--redtx)')})}
function svcAcf(){var id=$('svcId').value;if(!confirm('Apply auto-config from BMS '+id+'?'))return;svcMsg('Applying...');fetch('/svc/autocfg/apply?id='+id).then(function(r){return r.json()}).then(function(d){if(d.ok&&d.applied){var a=d.applied;if(a.s_vol!=null)$('i_svol').value=a.s_vol;if(a.s_cel!=null)$('i_scel').value=a.s_cel;if(a.chg!=null)$('i_chg').value=a.chg;if(a.dis!=null)$('i_dis').value=a.dis;if(a.cvl!=null)$('i_cvl').value=a.cvl;if(a.t_c_min!=null)$('i_tcmin').value=a.t_c_min;if(a.t_c_max!=null)$('i_tcmax').value=a.t_c_max;if(a.t_d_min!=null)$('i_tdmin').value=a.t_d_min;if(a.t_d_max!=null)$('i_tdmax').value=a.t_d_max;svcMsg('Applied. Save and Reboot to persist.','var(--acctx)')}else svcMsg('Failed','var(--redtx)')}).catch(function(e){svcMsg('Error','var(--redtx)')})}

/* --- MAIN DATA UPDATE (polls /data every 2.5s) --- */
function upd(){var s1=$('i_ch1'),s2=$('i_ch2');var q='';var rc1=0,rc2=1;if(I&&s1&&s2){rc1=parseInt(s1.value)||0;rc2=parseInt(s2.value)||1;q='?c1='+rc1+'&c2='+rc2}fetch('/data'+q).then(function(r){if(r.status===401){location.href='/login';return null}return r.json()}).then(function(d){
if(I){reqC1=rc1;reqC2=rc2}else if(d&&d.config){reqC1=d.config.chart1!=null?d.config.chart1:0;reqC2=d.config.chart2!=null?d.config.chart2:1}
if(!d)return;
D=d;var v=d.victron||{},c=d.config||{},bs=d.bms||[];
$('pb').textContent='BMS '+(v.active||0)+'x';$('pb').className='pl '+(v.active>0?'pl-g':'pl-r');
$('pc').className='pl '+(c.vic_en?(d.can_error?'pl-r':'pl-g'):'pl-y');$('pc').textContent=c.vic_en?(d.can_error?'CAN!':'CAN'):'CAN off';
var mqf=d.mqtt_fails||0;$('pm').className='pl '+(c.mq_en?(c.mq_connected?'pl-g':(mqf>=5?'pl-r':'pl-y')):'pl-y');$('pm').textContent=c.mq_en?(c.mq_connected?'MQTT':(mqf>0?'MQTT x'+mqf:'MQTT...')):'MQTT off';
var wf=d.wifi_rssi;$('pw').className='pl '+(wf!=null?(wf>-70?'pl-g':(wf>-80?'pl-y':'pl-r')):'pl-y');$('pw').textContent='WiFi';
/* --- Alert banner + alert log --- */
var ab=$('dab'),am='',ae=false;
if(d.safe_lock){am=d.alarm&&d.alarm!=='OK'?d.alarm:'Safety cutoff active';ae=true}
else if(d.alarm&&d.alarm!=='OK'&&d.alarm.indexOf('INFO')<0){am=d.alarm;if(d.alarm.indexOf('ALARM')>=0)ae=true}
else{bs.forEach(function(bm){if(!bm.online||!bm.temps)return;bm.temps.forEach(function(t){if(t.val>=(c.t_c_max||50)-5&&!am)am='BMS'+bm.id+' / '+t.lbl+': '+t.val.toFixed(1)+' C approaching charge max ('+(c.t_c_max||50)+' C)'})})}
if(am){ab.style.display='flex';ab.className=ae?'ab err':'ab';$('dam').innerHTML=am}else{ab.style.display='none'}
/* V2.61: alert log now authoritative from server NVS ring buffer (D.alerts, newest first).
   Server-side detection runs on Core 1 (alertDetectTick), rising-edge based to avoid spam. */
if(d.alerts&&d.alerts.length!==undefined){
alertLog=d.alerts.map(function(a){var s=a.sev===2?'err':(a.sev===1?'warn':'info');
return{sev:s,msg:a.msg,ts:a.ts||0,up:a.up||0}});
updAlertBadge();updAlertPage()}
/* --- Dashboard metrics --- */
$('ds').textContent=(v.avg_soc!=null?v.avg_soc.toFixed(1):'--')+'%';
$('dss').textContent=Math.round(v.rem_cap||0)+' / '+Math.round(v.total_cap||0)+' Ah';
var pw=v.total_power||0;$('dp').textContent=Math.round(pw)+' W';
if(Math.abs(pw-lastPwr)>5){chDirty=true;lastPwr=pw}
var stT='Idle',stC='var(--tx2)';if(pw>10){stT='Charging';stC='var(--acc)'}else if(pw<-10){stT='Discharging';stC='var(--ambertx)'}
$('dst').textContent=stT;$('dst').style.color=stC;
$('dc').textContent=fA(v.total_amps)+' A';$('dcs').textContent='Limit: '+fA(v.max_chg)+' A';
$('dv').textContent=fV(v.voltage)+' V';$('dvs').textContent='CVL: '+fV(v.dyn_cvl||c.cvl||0)+' V';
if(d.energy){var ei=d.energy.in||0,eo=d.energy.out||0;$('dei').textContent=(ei<10?ei.toFixed(2):ei.toFixed(1))+' kWh';$('deo').textContent='Out: '+(eo<10?eo.toFixed(2):eo.toFixed(1))+' kWh';
/* --- Energy details (Battery tab) --- */
$('bet_i').textContent=ei.toFixed(2)+' kWh';$('bet_o').textContent='Out: '+eo.toFixed(2)+' kWh';
if(d.energy.week_in!=null){$('bew_i').textContent=d.energy.week_in.toFixed(1)+' kWh';$('bew_o').textContent='Out: '+d.energy.week_out.toFixed(1)+' kWh'}
if(d.energy.month_in!=null){$('bem_i').textContent=d.energy.month_in.toFixed(1)+' kWh';$('bem_o').textContent='Out: '+d.energy.month_out.toFixed(1)+' kWh'}
if(d.energy.days_in){var dh='Last 7 days (in): ';d.energy.days_in.forEach(function(v,i){if(i)dh+=' | ';dh+=v.toFixed(1)});$('bedays').textContent=dh}
}
/* --- Cell extremes --- */
var mnV=Infinity,mxV=-Infinity,mnI='',mxI='',drft=0;
bs.forEach(function(bm){if(!bm.online)return;if(bm.min_cell<mnV){mnV=bm.min_cell;mnI='BMS'+bm.id}if(bm.max_cell>mxV){mxV=bm.max_cell;mxI='BMS'+bm.id}});
drft=mxV-mnV;
$('dcn').textContent=mnV<99?fV3(mnV):'--';$('dcni').textContent=mnI||'--';
$('dcx').textContent=mxV>-99?fV3(mxV):'--';$('dcxi').textContent=mxI||'--';
$('dcd').textContent=drft>0?fV3(drft):'--';$('dcds').textContent='Limit: '+fV3(c.s_drift);
$('dtp').textContent=fA(v.avg_temp)+' C';$('dtps').textContent='SOH: '+fA(v.avg_soh)+'%';
/* --- Runtime estimate --- */
var rt='--';if(v.total_cap>0&&v.total_amps){var hrs=0;if(v.total_amps<-0.2)hrs=v.rem_cap/Math.abs(v.total_amps);else if(v.total_amps>0.2)hrs=(v.total_cap-v.rem_cap)/v.total_amps;if(hrs>0&&hrs<999){rt=hrs>=1?Math.floor(hrs)+'h '+Math.round((hrs%1)*60)+'m':Math.round(hrs*60)+'m';$('drts').textContent=v.total_amps<-0.2?'until empty':'until full'}else $('drts').innerHTML='&nbsp;'}$('drt').textContent=rt;
/* --- BMS cards (throttled to every 15s to reduce visual noise) --- */
var bmsNow=Date.now();
if(bmsNow-lastBmsRender>=15000||lastBmsRender===0){lastBmsRender=bmsNow;
var con=$('bcon');var bH='';
bs.forEach(function(bm,bi){
var on=bm.online;var pc2='pl-g',brd='',st2='Idle';
if(!on){pc2='pl-r';brd=' bco';st2='Offline'}else if(bm.current>0.15){pc2='pl-g';st2='Charging'}else if(bm.current<-0.15){pc2='pl-y';brd=' bcy';st2='Discharging'}
bH+='<div class="bc'+brd+'">';
bH+='<div class="hd"><span class="nm">BMS #'+bm.id+'</span><span class="pl '+pc2+'">'+st2+'</span></div>';
if(on){
bH+='<div class="st"><span><b>'+bm.soc+'%</b></span><span>'+bm.pack_v.toFixed(2)+' V</span><span>'+bm.current.toFixed(1)+' A</span><span>'+Math.round(bm.pack_v*bm.current)+' W</span><span style="opacity:0.6">SOH '+bm.soh+'%</span></div>';
if(bm.stats){var sr=bm.stats.polls>0?Math.round(bm.stats.ok/bm.stats.polls*100):0;var sc=sr>=98?'var(--acctx)':sr>=90?'var(--ambertx)':'var(--redtx)';bH+='<div style="font-size:9px;color:var(--tx3);margin-top:4px">RS485: '+sr+'% ok <span style="opacity:0.6">('+bm.stats.ok+'/'+bm.stats.polls+')</span>'+(bm.stats.timeout>0?' <span style="color:var(--ambertx)">'+bm.stats.timeout+' tmout</span>':'')+(bm.stats.spikes>0?' <span style="color:var(--redtx)">'+bm.stats.spikes+' spike</span>':'')+'</div>'}
if(bm.cells&&bm.cells.length>1){var cl=bm.cells,ba=99,ta=-99,li=-1,xi=-1;for(var ci=0;ci<cl.length;ci++){if(cl[ci]<ba){ba=cl[ci];li=ci}if(cl[ci]>ta){ta=cl[ci];xi=ci}}var sp=ta-ba;if(sp<0.001)sp=0.001;
bH+='<div class="cell-bar">';for(ci=0;ci<cl.length;ci++){var pt=Math.max(8,Math.min(100,((cl[ci]-ba)/sp)*100));var cs='';if(ci===li)cs=' lo';else if(ci===xi)cs=' hi';bH+='<div class="cb'+cs+'" style="height:'+pt+'%"></div>'}
bH+='</div><div class="cell-lbl"><span>'+fV3(ba)+' V</span><span>'+cl.length+' cells</span><span>'+fV3(ta)+' V</span></div>'}
}bH+='</div>'});con.innerHTML=bH;
}
/* --- Events list --- */
/* Events moved to Alerts page */
/* --- Temperature grid (Battery tab) --- */
var tC=$('btmp');if(tC){var tH='';
bs.forEach(function(bm){if(!bm.online||!bm.temps)return;
var hw2=false;bm.temps.forEach(function(t){if(tCl(t.val,c.t_c_min||5,c.t_c_max||50)!=='tok')hw2=true});
tH+='<div class="bd'+(hw2?' bdy':'')+'"><div class="bh"><span class="nm">BMS #'+bm.id+'</span><span style="font-size:11px;color:var(--tx3);margin:0 8px">'+(bm.cycles||0)+' cycles</span><span class="pl '+(hw2?'pl-y':'pl-g')+'">'+(hw2?'Warning':'All OK')+'</span></div><div class="tg">';
bm.temps.forEach(function(t){tH+='<div class="tc '+tCl(t.val,c.t_c_min||5,c.t_c_max||50)+'"><div class="tv">'+t.val.toFixed(1)+'</div><div class="tl">'+t.lbl+'</div></div>'});
tH+='</div><div style="display:flex;justify-content:space-between;font-size:11px;color:var(--tx3);margin-top:6px"><span>Charge: '+(c.t_c_min||5)+' .. '+(c.t_c_max||50)+' C</span><span>Discharge: '+(c.t_d_min||-20)+' .. '+(c.t_d_max||60)+' C</span></div></div>'});
if(!tH)tH='<div style="color:var(--tx3);font-size:12px">Waiting for data...</div>';tC.innerHTML=tH}
/* --- Device info (General tab) --- */
$('gfw').textContent='V'+(c.fw||'--');$('guid').textContent=c.dev_uid||'--';
var um=d.uptime_ms||0;$('gup').textContent=uT(um);
if(d.boot_reason){var br=d.boot_reason;$('gboot').textContent=br;$('gboot').style.color=(br==='PANIC'||br==='INT_WDT'||br==='TASK_WDT'||br==='BROWNOUT')?'var(--redtx)':'var(--acctx)'}
if(c.pins){var p=c.pins;var bn=['Waveshare','LilyGo','Custom'];$('gpins').textContent=(bn[c.board_type]||'?')+': RS485 '+p.rs_tx+'/'+p.rs_rx+(p.rs_dir>=0?' DIR '+p.rs_dir:'')+', CAN '+p.can_tx+'/'+p.can_rx+(p.led>=0?', LED '+p.led:'')}
if(d.free_heap)$('ghp').textContent=Math.round(d.free_heap/1024)+' KB (min: '+Math.round((d.min_heap||0)/1024)+' KB)';
if(d.chip_temp!=null){var ct=d.chip_temp;$('gctemp').textContent=ct.toFixed(1)+' C';$('gctemp').style.color=ct>75?'var(--redtx)':ct>55?'var(--ambertx)':'var(--acctx)'}
if(d.chip_model){$('gchip').textContent=d.chip_model+' Rev '+d.chip_rev+' / '+(d.chip_cores||2)+' cores / '+(d.cpu_mhz||240)+' MHz'}
if(d.sketch_size&&d.flash_size){var pct=Math.round(d.sketch_size/d.flash_size*100);$('gflash').textContent=Math.round(d.sketch_size/1024)+' / '+Math.round(d.flash_size/1024)+' KB ('+pct+'%)'}
$('nuid').value=c.dev_uid||'--';
if(d.wifi_ip)$('nip').textContent=d.wifi_ip;
if(d.wifi_rssi!=null){var rs=d.wifi_rssi;var ql=rs>-50?'excellent':rs>-65?'good':rs>-75?'fair':'weak';$('nrs').textContent=rs+' dBm ('+ql+')';$('nrs').style.color=rs>-65?'var(--acctx)':rs>-75?'var(--ambertx)':'var(--redtx)'}
if(d.wifi_host)$('nhn').textContent=d.wifi_host;
if(d.log){var dl=$('dlog');dl.style.display='block';dl.innerHTML=d.log}
/* --- Populate form fields (once) --- */
if(!I&&c){
$('i_cnt').value=c.cnt;$('i_cells').value=c.cells;$('i_chg').value=c.chg;$('i_dis').value=c.dis;$('i_cvl').value=c.cvl;
$('i_svol').value=c.s_vol;$('i_scel').value=c.s_cel;$('i_sdrift').value=c.s_drift;
$('i_tcmin').value=c.t_c_min;$('i_tcmax').value=c.t_c_max;$('i_tdmin').value=c.t_d_min;$('i_tdmax').value=c.t_d_max;$('i_tmode').value=c.t_mode;
$('i_maint_en').checked=!!c.maint_en;$('i_maint_v').value=c.maint_v;$('i_ab_en').checked=c.ab_en!=null?!!c.ab_en:true;
$('i_vic_en').checked=!!c.vic_en;$('i_can_proto').value=c.can_proto;
$('i_mq_en').checked=!!c.mq_en;$('i_mq_full').checked=!!c.mq_full;$('i_mq_ip').value=c.mq_ip||'';$('i_mq_pt').value=c.mq_pt;
$('i_mq_us').value=c.mq_us||'';$('i_mq_top').value=c.mq_top||'Topband/BMS';$('i_ha_en').checked=!!c.ha_en;
$('i_soc_mode').value=c.soc_mode!=null?String(c.soc_mode):'2';$('i_ntp').value=c.ntp||'pool.ntp.org';$('i_tz').value=c.tz;$('i_debug').checked=!!c.debug;
$('i_board').value=c.board_type!=null?String(c.board_type):'0';
if(c.pins){$('i_p_rs_tx').value=c.pins.rs_tx;$('i_p_rs_rx').value=c.pins.rs_rx;$('i_p_rs_dir').value=c.pins.rs_dir;$('i_p_can_tx').value=c.pins.can_tx;$('i_p_can_rx').value=c.pins.can_rx;$('i_p_led').value=c.pins.led}
tgPins();
$('i_ch1').value=c.chart1!=null?String(c.chart1):'0';$('i_ch2').value=c.chart2!=null?String(c.chart2):'1';
$('i_auth_en').checked=!!c.auth_en;$('i_auth_user').value=c.auth_user||'admin';tgAuth();
if(c.auth_en){$('hauth').style.display='inline';$('hauser').textContent=c.auth_user||'admin'}else{$('hauth').style.display='none'}
var sel=$('svcId');if(sel&&sel.options.length===0){for(var si=0;si<c.cnt;si++){var o=document.createElement('option');o.value=si;o.textContent='BMS #'+si;sel.appendChild(o)}}
I=true;
if(!prL){prL=true;fetch('/svc/bms?id=0').then(function(r){return r.json()}).then(function(sd){
var bp=$('bpro');if(!sd.ok||!sd.system||!sd.system.ok){bp.innerHTML='<div style="color:var(--tx3);font-size:12px">Could not read BMS parameters</div>';return}
var s=sd.system;var h='';
h+='<div class="row"><span class="k">Module high / under voltage</span><span class="vl">'+s.module_high_v.toFixed(2)+' / '+s.module_under_v.toFixed(2)+' V</span></div>';
h+='<div class="row"><span class="k">Cell high voltage</span><span class="vl">'+s.cell_high_v.toFixed(3)+' V</span></div>';
h+='<div class="row"><span class="k">Charge / discharge max</span><span class="vl">'+s.charge_current_max_a.toFixed(1)+' / '+s.discharge_current_max_a.toFixed(1)+' A</span></div>';
h+='<div class="row"><span class="k">Charge temp range</span><span class="vl">'+s.charge_low_t.toFixed(1)+' .. '+s.charge_high_t.toFixed(1)+' C</span></div>';
h+='<div class="row"><span class="k">Discharge temp range</span><span class="vl">'+s.discharge_low_t.toFixed(1)+' .. '+s.discharge_high_t.toFixed(1)+' C</span></div>';
bp.innerHTML=h}).catch(function(){$('bpro').innerHTML='<div style="color:var(--tx3);font-size:12px">No data yet</div>'})}}
/* --- Sync chart dropdowns from server config (cross-device sync) --- */
if(c){var sCh1=c.chart1!=null?String(c.chart1):'0',sCh2=c.chart2!=null?String(c.chart2):'1';
if($('i_ch1').value!==sCh1||$('i_ch2').value!==sCh2){$('i_ch1').value=sCh1;$('i_ch2').value=sCh2;reqC1=parseInt(sCh1);reqC2=parseInt(sCh2);chDirty=true}}
drAll()}).catch(function(e){console.error('Fetch:',e)})}
setInterval(upd,2500);upd();
/* V2.61: ensure charts render on mobile even if initial layout hasn't settled */
initChartObserver();
setTimeout(function(){chDirty=true;drAll()},300);
setTimeout(function(){chDirty=true;drAll()},1500);
</script>
</body>
</html>
)RAWUI";
  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "text/html", h);
}


#endif  // END OF MODE_SMART_WIFI

// ==========================================
// RS485 + CAN TASK (runs on Core 0)
// Handles all time-critical BMS communication:
//   - Data polling: every 3s, all BMS packs sequentially
//   - Alarm polling: every 15s, round-robin one BMS per cycle
//   - System param polling: every 30s, round-robin one BMS per cycle
//   - CAN output: every 500ms, calculateVictronData + sendVictronCAN
// Stack: 16KB. WDT subscribed. Uses rs485Mutex for bus access,
// dataMutex for shared data writes.
// ==========================================
void rs485Task(void *param) {
  esp_task_wdt_add(NULL);
  unsigned long task_poll_time = 0;
  unsigned long task_can_time = 0;
  unsigned long task_alarm_time = 0;
  unsigned long task_sysparam_time = 0;
  int task_alarm_bms = 0;
  int task_sysparam_bms = 0;

  for (;;) {
    esp_task_wdt_reset();
    unsigned long now = millis();

    // BMS data polling
    if (!simulation_active && (now - task_poll_time > POLL_INTERVAL)) {
      task_poll_time = now;
      for (int i = 0; i < g_bms_count; i++) {
        esp_task_wdt_reset();
        if (xSemaphoreTake(rs485Mutex, pdMS_TO_TICKS(2000))) {
          rs485SetTX();
          RS485.flush(); while (RS485.available()) RS485.read();
          RS485.print(POLL_CMDS[i]); RS485.flush();
          rs485SetRX();
          g_bms_stats[i].polls++;
          unsigned long start = millis(); String resp = ""; resp.reserve(512);
          while (millis() - start < RS485_TIMEOUT) {
            if (RS485.available()) { char c = RS485.read(); if (c == '\r') break; if (c != '~') resp += c; }
            else vTaskDelay(1);
          }
          xSemaphoreGive(rs485Mutex);
          if (resp.length() > 0) parseTopband(resp, i);
          else g_bms_stats[i].timeouts++;
        }
        vTaskDelay(pdMS_TO_TICKS(BUS_GUARD_TIME));
      }
    }

    // Alarm polling (round-robin, one BMS per cycle)
    if (!simulation_active && (now - task_alarm_time > ALARM_POLL_INTERVAL)) {
      task_alarm_time = now;
      pollBmsAlarmStatus(task_alarm_bms);
      task_alarm_bms++;
      if (task_alarm_bms >= g_bms_count) task_alarm_bms = 0;
    }

    // System parameter polling (round-robin)
    if (!simulation_active && (now - task_sysparam_time > SYSPARAM_POLL_INTERVAL)) {
      task_sysparam_time = now;
      pollBmsSystemParameter(task_sysparam_bms);
      task_sysparam_bms++;
      if (task_sysparam_bms >= g_bms_count) task_sysparam_bms = 0;
    }

    // Calculate derived data + send CAN
    if (now - task_can_time > 500) {
      task_can_time = now;
      if (dataMutex) xSemaphoreTake(dataMutex, pdMS_TO_TICKS(200));
      calculateVictronData();
      if (dataMutex) xSemaphoreGive(dataMutex);
      if (g_victron_enable) { sendVictronCAN(); }
    }

    // Stack watermark check every 5 min
    static unsigned long last_stack_check = 0;
    if (now - last_stack_check > 300000) {
      last_stack_check = now;
      UBaseType_t hwm = uxTaskGetStackHighWaterMark(NULL);
      if (hwm < 1024) addToLog("WARN: RS485 task stack low: " + String(hwm) + " bytes free", true);
    }

    vTaskDelay(pdMS_TO_TICKS(50));
  }
}

// ==========================================
// SETUP
// Boot sequence: Reboot reason logging -> Hardware init -> Auth reset check
// (5x power-cycle) -> WiFi (fixed SSID or WiFiManager captive portal) ->
// Load NVS settings -> Init services (history, SD, MQTT) -> Register web
// routes -> Start CAN + mDNS -> Create rs485Task on Core 0.
// ==========================================
void setup() {
  Serial.begin(115200); 
  
  // Log reboot reason for diagnostics
  esp_reset_reason_t rst_reason = esp_reset_reason();
  const char* rst_names[] = {"UNKNOWN","POWERON","EXT","SW","PANIC","INT_WDT","TASK_WDT","WDT","DEEPSLEEP","BROWNOUT","SDIO"};
  g_boot_reason = (rst_reason < 11) ? rst_names[rst_reason] : "OTHER";
  Serial.println("Boot reason: " + g_boot_reason);
  
  // Load pin configuration from NVS (must happen before hardware init)
  {
    Preferences pinCfg;
    pinCfg.begin("gateway", true);
    g_board_type = pinCfg.getInt("board_type", 0);
    applyBoardPreset(g_board_type);
    // Custom overrides (only applied if board_type=2 or if values exist in NVS)
    if (g_board_type == 2 || pinCfg.isKey("p_rs_tx")) {
      g_pin_rs485_tx = pinCfg.getInt("p_rs_tx", g_pin_rs485_tx);
      g_pin_rs485_rx = pinCfg.getInt("p_rs_rx", g_pin_rs485_rx);
      g_pin_rs485_dir = pinCfg.getInt("p_rs_dir", g_pin_rs485_dir);
      g_pin_can_tx = pinCfg.getInt("p_can_tx", g_pin_can_tx);
      g_pin_can_rx = pinCfg.getInt("p_can_rx", g_pin_can_rx);
      g_pin_led = pinCfg.getInt("p_led", g_pin_led);
    }
    pinCfg.end();
    Serial.printf("Board type %d: RS485 TX=%d RX=%d DIR=%d, CAN TX=%d RX=%d, LED=%d\n",
                  g_board_type, g_pin_rs485_tx, g_pin_rs485_rx, g_pin_rs485_dir,
                  g_pin_can_tx, g_pin_can_rx, g_pin_led);
  }

  // Hardware init with runtime pin config
  if (g_board_type == 1) {
    // LilyGo: 3 RS485 control pins
    pinMode(g_pin_lilygo_pwr, OUTPUT); digitalWrite(g_pin_lilygo_pwr, HIGH);
    pinMode(g_pin_lilygo_en, OUTPUT); digitalWrite(g_pin_lilygo_en, HIGH);
    pinMode(g_pin_lilygo_cb, OUTPUT); digitalWrite(g_pin_lilygo_cb, HIGH);
  } else {
    // Waveshare / Custom: single direction pin
    if (g_pin_rs485_dir >= 0) { pinMode(g_pin_rs485_dir, OUTPUT); digitalWrite(g_pin_rs485_dir, LOW); }
  }

  pixels.setPin(g_pin_led);
  pixels.begin(); pixels.setBrightness(30); 
  setLed(0,0,255); // Blue = boot
  delay(500); 

  // Rapid power-cycle auth reset: 5x power cycle within 15s
  {
    Preferences rst;
    rst.begin("gateway", false);
    uint8_t cnt = rst.getUChar("rst_cnt", 0);
    cnt++;
    if (cnt >= 5) {
      // Reset triggered - clear auth credentials
      rst.putBool("auth_en", false);
      rst.putString("auth_hash", "");
      rst.putUChar("rst_cnt", 0);
      rst.end();
      Serial.println("AUTH RESET: 5x power cycle detected, credentials cleared");
      for (int i = 0; i < 6; i++) { setLed(255, 0, 0); delay(150); setLed(0, 0, 0); delay(150); }
      setLed(0, 0, 255);
    } else {
      rst.putUChar("rst_cnt", cnt);
      rst.end();
    }
  }

  #ifdef MODE_SMART_WIFI
    // 1. WiFi Manager
    WiFiManager wm;
    wm.setAPCallback(configModeCallback); 
    wm.setConnectTimeout(15);              // 15s statt 30s - gespeicherte Credentials brauchen max 10s
    wm.setConfigPortalTimeout(120);        // 2 min AP-Portal statt 3 min

    // Set hostname before WiFi connect
    WiFi.setHostname(HOSTNAME); 

    bool fixed_wifi_ok = false;
    if (String(FIXED_SSID) != "") {
        WiFi.mode(WIFI_STA);
        WiFi.setAutoReconnect(true);
        WiFi.persistent(false);
        WiFi.disconnect(true, true);
        delay(150);
        WiFi.begin(FIXED_SSID, FIXED_PASS);
        int tr = 0;
        while (WiFi.status() != WL_CONNECTED && tr < 80) {
            delay(500);
            tr++;
        }
        fixed_wifi_ok = (WiFi.status() == WL_CONNECTED);
    }

    setLed(150, 0, 150); // Purple = WiFi connecting

    if (!fixed_wifi_ok && !wm.autoConnect("Victron-Gateway-Setup")) {
        Serial.println("AP Timeout");
        setLed(255, 0, 0); // Red = AP timeout
    } else {
        if (MDNS.begin(HOSTNAME)) MDNS.addService("http", "tcp", 80);
        configTime(g_timezone_offset * 3600, 0, g_ntp_server.c_str());
        setLed(0, 255, 255); // Teal = WiFi connected
    }

    // Generate device UID from MAC address
    {
      uint8_t mac[6];
      WiFi.macAddress(mac);
      char uid[16];
      snprintf(uid, sizeof(uid), "tb_%02x%02x%02x", mac[3], mac[4], mac[5]);
      g_dev_uid = String(uid);
      addToLog("Device UID: " + g_dev_uid, false);
    }

    // Generate session token for cookie-based auth
    generateSessionToken();

    // 2. Load settings
    preferences.begin("gateway", false);
    g_bms_count = preferences.getInt("cnt", 2); 
    g_force_cell_count = preferences.getInt("cells", 0);
    g_charge_amps = preferences.getFloat("chg", 30.0);
    g_discharge_amps = preferences.getFloat("dis", 30.0);
    g_expert_mode = preferences.getBool("exp", true);
    if(g_expert_mode) g_cvl_voltage = preferences.getFloat("cvl", 52.5); else g_cvl_voltage = 52.5;
    g_safe_volt = preferences.getFloat("s_vol", 53.25);
    g_safe_cell = preferences.getFloat("s_cel", 3.55);
    // Default charge min temp is 5.0 C
    g_tc_min = preferences.getFloat("t_c_min", 5.0);
    g_tc_max = preferences.getFloat("t_c_max", 50.0);
    g_td_min = preferences.getFloat("t_d_min", -20.0);
    g_td_max = preferences.getFloat("t_d_max", 60.0);
    g_temp_mode = preferences.getInt("t_mode", 0);
    g_safe_drift = preferences.getFloat("s_drift", 0.20);
    // Config Flags
    g_sd_enable = preferences.getBool("sd_en", false); 
    g_serial_debug = preferences.getBool("debug", false); 
    g_sd_spy = preferences.getBool("spy", false); 
    g_mqtt_enable = preferences.getBool("mq_en", false);
    g_mqtt_full = preferences.getBool("mq_full", false); 
    g_ha_enable = preferences.getBool("ha_en", false); 
    g_theme_id = preferences.getInt("theme", 0);
    g_can_protocol = preferences.getInt("can_proto", 0); 
    g_victron_enable = preferences.getBool("vic_en", true); 
    g_soc_source_mode = preferences.getInt("soc_mode", 2);
    g_chart1 = preferences.getInt("chart1", 0);
    g_chart2 = preferences.getInt("chart2", 1);
    if (g_soc_source_mode < 0 || g_soc_source_mode > 2) g_soc_source_mode = 2;
    g_maint_charge_mode = preferences.getBool("maint_en", false);
    g_maint_target_v = preferences.getFloat("maint_v", 51.00f);
    if (g_maint_target_v < 40.0f) g_maint_target_v = 40.0f;
    if (g_maint_target_v > 65.0f) g_maint_target_v = 65.0f;
    g_auto_balance_enable = preferences.getBool("ab_en", true);
    g_auto_balance_last_ts = preferences.getULong("ab_last", 0);
    g_easy_mode = preferences.getBool("easy", false);
    g_setup_done = preferences.getBool("setupd", true);
    if (!g_expert_mode) {
    g_easy_mode = false;
    g_charge_amps = 30.0f;
    g_discharge_amps = 30.0f;
    g_cvl_voltage = 52.5f;
    g_safe_volt = 53.25f;
    g_safe_cell = 3.55f;
    g_safe_drift = 0.20f;
    g_tc_min = 5.0f;
    g_tc_max = 50.0f;
    g_td_min = -20.0f;
    g_td_max = 60.0f;
    g_maint_target_v = 51.0f;
  }
    g_mqtt_server = preferences.getString("mq_ip", ""); g_mqtt_port = preferences.getInt("mq_pt", 1883);
    g_mqtt_user = preferences.getString("mq_us", "");
    g_mqtt_pass = preferences.getString("mq_pw", "");
    g_mqtt_topic = preferences.getString("mq_top", "Topband/BMS");
    g_ntp_server = preferences.getString("ntp", "pool.ntp.org"); g_timezone_offset = preferences.getInt("tz", 1);
    g_auth_enable = preferences.getBool("auth_en", false);
    g_auth_user = preferences.getString("auth_user", "admin");
    g_auth_hash = preferences.getString("auth_hash", "");
    preferences.end();
    
    // 3. Init services
    addToLog("System start (boot: " + g_boot_reason + "), waiting for BMS...", false); 
    loadHistory(); 
    loadAlertsFromNvs();   // V2.61: restore alert ring buffer
    addToLog("Alerts loaded: " + String(g_alert_count) + " entries", false);
    initSD();
    if (g_mqtt_enable && g_mqtt_server != "") { mqtt.setServer(g_mqtt_server.c_str(), g_mqtt_port); mqtt.setBufferSize(4096); mqtt.setSocketTimeout(3); }

    // 4. Server routes
    const char* authHeaders[] = {"Cookie"};
    server.collectHeaders(authHeaders, 1);
    server.on("/login", handleLogin);
    server.on("/logout", handleLogout);
    server.on("/", handleRoot);
    server.on("/data", handleData);
    server.on("/sim", handleSim); server.on("/save", handleSave); server.on("/chartcfg", handleChartCfg);
    server.on("/svc/bms", HTTP_GET, handleServiceBmsDiag);
    server.on("/svc/bms/time", HTTP_GET, handleServiceBmsTimeGet);
    server.on("/svc/bms/time/set", HTTP_GET, handleServiceBmsTimeSet);
    server.on("/svc/autocfg/apply", HTTP_GET, handleServiceAutoCfgApply);
    server.on("/ha/discovery/send", HTTP_GET, handleHaDiscoverySend);
    server.on("/ha/discovery/clear", HTTP_GET, handleHaDiscoveryClear);
    server.on("/alerts", HTTP_GET, handleAlertsGet);
    server.on("/alerts/clear", HTTP_POST, handleAlertsClear);
    server.on("/alerts/clear", HTTP_GET, handleAlertsClear);  // GET fallback for simple clients
    #if HAS_SD_CARD
    server.on("/sd/download", HTTP_GET, handleSDDownload); 
    server.on("/sd/spy", HTTP_GET, handleSpyDownload); 
    server.on("/sd/clear", HTTP_GET, handleSDDelete); 
    server.on("/sd/list", HTTP_GET, handleSDList); 
    #endif
    server.on("/update", HTTP_GET, handleOTAPage);
    server.on("/service", HTTP_GET, handleServicePage);
    server.on("/backup", HTTP_GET, handleBackup);
    server.on("/export", HTTP_GET, handleExport);
    server.on("/restore", HTTP_POST, handleRestore);
    server.on("/update", HTTP_POST, handleOTADone, handleOTAUpload);
    server.begin();
    // mDNS: allows access via http://topband-gateway.local
    if (MDNS.begin(HOSTNAME)) {
      MDNS.addService("http", "tcp", 80);
      addToLog("mDNS: " + String(HOSTNAME) + ".local", false);
    }
    setLed(0,0,0); // LED off, enter loop
  #else
    // Cable mode (no WiFi)
    g_bms_count = 2; 
    g_can_protocol = 0;
  #endif

  RS485.begin(RS485_BAUD, SERIAL_8N1, g_pin_rs485_rx, g_pin_rs485_tx);
  #if ARDUINO_ESP32_MAJOR >= 2
  RS485.setRxBufferSize(1024);
  #endif
  
  twai_general_config_t g_cfg = TWAI_GENERAL_CONFIG_DEFAULT((gpio_num_t)g_pin_can_tx, (gpio_num_t)g_pin_can_rx, TWAI_MODE_NORMAL);
  twai_timing_config_t t_cfg = TWAI_TIMING_CONFIG_500KBITS();
  twai_filter_config_t f_cfg = TWAI_FILTER_CONFIG_ACCEPT_ALL();
  esp_err_t can_inst = twai_driver_install(&g_cfg, &t_cfg, &f_cfg);
  esp_err_t can_start = twai_start();
  if (can_inst != ESP_OK || can_start != ESP_OK) {
    debug_can_status = "Init failed - check hardware (GPIO " + String(g_pin_can_tx) + "/" + String(g_pin_can_rx) + ")";
    can_error_flag = true; addToLog("CAN Init Failed", true);
  } else {
    debug_can_status = "Ready - waiting for inverter";
    addToLog("CAN bus initialized", false);
  }
  
  #if ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0)
  esp_task_wdt_config_t wdt_cfg = { .timeout_ms = WDT_TIMEOUT * 1000, .idle_core_mask = (1 << 0) | (1 << 1), .trigger_panic = true };
  esp_task_wdt_init(&wdt_cfg);
  #else
  esp_task_wdt_init(WDT_TIMEOUT, true);
  #endif
  esp_task_wdt_add(NULL);

  // Create mutexes for thread-safe RS485 bus and data access
  rs485Mutex = xSemaphoreCreateMutex();
  dataMutex = xSemaphoreCreateMutex();

  // Launch RS485 + CAN task on Core 0 (WiFi core has idle time during polling waits)
  xTaskCreatePinnedToCore(rs485Task, "rs485", 16384, NULL, 1, &rs485TaskHandle, 0);
  addToLog("Dual-core: RS485 on Core 0, Web on Core 1", false);
  addToLog("Heap: " + String(ESP.getFreeHeap()) + " bytes, stack RS485: 16KB", false);
  addToLog("Chip: " + String(ESP.getChipModel()) + " " + String(ESP.getCpuFreqMHz()) + "MHz", false);
}

// ==========================================
// MAIN LOOP (runs on Core 1)
// Handles: WebServer, MQTT reconnect/publish, WiFi monitoring,
// NTP resync (6h), energy calculation, history accumulation (3-min),
// periodic NVS save (15-min), LED status, heap monitoring.
// Auth reset counter is cleared after 15s of stable operation.
// ==========================================
void loop() {
  esp_task_wdt_reset(); 
  unsigned long now = millis();
  
  uint32_t h_now = ESP.getFreeHeap();
  if (h_now < heap_min) heap_min = h_now;
  static unsigned long last_heap_warn = 0;
  if (h_now < 20000 && now - last_heap_warn > 60000) { last_heap_warn = now; addToLog("WARN: Low heap " + String(h_now) + " bytes (min: " + String(heap_min) + ")", true); }

  // Clear rapid power-cycle counter after 15s stable operation
  static bool rst_cleared = false;
  if (!rst_cleared && now > 15000) {
    rst_cleared = true;
    Preferences rst; rst.begin("gateway", false);
    rst.putUChar("rst_cnt", 0);
    rst.end();
  }

  static unsigned long last_led = 0;
  if(now - last_led > 50) { last_led = now; handleLedStatus(); }

  #ifdef MODE_SMART_WIFI
    server.handleClient();
    // WiFi reconnect monitoring
    static bool wifi_was_connected = true;
    static unsigned long last_wifi_check = 0;
    static unsigned long last_ntp_sync = 0;
    if (now - last_wifi_check > 10000) {
      last_wifi_check = now;
      bool wifi_ok = (WiFi.status() == WL_CONNECTED);
      if (wifi_was_connected && !wifi_ok) { addToLog("WiFi: connection lost, reconnecting...", true); WiFi.reconnect(); }
      else if (!wifi_was_connected && wifi_ok) { addToLog("WiFi: reconnected, IP=" + WiFi.localIP().toString(), false); }
      wifi_was_connected = wifi_ok;
    }
    // NTP resync every 6 hours
    if (now - last_ntp_sync > 21600000UL) {
      last_ntp_sync = now;
      configTime(g_timezone_offset * 3600, 0, g_ntp_server.c_str());
    }
    unsigned long now_hist = millis();
    // Snapshot power under mutex for energy + history (written on Core 0)
    float snap_power = 0, snap_voltage = 0, snap_soc = 0, snap_temp = 0;
    if (dataMutex && xSemaphoreTake(dataMutex, pdMS_TO_TICKS(50))) {
      snap_power = victronData.totalPower;
      snap_voltage = victronData.avgVoltage;
      snap_soc = victronData.avgSOC;
      snap_temp = victronData.avgTemp;
      xSemaphoreGive(dataMutex);
    }
    // Accumulate min/max/avg between history intervals
    float sv[4] = { snap_power, snap_voltage * 100.0f, snap_soc * 10.0f, snap_temp * 10.0f };
    if (acc_reset) { for (int k=0;k<4;k++){acc_min[k]=sv[k];acc_max[k]=sv[k];acc_sum[k]=0;} acc_count=0; acc_reset=false; }
    for (int k=0;k<4;k++) { if(sv[k]<acc_min[k])acc_min[k]=sv[k]; if(sv[k]>acc_max[k])acc_max[k]=sv[k]; acc_sum[k]+=sv[k]; }
    acc_count++;
    if (now_hist - last_hist_time > HISTORY_INTERVAL) {
      last_hist_time = now_hist;
      int idx = historyIdx;
      int16_t *avg_arr[] = { powerHistory, voltHistory, socHistory, tempHistory };
      int16_t *min_arr[] = { powerMin, voltMin, socMin, tempMin };
      int16_t *max_arr[] = { powerMax, voltMax, socMax, tempMax };
      for (int k=0;k<4;k++) {
        int16_t a = (acc_count>0) ? (int16_t)(acc_sum[k]/acc_count) : (int16_t)sv[k];
        avg_arr[k][idx] = a;
        min_arr[k][idx] = (int16_t)acc_min[k];
        max_arr[k][idx] = (int16_t)acc_max[k];
      }
      historyIdx++; if (historyIdx >= HISTORY_LEN) historyIdx = 0;
      if (historyFilled < HISTORY_LEN) historyFilled++;
      acc_reset = true;
    }
    calculateEnergy(snap_power);
    // Periodic save: graph + energy every 15 min
    static unsigned long last_graph_save = 0;
    if (now - last_graph_save > 900000UL) { last_graph_save = now; saveHistory(true); }
    if (g_sd_enable && (now_hist - last_sd_time > SD_LOG_INTERVAL)) { last_sd_time = now_hist; writeLogToSD(); }
    mqttReconnect();
    if (g_mqtt_enable && WiFi.status() == WL_CONNECTED && mqtt.connected()) { mqtt.loop(); if (now_hist - last_mqtt_time > MQTT_INTERVAL) { last_mqtt_time = now_hist; sendMqttData(); } }
    alertDetectTick();   // V2.61: server-side alert generation (throttled to 10s internally)
  #endif
}




