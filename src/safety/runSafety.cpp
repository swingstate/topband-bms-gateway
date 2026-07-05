// TopBand BMS Gateway — safety/runSafety.cpp
//
// Pure safety-decision function.  Re-implementation of V2.67's
// calculateVictronData() (lines 2081-2233) and calcFactor() (lines 2034-2040).
//
// Architecture §4.4 contract:
//   - No globals, no millis(), no time(), no I/O, no allocations.
//   - Fully unit-testable on host.
//   - Events emitted to SafetyState.events[]; caller routes them.
//
// Event emission matches the Python reference (v267_reference.py) exactly:
//   - Per-pack OV and imbalance fire every cycle when active (no prev check).
//   - System-level OV/UV/temp edges fire on alarm_flags bit transitions.
//   - Temp-stop fires twice: once on factor transition, once on flag transition.
//
// Battery config mode (BatteryConfigMode):
//   Auto       — CCL/DCL/CVL primary source is sysparam (0x47); sanity caps applied.
//   AutoMargin — same as Auto with per-quantity safety insets applied BEFORE caps.
//   Manual     — CCL/DCL from config amps × temp_factor; sysparam as safe-direction cap.
//
// LiFePO4 sanity caps (ALL modes, unsafe direction only):
//   LIFEPO4_CELL_MAX_V  = 3.65 V   — absolute cell OVP ceiling
//   LIFEPO4_CELL_MARGIN_V = 0.05 V  — Auto+Margin cell voltage inset → 3.60 V effective
//   MARGIN_CCL_DCL = 0.10           — 10% inward on CCL/DCL in AutoMargin
//   MARGIN_CVL     = 0.05           — 5% inward on CVL in AutoMargin
//   MARGIN_TEMP_C  = 3.0 °C        — temperature window inset (both sides) in AutoMargin

#include "safety/runSafety.h"
#include "safety/filters.h"
#include <cstring>
#include <cstdio>
#include <cmath>
#include <algorithm>

// ── LiFePO4 sanity cap constants ─────────────────────────────────────────────
// Applied in ALL battery config modes — constrain only the unsafe direction.
static constexpr float LIFEPO4_CELL_MAX_V    = 3.65f;   // hard cell OVP ceiling
static constexpr float LIFEPO4_CELL_MARGIN_V = 0.05f;   // Auto+Margin cell inset → 3.60 V
static constexpr float MARGIN_CCL_DCL        = 0.10f;   // 10% inward for Auto+Margin
static constexpr float MARGIN_CVL            = 0.05f;   // 5% inward CVL for Auto+Margin
static constexpr float MARGIN_TEMP_C         = 3.0f;    // °C inward on each temp bound

namespace safety {

// ── calcFactor ───────────────────────────────────────────────────────────────
// V2.67 calcFactor() (lines 2034-2040).
// Temperature throttle factor. Returns exactly one of {0.0, 0.2, 0.5, 1.0}.
//
// Bands (with soft_zone = 5):
//   t < t_min                 → 0.0   (cutoff)
//   t in [t_min, t_min+5)    → 0.2   (low-temp soft zone)
//   t in [t_min+5, t_max-5]  → 1.0   (normal operating range)
//   t in (t_max-5, t_max]    → 0.5   (high-temp soft zone)
//   t > t_max                 → 0.0   (cutoff)
//
// Low soft zone returns 0.2, high soft zone returns 0.5 — not vice versa.
static float calc_factor(float t, float t_min, float t_max) {
  if (t < t_min) return 0.0f;
  if (t > t_max) return 0.0f;
  constexpr float soft_zone = 5.0f;
  if (t < (t_min + soft_zone)) return 0.2f;
  if (t > (t_max - soft_zone)) return 0.5f;
  return 1.0f;
}

// ── emit_event ───────────────────────────────────────────────────────────────
// Append an event entry to out.events[].  Sets events_overflowed when full.
static void emit_event(SafetyState& out,
                       SafetyState::SafetyEvent type,
                       uint8_t bms_id  = 0xFF,
                       uint64_t alarm_bits = 0) {
  if (out.event_count >= SafetyState::MAX_EVENTS) {
    out.events_overflowed = true;
    return;
  }
  out.events[out.event_count++] = {type, bms_id, alarm_bits};
}

// ── set_message ──────────────────────────────────────────────────────────────
static void set_message(SafetyState& out, const char* msg) {
  snprintf(out.sys_message, sizeof(out.sys_message), "%s", msg);
}

// ── runSafety ─────────────────────────────────────────────────────────────────
void runSafety(const BmsSystemSnapshot& snap,
               const Config&            cfg,
               const PrevSafetyState&   prev,
               uint32_t                 now_ms,
               SafetyState&             out) {
  // Zero-initialise output so no field is left uninitialised.
  memset(&out, 0, sizeof(SafetyState));
  set_message(out, "OK");
  out.cycle_id         = snap.cycle_id;
  out.produced_ms      = now_ms;
  out.packs_configured = snap.pack_count_configured;
  out.factor_charge    = 1.0f;
  out.factor_discharge = 1.0f;
  // Discharge voltage limit reported to the inverter (Pylontech 0x351 bytes 6-7):
  // the configured pack low-voltage cutoff. A stable config constant, not a live
  // measurement, so it is meaningful regardless of pack state — kept unchanged
  // even during a discharge-disable (dcl_amps==0 already signals the lockout).
  out.dvl_volts        = cfg.safe_pack_volt;

  // ── Accumulators ─────────────────────────────────────────────────────────
  float sum_i   = 0.0f, sum_v = 0.0f, sum_soc = 0.0f, sum_soh = 0.0f;
  float sum_t   = 0.0f, sum_cap = 0.0f, sum_rem = 0.0f;
  float proto_ccl_cap = 0.0f, proto_dcl_cap = 0.0f;
  float proto_cvl_cap = 999.0f;        // sentinel: no sysparam-based cap yet
  bool  proto_under_v_hit = false;
  int   proto_count    = 0;
  int   count          = 0;            // online pack count

  // Auto/AutoMargin accumulators: most-conservative sysparam temp limits
  // (highest low_t and lowest high_t across all online packs with fresh sysparam).
  float auto_chg_t_min =  -999.0f;    // max across packs → most restrictive
  float auto_chg_t_max =   999.0f;    // min across packs → most restrictive
  float auto_dis_t_min =  -999.0f;
  float auto_dis_t_max =   999.0f;
  int   auto_sp_count  = 0;           // packs with valid sysparam temp limits

  const Config::BatteryConfigMode mode = cfg.battery_config_mode;

  // t_check_val: V2.67 overwrites this per online pack → last online pack wins.
  float t_check_val = 25.0f;

  // ── Per-pack loop ─────────────────────────────────────────────────────────
  for (int i = 0; i < snap.pack_count_configured; ++i) {
    const BmsPackSnapshot& p = snap.pack[i];
    bool on = p.online;

    // Edge: BMS went offline (V2.67 lines 2098-2102)
    // sys_message only set if still "OK" (first offline pack wins)
    if (prev.was_pack_online[i] && !on) {
      if (strcmp(out.sys_message, "OK") == 0) {
        char msg_buf[48];
        snprintf(msg_buf, sizeof(msg_buf), "ALARM: BMS%d OFFLINE", i);
        set_message(out, msg_buf);
      }
      emit_event(out, SafetyState::BmsWentOffline, static_cast<uint8_t>(i));
    }
    // Edge: BMS came back online
    if (!prev.was_pack_online[i] && on) {
      emit_event(out, SafetyState::BmsCameOnline, static_cast<uint8_t>(i));
    }

    if (!on) continue;

    // ── Alarm filter (V2.67 lines 2104-2127) ────────────────────────────
    bool alarm_fresh = (p.last_alarm_ms > 0) &&
                       ((now_ms - p.last_alarm_ms) < 60000u);
    if (alarm_fresh) {
      int cells_for_filter = (p.cell_count > 0) ? p.cell_count : 0;
      float temp_limit = (cfg.charge_temp_max < cfg.discharge_temp_max)
                         ? cfg.charge_temp_max : cfg.discharge_temp_max;
      uint64_t crit_bits = filters::filter_alarm_bits(
          p.alarm_bits,
          p.pack_voltage,
          p.temp_max_c,
          cells_for_filter,
          temp_limit,
          p.cell_max_v,
          p.sys_cell_high_v,
          p.sys_module_high_v);
      if (crit_bits != 0) {
        if (crit_bits & filters::UV_ALARM_BITS) {
          out.alarm_flags |= 0x10;
          if (strcmp(out.sys_message, "OK") == 0)
            set_message(out, "ALARM: PACK UNDERVOLT");
        } else {
          out.alarm_flags |= 0x40;
          if (strcmp(out.sys_message, "OK") == 0)
            set_message(out, "ALARM: BMS STATUS");
        }
        // Always emit when crit_bits != 0 (V2.67 behaviour; no prev check)
        emit_event(out, SafetyState::BmsReportedAlarm,
                   static_cast<uint8_t>(i), crit_bits);
      }
    }

    // ── Sysparam protocol caps (V2.67 lines 2128-2145) ──────────────────
    bool sp_fresh = p.sysparam_valid &&
                    ((now_ms - p.last_sysparam_ms) < 300000u);
    if (sp_fresh) {
      if (p.sys_charge_max_a > 0.1f)
        proto_ccl_cap += p.sys_charge_max_a;
      if (p.sys_discharge_max_a > 0.1f)
        proto_dcl_cap += p.sys_discharge_max_a;

      int cells_for_cap = (p.cell_count > 0) ? p.cell_count : 0;
      float cvl_cap = p.sys_module_high_v - 0.20f;
      if (cells_for_cap > 0) {
        float chem_cap = static_cast<float>(cells_for_cap) * 3.60f;
        if (cvl_cap > chem_cap) cvl_cap = chem_cap;
      }
      if (cfg.safe_pack_volt > 40.0f && cvl_cap > (cfg.safe_pack_volt - 0.20f))
        cvl_cap = cfg.safe_pack_volt - 0.20f;
      if (cfg.cvl_voltage > 40.0f && cvl_cap > cfg.cvl_voltage)
        cvl_cap = cfg.cvl_voltage;
      if (cvl_cap > 40.0f && cvl_cap < proto_cvl_cap)
        proto_cvl_cap = cvl_cap;

      int cells_for_uv = cells_for_cap;
      if (filters::should_flag_proto_under_volt(
              p.pack_voltage, p.sys_module_under_v, cells_for_uv)) {
        proto_under_v_hit = true;
      }

      proto_count++;
    }

    // ── Effective OVP thresholds — mode + sanity caps (V2.67 lines 2146-2147) ──
    // In Auto/AutoMargin the threshold comes from pack sysparam; in Manual it
    // comes from config. The LIFEPO4_CELL_MAX_V hard cap applies in all modes
    // so a faulty pack can never raise the ceiling above the physical chemistry
    // maximum. Caps constrain only the unsafe direction.
    float eff_cell_cap_v;
    float eff_pack_cap_v;
    if ((mode == Config::BatteryConfigMode::Auto ||
         mode == Config::BatteryConfigMode::AutoMargin) && p.sysparam_valid) {
      float cell_ceiling = LIFEPO4_CELL_MAX_V;
      if (mode == Config::BatteryConfigMode::AutoMargin)
        cell_ceiling -= LIFEPO4_CELL_MARGIN_V;      // 3.60 V in AutoMargin
      eff_cell_cap_v = (p.sys_cell_high_v > 0.1f)
                       ? std::min(p.sys_cell_high_v, cell_ceiling)
                       : cell_ceiling;
      float pack_ceiling = static_cast<float>(p.cell_count > 0 ? p.cell_count : 16)
                           * cell_ceiling;
      eff_pack_cap_v = (p.sys_module_high_v > 0.1f)
                       ? std::min(p.sys_module_high_v, pack_ceiling)
                       : pack_ceiling;
    } else {
      // Manual or Auto without sysparam yet: use config, capped at absolute limit.
      eff_cell_cap_v = std::min(cfg.safe_cell_volt, LIFEPO4_CELL_MAX_V);
      eff_pack_cap_v = cfg.safe_pack_volt;
    }

    if (p.pack_voltage > eff_pack_cap_v) {
      out.alarm_flags |= 0x02;
      set_message(out, "ALARM: PACK OVERVOLT");  // unconditional overwrite
    }
    if (p.cell_max_v > eff_cell_cap_v) {
      out.alarm_flags |= 0x02;
      set_message(out, "ALARM: CELL OVERVOLT");  // unconditional overwrite
    }

    // Per-pack OV event: always emits every cycle when OV is active (V2.67).
    // System-level PackOvervoltStart is emitted separately via alarm_flags edge.
    bool ov_now = (p.pack_voltage > eff_pack_cap_v) ||
                  (p.cell_max_v > eff_cell_cap_v);
    if (ov_now)
      emit_event(out, SafetyState::PackOvervoltStart, static_cast<uint8_t>(i));

    // ── Auto/AutoMargin: accumulate most-conservative sysparam temp limits ───
    if ((mode == Config::BatteryConfigMode::Auto ||
         mode == Config::BatteryConfigMode::AutoMargin) && p.sysparam_valid) {
      if (p.sys_charge_low_t > auto_chg_t_min)  auto_chg_t_min = p.sys_charge_low_t;
      if (p.sys_charge_high_t < auto_chg_t_max) auto_chg_t_max = p.sys_charge_high_t;
      if (p.sys_discharge_low_t > auto_dis_t_min)  auto_dis_t_min = p.sys_discharge_low_t;
      if (p.sys_discharge_high_t < auto_dis_t_max) auto_dis_t_max = p.sys_discharge_high_t;
      auto_sp_count++;
    }

    // ── Temperature t_check_val update (V2.67 line 2148) ─────────────────
    if (cfg.temp_mode == Config::TempMode::Hottest)
      t_check_val = p.temp_max_c;
    else
      t_check_val = p.temp_avg_c;

    // ── Cell drift (V2.67 line 2149) ─────────────────────────────────────
    // Always emits CellImbalanceStart every cycle when active (V2.67 behaviour).
    if (p.cell_drift_v > cfg.safe_cell_drift) {
      out.alarm_flags |= 0x20;
      if (strcmp(out.sys_message, "OK") == 0)
        set_message(out, "WARN: CELL IMBALANCE");
      emit_event(out, SafetyState::CellImbalanceStart, static_cast<uint8_t>(i));
    }

    // ── Accumulate (V2.67 line 2150) ─────────────────────────────────────
    sum_i   += p.pack_current;
    sum_v   += p.pack_voltage;
    sum_soc += static_cast<float>(p.soc);
    sum_soh += static_cast<float>(p.soh);
    sum_t   += p.temp_avg_c;
    sum_cap += p.full_ah;
    sum_rem += p.rem_ah;
    count++;
  }  // end per-pack loop

  // ── Count > 0 path (V2.67 lines 2154-2266) ───────────────────────────────
  if (count > 0) {
    out.packs_online         = static_cast<uint8_t>(count);
    out.pack_current_total   = sum_i;
    out.pack_voltage_avg     = sum_v / static_cast<float>(count);
    out.soc_avg              = sum_soc / static_cast<float>(count);
    out.soh_avg              = sum_soh / static_cast<float>(count);
    out.temp_avg             = sum_t / static_cast<float>(count);
    out.capacity_total_ah    = sum_cap;
    out.capacity_remain_ah   = sum_rem;

    // Packs-online-recovered edge (any packs came back after full outage)
    if (!prev.was_packs_online_any)
      emit_event(out, SafetyState::PacksOnlineRecovered);

    // ── Effective temperature limits for calc_factor ─────────────────────
    // Auto/AutoMargin: use most-conservative sysparam temp limits aggregated
    // from all online packs with fresh sysparam. AutoMargin applies a 3 °C
    // inset on each bound. If no sysparam yet, fall back to config values.
    // Manual: always use config values.
    float eff_chg_t_min, eff_chg_t_max, eff_dis_t_min, eff_dis_t_max;
    if ((mode == Config::BatteryConfigMode::Auto ||
         mode == Config::BatteryConfigMode::AutoMargin) && auto_sp_count > 0) {
      float tm = (mode == Config::BatteryConfigMode::AutoMargin) ? MARGIN_TEMP_C : 0.0f;
      eff_chg_t_min = auto_chg_t_min + tm;
      eff_chg_t_max = auto_chg_t_max - tm;
      eff_dis_t_min = auto_dis_t_min + tm;
      eff_dis_t_max = auto_dis_t_max - tm;
    } else {
      eff_chg_t_min = cfg.charge_temp_min;
      eff_chg_t_max = cfg.charge_temp_max;
      eff_dis_t_min = cfg.discharge_temp_min;
      eff_dis_t_max = cfg.discharge_temp_max;
    }

    // ── Temperature factors (V2.67 lines 2186-2189) ─────────────────────
    out.factor_charge    = calc_factor(t_check_val, eff_chg_t_min, eff_chg_t_max);
    out.factor_discharge = calc_factor(t_check_val, eff_dis_t_min, eff_dis_t_max);

    // ── CCL/DCL/CVL: mode-dependent primary source ───────────────────────
    // Auto/AutoMargin: sysparam is the primary source (battery-reported limits
    // including temperature-dependent DCL). Manual: config values are primary.
    // In all modes: safe direction (cold-DCL) passes through uncapped.
    float safe_chg, safe_dis;
    if ((mode == Config::BatteryConfigMode::Auto ||
         mode == Config::BatteryConfigMode::AutoMargin) && proto_count > 0) {
      // Sysparam primary: CCL/DCL come from the pack's 0x47 reporting.
      // The pack itself already accounts for temperature in its reported limits.
      // We still apply our temp factor as an additional safety layer (most
      // conservative: both the pack's self-reported cold reduction AND our
      // own temperature throttle are honoured — the lower value wins).
      float margin = (mode == Config::BatteryConfigMode::AutoMargin)
                     ? (1.0f - MARGIN_CCL_DCL) : 1.0f;
      safe_chg = (proto_ccl_cap > 0.1f ? proto_ccl_cap : 0.0f) * margin
                 * out.factor_charge;
      safe_dis = (proto_dcl_cap > 0.1f ? proto_dcl_cap : 0.0f) * margin
                 * out.factor_discharge;

      // CVL: from sysparam, reduced by margin in AutoMargin, then capped
      // at cell_count × cell_ceiling so a mis-reporting pack cannot push
      // CVL above the chemistry maximum.
      if (proto_cvl_cap < 900.0f) {
        float cvl = (mode == Config::BatteryConfigMode::AutoMargin)
                    ? proto_cvl_cap * (1.0f - MARGIN_CVL)
                    : proto_cvl_cap;
        out.cvl_volts = cvl;
      } else {
        out.cvl_volts = cfg.cvl_voltage;
      }
    } else {
      // Manual (or Auto/AutoMargin before first 0x47 arrives):
      // Use config-based values (V2.67 behavior), with sysparam as a safe-direction
      // cap in case the user's configured value is higher than what the pack allows.
      safe_chg = static_cast<float>(count) * cfg.charge_amps_per_pack
                 * out.factor_charge;
      safe_dis = static_cast<float>(count) * cfg.discharge_amps_per_pack
                 * out.factor_discharge;

      // Protocol caps from sysparam (V2.67 lines 2194-2205) — safe direction only.
      if (proto_count > 0) {
        if (proto_ccl_cap > 0.1f && safe_chg > proto_ccl_cap)
          safe_chg = proto_ccl_cap;
        if (proto_dcl_cap > 0.1f && safe_dis > proto_dcl_cap)
          safe_dis = proto_dcl_cap;
        out.cvl_volts = (proto_cvl_cap < 900.0f) ? proto_cvl_cap : cfg.cvl_voltage;
      } else {
        out.cvl_volts = cfg.cvl_voltage;
      }
    }

    // ── Temperature stop flags (V2.67 lines 2207-2208) ──────────────────
    // Unconditional overwrite of sys_message (V2.67 behaviour).
    if (out.factor_charge == 0.0f) {
      out.alarm_flags |= 0x08;
      set_message(out, "INFO: TEMP CHARGE STOP");
    }
    if (out.factor_discharge == 0.0f) {
      out.alarm_flags |= 0x08;
      set_message(out, "INFO: TEMP DISCHG STOP");
    }

    // Temperature-stop direction (cold vs hot). The 0x08 flag above is combined;
    // CAN encoders that report over/under temperature separately (Pylontech
    // 0x359) need the direction. Derived from the same t_check_val and effective
    // thresholds that produced the zero factor: below the relevant min = cold
    // (under-temp), above the relevant max = hot (over-temp). No new comparison
    // logic — this is the same source of truth, just projected onto a direction.
    if (out.factor_charge == 0.0f || out.factor_discharge == 0.0f) {
      if (t_check_val < eff_chg_t_min || t_check_val < eff_dis_t_min)
        out.temp_alarm |= 0x01;  // under-temperature (cold)
      if (t_check_val > eff_chg_t_max || t_check_val > eff_dis_t_max)
        out.temp_alarm |= 0x02;  // over-temperature (hot)
    }

    // ── Factor edge events (V2.67 lines 2220-2223) ───────────────────────
    // TempChargeStop fires when throttle starts (factor goes below 1.0),
    // which includes soft zones (0.2 or 0.5), not only full cutoff (0.0).
    if (out.factor_charge < 1.0f && prev.prev_factor_charge >= 1.0f)
      emit_event(out, SafetyState::TempChargeStop);
    if (out.factor_charge >= 1.0f && prev.prev_factor_charge < 1.0f)
      emit_event(out, SafetyState::TempChargeResume);
    if (out.factor_discharge < 1.0f && prev.prev_factor_discharge >= 1.0f)
      emit_event(out, SafetyState::TempDischargeStop);
    if (out.factor_discharge >= 1.0f && prev.prev_factor_discharge < 1.0f)
      emit_event(out, SafetyState::TempDischargeResume);

    // ── System-level alarm flag edge events (V2.67 lines 2215-2219) ─────
    // Note: checked before proto_under_v_hit modifies alarm_flags, so a
    // pure-proto UV trigger does not fire PackUndervoltStart here (V2.67).
    if ((out.alarm_flags & 0x02) && !(prev.prev_alarm_flags & 0x02))
      emit_event(out, SafetyState::PackOvervoltStart);
    if (!(out.alarm_flags & 0x02) && (prev.prev_alarm_flags & 0x02))
      emit_event(out, SafetyState::PackOvervoltClear);
    if ((out.alarm_flags & 0x10) && !(prev.prev_alarm_flags & 0x10))
      emit_event(out, SafetyState::PackUndervoltStart);
    if (!(out.alarm_flags & 0x10) && (prev.prev_alarm_flags & 0x10))
      emit_event(out, SafetyState::PackUndervoltClear);
    // Second TempChargeStop/DischargeStop when temp-stop flag first appears.
    if ((out.alarm_flags & 0x08) && !(prev.prev_alarm_flags & 0x08)) {
      if (out.factor_charge == 0.0f)
        emit_event(out, SafetyState::TempChargeStop);
      if (out.factor_discharge == 0.0f)
        emit_event(out, SafetyState::TempDischargeStop);
    }

    // ── Proto undervolt (V2.67 lines 2209-2210) ──────────────────────────
    if (proto_under_v_hit) {
      out.alarm_flags |= 0x10;
      safe_dis = 0.0f;
      if (strcmp(out.sys_message, "OK") == 0)
        set_message(out, "ALARM: PACK UNDERVOLT");
    }

    // ── Lock to zero rule (V2.67 line 2211) ─────────────────────────────
    if ((out.alarm_flags & 0x02) ||
        (out.alarm_flags & 0x10) ||
        (out.alarm_flags & 0x40)) {
      safe_chg = 0.0f;
      safe_dis = 0.0f;
    }

    // ── SOC-based charge taper (V2.67 lines 2226-2233) ──────────────────
    // PERMANENT SAFETY INVARIANT — not a TODO, not pending further validation:
    // this taper is keyed on out.soc_avg (pure BMS mean) and MUST NEVER read
    // any Battery Value Sources fused value (SafetyState::soc_display/
    // voltage_display/current_display, set by the caller in bms/poller.cpp
    // strictly after runSafety() returns). There is no Auto/Manual policy, no
    // config field, and no code path anywhere that may redirect this taper to
    // the shunt — Config::BatterySourcePolicy and Config::MetricSource
    // (storage/config.h) govern display/dashboard/MQTT only and are never
    // threaded into runSafety()'s inputs. Charge-taper stays on the BMS data
    // path permanently; the shunt improves display/telemetry only, never
    // charge control.
    //
    // NOTE (V3.2): this is a DIFFERENT decision from the CAN TX *reported* SOC.
    // As of V3.2 the SOC number sent to the inverter (can/*.cpp build_0x355 via
    // can_tx_soc()) follows the fused Combined SOC, matching the dashboard. That
    // is a display/reporting choice and does NOT feed this taper: the taper reads
    // out.soc_avg directly below, never soc_display. Do not "unify" them.
    // See docs/research/v3.2-shunt-soc-fusion.md.
    if (!cfg.maint_charge_enabled) {
      if (out.soc_avg >= 99.0f)
        safe_chg = static_cast<float>(count) * 2.0f;
      if (out.soc_avg >= 100.0f)
        safe_chg = 0.0f;
    } else {
      float maint_target = cfg.maint_target_voltage;
      if (maint_target < 40.0f) maint_target = 40.0f;
      if (maint_target > cfg.safe_pack_volt) maint_target = cfg.safe_pack_volt;
      if (out.pack_voltage_avg >= maint_target &&
          safe_chg > static_cast<float>(count) * 2.0f) {
        safe_chg = static_cast<float>(count) * 2.0f;
      }
    }

    out.ccl_amps = safe_chg;
    out.dcl_amps = safe_dis;

  } else {
    // ── No packs online (V2.67 line 2267) ────────────────────────────────
    out.packs_online = 0;
    out.alarm_flags |= 0x80;
    out.ccl_amps = 0.0f;
    out.dcl_amps = 0.0f;
    out.cvl_volts = 0.0f;

    if (prev.was_packs_online_any)
      emit_event(out, SafetyState::NoPacksOnline);
  }
}

// ── update_prev_state ─────────────────────────────────────────────────────────
void update_prev_state(const SafetyState&       current,
                       const BmsSystemSnapshot& snap,
                       PrevSafetyState&         inout) {
  inout.prev_alarm_flags      = current.alarm_flags;
  inout.prev_factor_charge    = current.factor_charge;
  inout.prev_factor_discharge = current.factor_discharge;
  inout.was_packs_online_any  = (current.packs_online > 0);
  for (int i = 0; i < 16; ++i)
    inout.was_pack_online[i] = snap.pack[i].online;
}

// ── make_default_prev ─────────────────────────────────────────────────────────
PrevSafetyState make_default_prev() {
  PrevSafetyState p{};
  memset(&p, 0, sizeof(p));
  p.prev_factor_charge    = 1.0f;
  p.prev_factor_discharge = 1.0f;
  return p;
}

}  // namespace safety
