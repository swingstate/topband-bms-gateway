#include "bms_source.h"
#include "bus/snapshot_bus.h"
#include "esp_timer.h"
#include <cmath>

namespace sources {

void BmsSource::update() {
  BmsSystemSnapshot snap{};
  if (!bus::snapshot_bus::read(snap)) return;

  m_ts_ms = (uint32_t)(esp_timer_get_time() / 1000LL);
  m_pack_online = (snap.pack_count_online > 0);

  // Aggregate total current and average voltage from online packs.
  float total_current = 0.0f;
  float total_voltage = 0.0f;
  uint8_t online_count = 0;

  for (uint8_t i = 0; i < snap.pack_count_configured && i < 16; ++i) {
    const BmsPackSnapshot& p = snap.pack[i];
    if (!p.online) continue;
    total_current += p.pack_current;
    total_voltage += p.pack_voltage;
    ++online_count;
  }

  m_current_a = total_current;
  m_voltage_v = (online_count > 0) ? (total_voltage / online_count) : 0.0f;
}

SourceReading BmsSource::reading(Metric m) const {
  switch (m) {
    case Metric::TOTAL_CURRENT: {
      if (!m_pack_online || m_ts_ms == 0) return unavailable_reading("A");
      ReadingStatus st = (fabsf(m_current_a) < BMS_CURRENT_RESOLUTION_A)
                         ? ReadingStatus::Stale   // below BMS resolution floor
                         : ReadingStatus::Valid;
      return { m_current_a, "A", m_ts_ms, st };
    }
    case Metric::TOTAL_VOLTAGE: {
      if (!m_pack_online || m_ts_ms == 0) return unavailable_reading("V");
      return { m_voltage_v, "V", m_ts_ms, ReadingStatus::Valid };
    }
    default:
      return unavailable_reading("");
  }
}

}  // namespace sources
