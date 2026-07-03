// Link-only stubs for BmsSource/ShuntSource/MpptSource update()/reading().
//
// test_aggregator.cpp exercises only Aggregator's static pure functions
// (select() / select_bank_value()) and never calls them through a live
// Aggregator instance — but Aggregator::update()/reading() (defined in
// aggregator.cpp, unused by the test but still part of the translation unit)
// reference these methods, so they must resolve at link time. Real behaviour
// lives in the actual src/sources/*_source.cpp files, which are intentionally
// NOT linked here (they pull in NimBLE/BLE scan callback dependencies that
// don't belong in a host unit test).
#include "sources/bms_source.h"
#include "sources/shunt_source.h"
#include "sources/mppt_source.h"

namespace sources {

void BmsSource::update() {}
SourceReading BmsSource::reading(Metric) const { return unavailable_reading(); }

void ShuntSource::update() {}
SourceReading ShuntSource::reading(Metric) const { return unavailable_reading(); }

void MpptSource::update() {}
SourceReading MpptSource::reading(Metric) const { return unavailable_reading(); }

}  // namespace sources
