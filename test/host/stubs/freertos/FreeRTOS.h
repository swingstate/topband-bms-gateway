#pragma once
// Minimal host-build stand-in for the ESP-IDF FreeRTOS portmacro symbols used
// by header-only declarations (shunt_source.h, mppt_source.h portMUX_TYPE
// members). Only enough to let those headers parse for pure-function unit
// tests (e.g. test_aggregator.cpp) that never instantiate the real classes
// or link their .cpp files — the critical-section macros are no-ops here.

struct portMUX_TYPE_ {
  int dummy;
};
using portMUX_TYPE = portMUX_TYPE_;

#define portMUX_INITIALIZER_UNLOCKED \
  { 0 }

inline void portENTER_CRITICAL(portMUX_TYPE*) {}
inline void portEXIT_CRITICAL(portMUX_TYPE*) {}
