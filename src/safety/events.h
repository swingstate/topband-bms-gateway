#pragma once
// Convenience include. SafetyEvent and EventEntry are defined inside SafetyState
// (architecture §5.4). This header exists so callers that only need the event
// types can include a semantically named header without pulling all of safety/.
#include "safety_state.h"

namespace safety {
  // Re-export the types from the embedded namespace so safety:: code can use them
  // without fully qualifying SafetyState::.
  using SafetyEvent = SafetyState::SafetyEvent;
  using EventEntry  = SafetyState::EventEntry;
  constexpr size_t MAX_EVENTS_PER_CYCLE = SafetyState::MAX_EVENTS;
}  // namespace safety
