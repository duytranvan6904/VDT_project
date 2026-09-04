#pragma once
#include <optional>
#include "fsm_state_machine/fsm_types.hpp"

namespace fsm_state_machine
{

SensorInput sensor_validate(SensorInput s, const TimeoutFlags & timeout_flags);

void counters_update_marker_stable(Counters & counters, bool marker_detected);
void counters_update_marker_lost(Counters & counters, bool marker_detected, float dt);
void counters_reset(Counters & counters);

std::optional<State> check_search_transition(const Counters & counters);
std::optional<State> check_follow_transition(
  const Counters & counters, const RcInput & rc, const SensorInput & s);
std::optional<State> check_approach_transition(
  const Counters & counters, const RcInput & rc, const SensorInput & s, float land_entry_height);
std::optional<State> check_land_transition(const SensorInput & s);

std::optional<State> evaluate_transition(
  State current, const Counters & counters, const RcInput & rc,
  const SensorInput & s, float land_entry_height);

}  // namespace fsm_state_machine