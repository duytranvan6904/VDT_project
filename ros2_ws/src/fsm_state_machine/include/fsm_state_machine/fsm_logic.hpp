#pragma once
#include <optional>
#include "fsm_state_machine/fsm_types.hpp"

namespace fsm_state_machine
{

SensorInput sensor_validate(SensorInput s, const TimeoutFlags & timeout_flags);

bool search_entry_ok(const SensorInput & s, const FsmParams & p);
bool land_entry_ok(const SensorInput & s, const FsmParams & p);

void counters_update_marker_stable(Counters & counters, bool entry_ok);
void counters_update_marker_lost(Counters & counters, bool marker_detected, float dt);
void counters_update_land_ok(Counters & counters, State current, bool land_ok);
void counters_update_land_inhibit(
  Counters & counters, State prev, State next, const RcInput & rc);
void counters_reset(Counters & counters);

std::optional<State> check_search_transition(const Counters & counters, const FsmParams & p);
std::optional<State> check_follow_transition(
  const Counters & counters, const RcInput & rc, const SensorInput & s, const FsmParams & p);
std::optional<State> check_approach_transition(
  const Counters & counters, const RcInput & rc, const FsmParams & p);
std::optional<State> check_land_transition(const SensorInput & s);

std::optional<State> evaluate_transition(
  State current, const Counters & counters, const RcInput & rc,
  const SensorInput & s, const FsmParams & p);

RcInput effective_rc_input(const RcInput & rc, bool force_land_requested);
std::optional<State> force_land_transition(State current, bool force_land_requested);
std::optional<State> planner_timeout_transition(
  State current, bool planner_timeout, const FsmParams & p);

}  // namespace fsm_state_machine