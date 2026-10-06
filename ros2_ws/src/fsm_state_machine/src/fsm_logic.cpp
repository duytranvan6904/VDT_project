#include "fsm_state_machine/fsm_logic.hpp"
#include <cmath>

namespace fsm_state_machine
{

SensorInput sensor_validate(SensorInput s, const TimeoutFlags & timeout_flags)
{
  if (timeout_flags.ekf_timeout || timeout_flags.vision_timeout) {
    s.valid = false;
    s.marker_detected = false;
  }
  if (std::isnan(s.align_error) || std::isnan(s.altitude)) {
    s.valid = false;
  }
  s.geometry_valid = s.valid && std::isfinite(s.delta_h) && std::isfinite(s.d_horiz);
  return s;
}

void counters_update_marker_stable(Counters & counters, bool entry_ok)
{
  counters.marker_stable_count = entry_ok ? counters.marker_stable_count + 1 : 0;
}

void counters_update_marker_lost(Counters & counters, bool marker_detected, float dt)
{
  counters.marker_lost_time = marker_detected ? 0.0f : counters.marker_lost_time + dt;
}

bool search_entry_ok(const SensorInput & s, const FsmParams & p)
{
  return s.marker_detected && s.geometry_valid && std::abs(s.yaw_rate) < p.yaw_settle_rate;
}

bool land_entry_ok(const SensorInput & s, const FsmParams & p)
{
  return s.geometry_valid && s.align_error < p.align_threshold && s.delta_h < p.land_entry_height;
}

void counters_update_land_ok(Counters & counters, State current, bool land_ok)
{
  counters.land_ok_count = (current == State::APPROACH && land_ok) ? counters.land_ok_count + 1 : 0;
}

void counters_update_land_inhibit(
  Counters & counters, State prev, State next, const RcInput & rc)
{
  if (!rc.land_switch) {
    counters.land_inhibit = false;
  } else if (prev == State::APPROACH && next != State::APPROACH && next != State::LAND) {
    counters.land_inhibit = true;
  }
}

void counters_reset(Counters & counters)
{
  counters.marker_stable_count = 0;
  counters.land_ok_count = 0;
}

std::optional<State> check_search_transition(const Counters & counters, const FsmParams & p)
{
  if (counters.marker_stable_count >= p.enter_follow_cycles) {
    return State::FOLLOW;
  }
  return std::nullopt;
}

std::optional<State> check_follow_transition(
  const Counters & counters, const RcInput & rc, const SensorInput & s, const FsmParams & p)
{
  if (counters.marker_lost_time > p.follow_lost_timeout) {
    return State::SEARCH;
  }
  const bool inhibited = p.land_requires_rearm && counters.land_inhibit;
  if (rc.land_switch && s.marker_detected && !inhibited) {
    return State::APPROACH;
  }
  return std::nullopt;
}

std::optional<State> check_approach_transition(
  const Counters & counters, const RcInput & rc, const FsmParams & p)
{
  if (!rc.land_switch) {
    return State::FOLLOW;
  }
  if (counters.marker_lost_time > p.approach_lost_timeout) {
    return State::FOLLOW;
  }
  if (counters.land_ok_count >= p.land_entry_cycles) {
    return State::LAND;
  }
  return std::nullopt;
}

std::optional<State> check_land_transition(const SensorInput & s)
{
  if (s.touchdown) {
    return State::COMPLETE;
  }
  return std::nullopt;
}

std::optional<State> evaluate_transition(
  State current, const Counters & counters, const RcInput & rc,
  const SensorInput & s, const FsmParams & p)
{
  switch (current) {
    case State::SEARCH:
      return check_search_transition(counters, p);
    case State::FOLLOW:
      return check_follow_transition(counters, rc, s, p);
    case State::APPROACH:
      return check_approach_transition(counters, rc, p);
    case State::LAND:
      return check_land_transition(s);
    default:
      return std::nullopt;
  }
}

RcInput effective_rc_input(const RcInput & rc, bool force_land_requested)
{
  RcInput effective = rc;
  effective.land_switch = effective.land_switch || force_land_requested;
  return effective;
}

std::optional<State> force_land_transition(State current, bool force_land_requested)
{
  if (force_land_requested && current != State::LAND && current != State::COMPLETE) {
    return State::LAND;
  }
  return std::nullopt;
}

std::optional<State> planner_timeout_transition(State current, bool planner_timeout, const FsmParams & p)
{
  if (p.ignore_planner_timeout) {
    return std::nullopt;
  }
  if (planner_timeout && (current == State::FOLLOW || current == State::APPROACH)) {
    return State::SEARCH;
  }
  return std::nullopt;
}

}  // namespace fsm_state_machine