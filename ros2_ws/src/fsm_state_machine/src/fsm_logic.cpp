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
  return s;
}

void counters_update_marker_stable(Counters & counters, bool marker_detected)
{
  counters.marker_stable_count = marker_detected ? counters.marker_stable_count + 1 : 0;
}

void counters_update_marker_lost(Counters & counters, bool marker_detected, float dt)
{
  counters.marker_lost_time = marker_detected ? 0.0f : counters.marker_lost_time + dt;
}

void counters_reset(Counters & counters)
{
  counters.marker_stable_count = 0;
  counters.marker_lost_time = 0.0f;
}

std::optional<State> check_search_transition(const Counters & counters)
{
  if (counters.marker_stable_count >= 10) {
    return State::FOLLOW;
  }
  return std::nullopt;
}

std::optional<State> check_follow_transition(
  const Counters & counters, const RcInput & rc, const SensorInput & s)
{
  if (counters.marker_lost_time > 5.0f) {
    return State::SEARCH;
  }
  if (rc.land_switch && s.marker_detected) {
    return State::APPROACH;
  }
  return std::nullopt;
}

std::optional<State> check_approach_transition(
  const Counters & counters, const RcInput & rc, const SensorInput & s, float land_entry_height)
{
  if (!rc.land_switch) {
    return State::FOLLOW;
  }
  if (counters.marker_lost_time > 3.0f) {
    return State::FOLLOW;
  }
  if (s.align_error < 0.3f && s.delta_h < land_entry_height) {
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
  const SensorInput & s, float land_entry_height)
{
  switch (current) {
    case State::SEARCH:
      return check_search_transition(counters);
    case State::FOLLOW:
      return check_follow_transition(counters, rc, s);
    case State::APPROACH:
      return check_approach_transition(counters, rc, s, land_entry_height);
    case State::LAND:
      return check_land_transition(s);
    default:
      return std::nullopt;
  }
}

}  // namespace fsm_state_machine