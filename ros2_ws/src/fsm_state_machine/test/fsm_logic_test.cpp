#include <cassert>
#include <cmath>
#include "fsm_state_machine/fsm_logic.hpp"

using namespace fsm_state_machine;

int main()
{
  const RcInput rc{};
  const RcInput effective = effective_rc_input(rc, true);
  assert(effective.land_switch);

  for (const State state : {State::SEARCH, State::FOLLOW, State::APPROACH}) {
    const auto transition = force_land_transition(state, true);
    assert(transition && *transition == State::LAND);
  }
  assert(!force_land_transition(State::LAND, true));
  assert(!force_land_transition(State::COMPLETE, true));

  const auto follow_timeout = planner_timeout_transition(State::FOLLOW, true);
  const auto approach_timeout = planner_timeout_transition(State::APPROACH, true);
  assert(follow_timeout && *follow_timeout == State::SEARCH);
  assert(approach_timeout && *approach_timeout == State::SEARCH);
  assert(!planner_timeout_transition(State::SEARCH, true));
  assert(!planner_timeout_transition(State::FOLLOW, false));

  SensorInput sensor;
  sensor.marker_detected = true;
  sensor.align_error = 0.1f;
  sensor.delta_h = 0.2f;
  const Counters counters{};
  const auto approach = evaluate_transition(State::FOLLOW, counters, effective, sensor, 0.5f);
  assert(approach && *approach == State::APPROACH);
  assert(std::isfinite(sensor.align_error));
  return 0;
}
