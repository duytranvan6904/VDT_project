#include <cassert>
#include <cmath>
#include <limits>
#include "offboard_manager/offboard_logic.hpp"

using namespace offboard_manager;

int main()
{
  PlannerOutput invalid;
  invalid.vx = std::numeric_limits<float>::quiet_NaN();
  const Setpoint safe = build_setpoint_follow(invalid);
  assert(safe.vx == 0.0f);
  assert(safe.vy == 0.0f);
  assert(safe.vz == 0.0f);
  assert(safe.use_yaw_rate);

  PlannerOutput planner;
  planner.vx = 1.0f;
  planner.vy = -0.5f;
  planner.vz = 0.2f;
  planner.yaw = 0.4f;
  const Setpoint follow = build_setpoint_follow(planner);
  assert(std::isfinite(follow.vx));
  assert(follow.vx == planner.vx);

  planner.yaw = std::numeric_limits<float>::infinity();
  const Setpoint land = build_setpoint_land(planner, 0.4f);
  assert(land.yaw == 0.0f);
  return 0;
}
