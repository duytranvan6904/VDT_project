#include "offboard_manager/offboard_logic.hpp"
#include <cmath>
#include <limits>

namespace offboard_manager
{

namespace
{
constexpr float kNaN = std::numeric_limits<float>::quiet_NaN();
constexpr float kPi = 3.14159265358979f;

float enu_yaw_to_ned(float yaw_enu)
{
  if (!std::isfinite(yaw_enu)) {
    return kNaN;
  }
  return std::remainder(kPi / 2.0f - yaw_enu, 2.0f * kPi);
}
}  // namespace

Setpoint build_setpoint_search(float yaw_search_rate)
{
  Setpoint sp;
  sp.type = SetpointType::VELOCITY;
  sp.vx = 0.0f;
  sp.vy = 0.0f;
  sp.vz = 0.0f;
  sp.yaw_rate = yaw_search_rate;
  sp.use_yaw_rate = true;
  return sp;
}

Setpoint build_setpoint_hold()
{
  Setpoint sp;
  sp.type = SetpointType::VELOCITY;
  sp.vx = 0.0f;
  sp.vy = 0.0f;
  sp.vz = 0.0f;
  sp.yaw = kNaN;
  sp.use_yaw_rate = false;
  return sp;
}

Setpoint build_setpoint_follow(const PlannerOutput & planner_output)
{
  if (!std::isfinite(planner_output.vx) || !std::isfinite(planner_output.vy) ||
    !std::isfinite(planner_output.vz))
  {
    return build_setpoint_hold();
  }
  Setpoint sp;
  sp.type = SetpointType::VELOCITY;
  sp.vx = planner_output.vy;
  sp.vy = planner_output.vx;
  sp.vz = -planner_output.vz;
  sp.yaw = enu_yaw_to_ned(planner_output.yaw);
  return sp;
}

Setpoint build_setpoint_approach(const PlannerOutput & planner_output)
{
  return build_setpoint_follow(planner_output);
}

Setpoint build_setpoint_land(const PlannerOutput &, float land_descent_rate)
{
  Setpoint sp = build_setpoint_hold();
  sp.vz = std::isfinite(land_descent_rate) ? land_descent_rate : 0.0f;
  return sp;
}

Setpoint build_setpoint(
  FsmState state, const PlannerOutput & planner_output,
  float yaw_search_rate, float land_descent_rate, bool planner_stale)
{
  switch (state) {
    case FsmState::SEARCH:
      return build_setpoint_search(yaw_search_rate);
    case FsmState::FOLLOW:
      return planner_stale ? build_setpoint_hold() : build_setpoint_follow(planner_output);
    case FsmState::APPROACH:
      return planner_stale ? build_setpoint_hold() : build_setpoint_approach(planner_output);
    case FsmState::LAND:
      return build_setpoint_land(planner_output, land_descent_rate);
    default:
      return build_setpoint_search(0.0f);
  }
}

}  // namespace offboard_manager