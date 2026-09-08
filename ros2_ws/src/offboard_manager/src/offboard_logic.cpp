#include "offboard_manager/offboard_logic.hpp"
#include <cmath>

namespace offboard_manager
{

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

Setpoint build_setpoint_follow(const PlannerOutput & planner_output)
{
  Setpoint sp;
  sp.type = SetpointType::VELOCITY;
  if (!std::isfinite(planner_output.vx) || !std::isfinite(planner_output.vy) ||
    !std::isfinite(planner_output.vz) || !std::isfinite(planner_output.yaw))
  {
    return build_setpoint_search(0.0f);
  }
  sp.vx = planner_output.vx;
  sp.vy = planner_output.vy;
  sp.vz = planner_output.vz;
  sp.yaw = planner_output.yaw;
  return sp;
}

Setpoint build_setpoint_approach(const PlannerOutput & planner_output)
{
  return build_setpoint_follow(planner_output);
}

Setpoint build_setpoint_land(const PlannerOutput & planner_output, float land_descent_rate)
{
  Setpoint sp;
  sp.type = SetpointType::VELOCITY;
  sp.vx = 0.0f;
  sp.vy = 0.0f;
  sp.vz = std::isfinite(land_descent_rate) ? land_descent_rate : 0.0f;
  sp.yaw = std::isfinite(planner_output.yaw) ? planner_output.yaw : 0.0f;
  return sp;
}

Setpoint build_setpoint(
  FsmState state, const PlannerOutput & planner_output,
  float yaw_search_rate, float land_descent_rate)
{
  switch (state) {
    case FsmState::SEARCH:
      return build_setpoint_search(yaw_search_rate);
    case FsmState::FOLLOW:
      return build_setpoint_follow(planner_output);
    case FsmState::APPROACH:
      return build_setpoint_approach(planner_output);
    case FsmState::LAND:
      return build_setpoint_land(planner_output, land_descent_rate);
    default:
      return build_setpoint_search(0.0f);
  }
}

}  // namespace offboard_manager