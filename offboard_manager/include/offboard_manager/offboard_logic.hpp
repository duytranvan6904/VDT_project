#pragma once
#include "offboard_manager/offboard_types.hpp"

namespace offboard_manager
{

Setpoint build_setpoint_search(float yaw_search_rate);
Setpoint build_setpoint_follow(const PlannerOutput & planner_output);
Setpoint build_setpoint_approach(const PlannerOutput & planner_output);
Setpoint build_setpoint_land(const PlannerOutput & planner_output, float land_descent_rate);
Setpoint build_setpoint(
  FsmState state, const PlannerOutput & planner_output,
  float yaw_search_rate, float land_descent_rate);

}  // namespace offboard_manager