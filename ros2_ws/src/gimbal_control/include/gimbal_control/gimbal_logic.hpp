#pragma once
#include "gimbal_control/gimbal_types.hpp"
#include "vdt_msgs/msg/timeout_flags.hpp"

namespace gimbal_control
{
namespace msg = vdt_msgs::msg;
GimbalTelemetry telemetry_validate(
  GimbalTelemetry t, const vdt_msgs::msg::TimeoutFlags & flags);

float target_angle_search();
float target_angle_follow(const GimbalTelemetry & t);
float target_angle_approach(const GimbalTelemetry & t);
float target_angle_land(const GimbalTelemetry & t, float land_entry_height);
float gimbal_target_angle(GimbalPhase phase, const GimbalTelemetry & t, float land_entry_height);

void pid_reset(PidState & pid);
float pid_update(PidState & pid, float target, float current, float dt);

float clamp_slew_rate(float prev_angle, float new_angle, float max_rate_deg_s, float dt);

}  // namespace gimbal_control