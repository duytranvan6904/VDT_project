#include "gimbal_control/gimbal_logic.hpp"
#include <algorithm>
#include <cmath>

namespace gimbal_control
{

namespace
{
float clampf(float v, float lo, float hi)
{
  return std::max(lo, std::min(v, hi));
}

float lerp(float a, float b, float t)
{
  return a + (b - a) * t;
}
}  // namespace

GimbalTelemetry telemetry_validate(
  GimbalTelemetry t, const fsm_state_machine::msg::TimeoutFlags & flags)
{
  if (flags.ekf_timeout || flags.alt_timeout) {
    t.valid = false;
  }
  if (std::isnan(t.delta_h) || std::isnan(t.d_horiz)) {
    t.valid = false;
  }
  return t;
}

float target_angle_search()
{
  return 0.0f;
}

float target_angle_follow(const GimbalTelemetry & t)
{
  return -std::atan2(t.delta_h, t.d_horiz) * 180.0f / static_cast<float>(M_PI);
}

float target_angle_approach(const GimbalTelemetry & t)
{
  return -std::atan2(t.delta_h, t.d_horiz) * 180.0f / static_cast<float>(M_PI);
}

float target_angle_land(const GimbalTelemetry & t, float land_entry_height)
{
  const float ratio = clampf(1.0f - t.delta_h / land_entry_height, 0.0f, 1.0f);
  return lerp(-60.0f, -90.0f, ratio);
}

float gimbal_target_angle(GimbalPhase phase, const GimbalTelemetry & t, float land_entry_height)
{
  switch (phase) {
    case GimbalPhase::SEARCH:
      return target_angle_search();
    case GimbalPhase::FOLLOW:
      return target_angle_follow(t);
    case GimbalPhase::APPROACH:
      return target_angle_approach(t);
    case GimbalPhase::LAND:
      return target_angle_land(t, land_entry_height);
    default:
      return 0.0f;
  }
}

void pid_reset(PidState & pid)
{
  pid.integral = 0.0f;
  pid.prev_error = 0.0f;
}

float pid_update(PidState & pid, float target, float current, float dt)
{
  if (dt <= 0.0f) {
    return current;
  }
  const float error = target - current;
  pid.integral += error * dt;
  const float derivative = (error - pid.prev_error) / dt;
  float output = pid.kp * error + pid.ki * pid.integral + pid.kd * derivative;
  output = clampf(output, pid.out_min, pid.out_max);
  pid.prev_error = error;
  return current + output * dt;
}

float clamp_slew_rate(float prev_angle, float new_angle, float max_rate_deg_s, float dt)
{
  const float max_delta = max_rate_deg_s * dt;
  const float delta = clampf(new_angle - prev_angle, -max_delta, max_delta);
  return prev_angle + delta;
}

}  // namespace gimbal_control