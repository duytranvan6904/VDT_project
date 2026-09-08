#include "gimbal_control/gimbal_logic.hpp"
#include <algorithm>
#include <cmath>

namespace gimbal_control
{

namespace
{
float clampf(float v, float lo, float hi)
{
  if (!std::isfinite(v)) {
    return lo;
  }
  return std::max(lo, std::min(v, hi));
}

float lerp(float a, float b, float t)
{
  if (!std::isfinite(a) || !std::isfinite(b) || !std::isfinite(t)) {
    return 0.0f;
  }
  return a + (b - a) * t;
}
}  // namespace

GimbalTelemetry telemetry_validate(
  GimbalTelemetry t, const fsm_state_machine::msg::TimeoutFlags & flags)
{
  if (flags.ekf_timeout || flags.alt_timeout) {
    t.valid = false;
  }
  if (!std::isfinite(t.delta_h) || !std::isfinite(t.d_horiz) ||
    !std::isfinite(t.altitude))
  {
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
  if (!t.valid || !std::isfinite(t.delta_h) || !std::isfinite(t.d_horiz)) {
    return 0.0f;
  }
  return -std::atan2(t.delta_h, t.d_horiz) * 180.0f / static_cast<float>(M_PI);
}

float target_angle_approach(const GimbalTelemetry & t)
{
  if (!t.valid || !std::isfinite(t.delta_h) || !std::isfinite(t.d_horiz)) {
    return 0.0f;
  }
  return -std::atan2(t.delta_h, t.d_horiz) * 180.0f / static_cast<float>(M_PI);
}

float target_angle_land(const GimbalTelemetry & t, float land_entry_height)
{
  if (!std::isfinite(land_entry_height) || land_entry_height <= 0.0f) {
    return -60.0f;
  }
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
  if (!std::isfinite(target) || !std::isfinite(current) || !std::isfinite(dt) ||
    dt <= 0.0f || !std::isfinite(pid.kp) || !std::isfinite(pid.ki) ||
    !std::isfinite(pid.kd) || !std::isfinite(pid.integral) ||
    !std::isfinite(pid.prev_error) || !std::isfinite(pid.out_min) ||
    !std::isfinite(pid.out_max) || pid.out_min > pid.out_max)
  {
    pid_reset(pid);
    return current;
  }
  const float error = target - current;
  const float derivative = (error - pid.prev_error) / dt;
  if (!std::isfinite(derivative)) {
    pid_reset(pid);
    return current;
  }
  const float candidate_integral = pid.integral + error * dt;
  const float unsaturated_output =
    pid.kp * error + pid.ki * candidate_integral + pid.kd * derivative;
  if (!std::isfinite(candidate_integral) || !std::isfinite(unsaturated_output)) {
    pid_reset(pid);
    return current;
  }
  const bool saturating_high = unsaturated_output > pid.out_max && error > 0.0f;
  const bool saturating_low = unsaturated_output < pid.out_min && error < 0.0f;
  if (!saturating_high && !saturating_low) {
    pid.integral = candidate_integral;
  }
  float output = clampf(
    pid.kp * error + pid.ki * pid.integral + pid.kd * derivative,
    pid.out_min, pid.out_max);
  pid.prev_error = error;
  const float next = current + output * dt;
  return std::isfinite(next) ? next : current;
}

float clamp_slew_rate(float prev_angle, float new_angle, float max_rate_deg_s, float dt)
{
  if (!std::isfinite(prev_angle) || !std::isfinite(new_angle) ||
    !std::isfinite(max_rate_deg_s) || !std::isfinite(dt) ||
    max_rate_deg_s < 0.0f || dt <= 0.0f)
  {
    return prev_angle;
  }
  const float max_delta = max_rate_deg_s * dt;
  const float delta = clampf(new_angle - prev_angle, -max_delta, max_delta);
  return prev_angle + delta;
}

}  // namespace gimbal_control