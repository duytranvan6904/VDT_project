#include "offboard_safety_monitor/safety_logic.hpp"
#include <cmath>

namespace offboard_safety_monitor
{

bool is_fresh(bool has_data, double last_received_time, double now, double timeout_sec)
{
  if (!has_data) {
    return false;
  }
  return (now - last_received_time) <= timeout_sec;
}

FailsafeLevel check_battery_failsafe(
  float remaining_frac, bool battery_fresh, const SafetyThresholds & th)
{
  if (!battery_fresh) {
    return FailsafeLevel::NONE;
  }
  if (!std::isfinite(remaining_frac) || remaining_frac < 0.0f || remaining_frac > 1.0f) {
    return FailsafeLevel::NONE;
  }
  if (remaining_frac < th.battery_critical_frac) {
    return FailsafeLevel::BATTERY_CRITICAL;
  }
  if (remaining_frac < th.battery_warning_frac) {
    return FailsafeLevel::BATTERY_WARNING;
  }
  return FailsafeLevel::NONE;
}

bool check_ekf_health(bool xy_valid, bool z_valid, bool ekf_fresh)
{
  if (!ekf_fresh) {
    return true;
  }
  return xy_valid && z_valid;
}

bool check_rc_override(bool offboard_active, uint8_t nav_state, uint8_t nav_state_offboard)
{
  return offboard_active && (nav_state != nav_state_offboard);
}

FailsafeLevel offboard_watchdog_escalate(float heartbeat_age_sec, const SafetyThresholds & th)
{
  if (heartbeat_age_sec > th.offboard_rtl_timeout) {
    return FailsafeLevel::OFFBOARD_LOST_LONG;
  }
  if (heartbeat_age_sec > th.offboard_hold_timeout) {
    return FailsafeLevel::OFFBOARD_LOST_SHORT;
  }
  return FailsafeLevel::NONE;
}

FailsafeLevel evaluate_failsafe_level(
  bool rc_override, bool ekf_healthy,
  FailsafeLevel battery_level, FailsafeLevel escalate_level)
{
  if (rc_override) {
    return FailsafeLevel::RC_OVERRIDE;
  }
  if (!ekf_healthy) {
    return FailsafeLevel::EKF_UNHEALTHY;
  }
  if (battery_level != FailsafeLevel::NONE) {
    return battery_level;
  }
  return escalate_level;
}

}  // namespace offboard_safety_monitor