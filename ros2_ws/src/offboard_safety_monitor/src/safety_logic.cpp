#include "offboard_safety_monitor/safety_logic.hpp"

namespace offboard_safety_monitor
{

FailsafeLevel check_battery_failsafe(float remaining_frac, const SafetyThresholds & th)
{
  if (remaining_frac < th.battery_critical_frac) {
    return FailsafeLevel::BATTERY_CRITICAL;
  }
  if (remaining_frac < th.battery_warning_frac) {
    return FailsafeLevel::BATTERY_WARNING;
  }
  return FailsafeLevel::NONE;
}

bool check_ekf_health(bool xy_valid, bool z_valid)
{
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