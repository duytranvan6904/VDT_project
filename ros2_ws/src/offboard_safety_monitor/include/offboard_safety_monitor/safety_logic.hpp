#pragma once
#include "offboard_safety_monitor/safety_types.hpp"

namespace offboard_safety_monitor
{

FailsafeLevel check_battery_failsafe(float remaining_frac, const SafetyThresholds & th);
bool check_ekf_health(bool xy_valid, bool z_valid);
bool check_rc_override(bool offboard_active, uint8_t nav_state, uint8_t nav_state_offboard);
FailsafeLevel offboard_watchdog_escalate(float heartbeat_age_sec, const SafetyThresholds & th);

FailsafeLevel evaluate_failsafe_level(
  bool rc_override, bool ekf_healthy,
  FailsafeLevel battery_level, FailsafeLevel escalate_level);

}  // namespace offboard_safety_monitor