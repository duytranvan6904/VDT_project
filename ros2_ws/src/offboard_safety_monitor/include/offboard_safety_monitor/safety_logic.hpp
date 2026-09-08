#pragma once
#include "offboard_safety_monitor/safety_types.hpp"

namespace offboard_safety_monitor
{

bool is_fresh(bool has_data, double last_received_time, double now, double timeout_sec);

FailsafeLevel check_battery_failsafe(
  float remaining_frac, bool battery_fresh, const SafetyThresholds & th);
bool update_force_land_request(
  bool current, bool latched, FailsafeLevel level,
  float remaining_frac, bool battery_fresh, const SafetyThresholds & th);
bool check_ekf_health(bool xy_valid, bool z_valid, bool ekf_fresh);
bool check_rc_override(bool offboard_active, uint8_t nav_state, uint8_t nav_state_offboard);
FailsafeLevel offboard_watchdog_escalate(float heartbeat_age_sec, const SafetyThresholds & th);

FailsafeLevel evaluate_failsafe_level(
  bool rc_override, bool ekf_healthy,
  FailsafeLevel battery_level, FailsafeLevel escalate_level);

}  // namespace offboard_safety_monitor