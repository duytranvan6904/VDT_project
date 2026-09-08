#pragma once
#include <cstdint>

namespace offboard_safety_monitor
{

enum class FailsafeLevel : uint8_t
{
  NONE = 0,
  RC_OVERRIDE = 1,
  EKF_UNHEALTHY = 2,
  BATTERY_WARNING = 3,
  BATTERY_CRITICAL = 4,
  OFFBOARD_LOST_SHORT = 5,
  OFFBOARD_LOST_LONG = 6
};

struct SafetyThresholds
{
  float battery_warning_frac = 0.3f;
  float battery_critical_frac = 0.15f;
  double offboard_hold_timeout = 1.0;
  double offboard_rtl_timeout = 5.0;
};

struct SafetyContext
{
  FailsafeLevel active_failsafe = FailsafeLevel::NONE;
  bool force_land_requested = false;
};

}  // namespace offboard_safety_monitor