#include <cassert>
#include <cmath>
#include <limits>
#include "offboard_safety_monitor/safety_logic.hpp"

using namespace offboard_safety_monitor;

int main()
{
  SafetyThresholds thresholds;
  assert(check_battery_failsafe(std::numeric_limits<float>::quiet_NaN(), true, thresholds) == FailsafeLevel::NONE);
  assert(check_battery_failsafe(-0.1f, true, thresholds) == FailsafeLevel::NONE);
  assert(check_battery_failsafe(0.1f, true, thresholds) == FailsafeLevel::BATTERY_CRITICAL);
  assert(check_battery_failsafe(0.1f, false, thresholds) == FailsafeLevel::NONE);
  assert(update_force_land_request(false, true, FailsafeLevel::BATTERY_WARNING, 0.2f, true, thresholds));
  assert(update_force_land_request(true, true, FailsafeLevel::NONE, 0.9f, true, thresholds));
  assert(!update_force_land_request(true, false, FailsafeLevel::NONE, 0.9f, true, thresholds));
  assert(update_force_land_request(true, false, FailsafeLevel::NONE, 0.9f, false, thresholds));
  assert(check_ekf_health(false, false, false));
  assert(!check_ekf_health(false, true, true));
  return 0;
}
