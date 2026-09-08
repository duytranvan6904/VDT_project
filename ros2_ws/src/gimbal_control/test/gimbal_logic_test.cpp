#include <cassert>
#include <cmath>
#include <limits>
#include "gimbal_control/gimbal_logic.hpp"

using namespace gimbal_control;

int main()
{
  GimbalTelemetry telemetry;
  telemetry.delta_h = std::numeric_limits<float>::infinity();
  assert(!telemetry_validate(telemetry, {}).valid);
  assert(target_angle_land({}, 0.0f) == -60.0f);

  PidState pid;
  pid.kp = 1.0f;
  pid.ki = 1.0f;
  pid.kd = 0.0f;
  pid.out_min = -1.0f;
  pid.out_max = 1.0f;
  const float first = pid_update(pid, 10.0f, 0.0f, 1.0f);
  const float integral_after_saturation = pid.integral;
  pid_update(pid, 10.0f, first, 1.0f);
  assert(pid.integral == integral_after_saturation);
  assert(std::isfinite(pid_update(pid, std::numeric_limits<float>::quiet_NaN(), 0.0f, 1.0f)));
  return 0;
}
