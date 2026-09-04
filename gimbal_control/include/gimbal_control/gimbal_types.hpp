#pragma once
#include <cstdint>

namespace gimbal_control
{

enum class GimbalPhase : uint8_t
{
  SEARCH = 0,
  FOLLOW = 1,
  APPROACH = 2,
  LAND = 3,
  COMPLETE = 4
};

struct GimbalTelemetry
{
  float delta_h = 0.0f;
  float d_horiz = 0.0f;
  float altitude = 0.0f;
  bool valid = true;
};

struct PidState
{
  float kp = 0.0f;
  float ki = 0.0f;
  float kd = 0.0f;
  float integral = 0.0f;
  float prev_error = 0.0f;
  float out_min = -90.0f;
  float out_max = 90.0f;
};

}  // namespace gimbal_control