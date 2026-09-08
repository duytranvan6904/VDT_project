#pragma once

namespace input_state_cache
{

struct TopicFreshness
{
  double last_ekf_time = 0.0;
  double last_vision_time = 0.0;
  double last_alt_time = 0.0;
  double last_planner_time = 0.0;
};

struct TimeoutThresholds
{
  double ekf_timeout_sec = 0.5;
  double vision_timeout_sec = 0.5;
  double alt_timeout_sec = 0.5;
  double planner_timeout_sec = 1.0;
};

}  // namespace input_state_cache