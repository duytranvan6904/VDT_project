#include "input_state_cache/input_cache_logic.hpp"

namespace input_state_cache
{

msg::TimeoutFlags compute_timeout_flags(
  const TopicFreshness & fresh, const TimeoutThresholds & th, double now)
{
  msg::TimeoutFlags flags;
  flags.ekf_timeout = (now - fresh.last_ekf_time) > th.ekf_timeout_sec;
  flags.vision_timeout = (now - fresh.last_vision_time) > th.vision_timeout_sec;
  flags.alt_timeout = (now - fresh.last_alt_time) > th.alt_timeout_sec;
  flags.planner_timeout = (now - fresh.last_planner_time) > th.planner_timeout_sec;
  return flags;
}

}  // namespace input_state_cache