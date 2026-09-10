#pragma once
#include "input_state_cache/input_cache_types.hpp"
#include "input_state_cache/msg/timeout_flags.hpp"

namespace input_state_cache
{

msg::TimeoutFlags compute_timeout_flags(
  const TopicFreshness & fresh, const TimeoutThresholds & th, double now);

}  // namespace input_state_cache