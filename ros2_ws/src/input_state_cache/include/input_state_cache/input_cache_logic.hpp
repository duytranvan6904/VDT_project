#pragma once
#include "input_state_cache/input_cache_types.hpp"

namespace input_state_cache
{

SnapshotData build_snapshot(
  const RawSensors & raw,
  const TopicFreshness & fresh,
  const TimeoutThresholds & th,
  double now);

}  // namespace input_state_cache