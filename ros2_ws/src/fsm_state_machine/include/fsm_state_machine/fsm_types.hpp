#pragma once
#include <cstdint>

namespace fsm_state_machine
{

enum class State : uint8_t
{
  SEARCH = 0,
  FOLLOW = 1,
  APPROACH = 2,
  LAND = 3,
  COMPLETE = 4
};

struct SensorInput
{
  bool marker_detected = false;
  float align_error = 0.0f;
  float altitude = 0.0f;
  float delta_h = 0.0f;
  float d_horiz = 0.0f;
  bool touchdown = false;
  bool valid = true;
};

struct RcInput
{
  bool land_switch = false;
  bool kill_switch = false;
};

struct Counters
{
  int marker_stable_count = 0;
  float marker_lost_time = 0.0f;
};

struct TimeoutFlags
{
  bool ekf_timeout = false;
  bool vision_timeout = false;
};

struct FsmContext
{
  State state = State::SEARCH;
  Counters counters;
  double last_transition_time = 0.0;
};

}  // namespace fsm_state_machine