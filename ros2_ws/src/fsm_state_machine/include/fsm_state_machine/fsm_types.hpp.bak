#pragma once
#include <cstdint>
#include <limits>

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
  float yaw_rate{std::numeric_limits<float>::quiet_NaN()};
  bool geometry_valid{false};
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
  int land_ok_count{0};
  bool land_inhibit{false};
};

struct TimeoutFlags
{
  bool ekf_timeout = false;
  bool vision_timeout = false;
  bool planner_timeout = false;
};

struct FsmContext
{
  State state = State::SEARCH;
  Counters counters;
  double last_transition_time = 0.0;
};

struct FsmParams
{
  int enter_follow_cycles{10};
  float follow_lost_timeout{2.5f};
  float approach_lost_timeout{1.5f};
  float align_threshold{0.3f};
  float land_entry_height{0.5f};
  int land_entry_cycles{5};
  float yaw_settle_rate{0.1f};
  bool land_requires_rearm{true};
  bool ignore_planner_timeout{false};
};

}  // namespace fsm_state_machine