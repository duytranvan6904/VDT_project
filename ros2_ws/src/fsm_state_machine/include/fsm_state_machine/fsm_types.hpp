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
  float follow_lost_timeout{5.0f};       // upgraded v2.4: 5.0s (was 2.5s)
  float approach_lost_timeout{7.0f};     // upgraded v2.4: 7.0s (was 1.5s)
  float align_threshold{0.3f};
  float land_entry_height{0.60f};        // upgraded v2.4: 0.60m (was 0.50m)
  float land_entry_rxy{0.85f};           // upgraded v2.4: 0.85m horizontal funnel
  int land_entry_cycles{5};
  float yaw_settle_rate{0.1f};
  bool land_requires_rearm{true};
  bool auto_reland_after_abort{false};   // upgraded v2.4: prevent automatic reland without fresh command
  bool ignore_planner_timeout{false};
};

}  // namespace fsm_state_machine
