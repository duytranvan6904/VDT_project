#pragma once
#include <cstdint>

namespace offboard_manager
{

enum class FsmState : uint8_t
{
  SEARCH = 0,
  FOLLOW = 1,
  APPROACH = 2,
  LAND = 3,
  COMPLETE = 4
};

enum class SetpointType
{
  POSITION,
  VELOCITY
};

struct PlannerOutput
{
  float vx = 0.0f;
  float vy = 0.0f;
  float vz = 0.0f;
  float yaw = 0.0f;
};

struct Setpoint
{
  SetpointType type = SetpointType::VELOCITY;
  float x = 0.0f;
  float y = 0.0f;
  float z = 0.0f;
  float vx = 0.0f;
  float vy = 0.0f;
  float vz = 0.0f;
  float yaw = 0.0f;
  float yaw_rate = 0.0f;
  bool use_yaw_rate = false;
};

struct OffboardContext
{
  double last_heartbeat_time = 0.0;
  bool armed = false;
  bool offboard_active = false;
  int engage_counter = 0;
  Setpoint setpoint;
};

}  // namespace offboard_manager