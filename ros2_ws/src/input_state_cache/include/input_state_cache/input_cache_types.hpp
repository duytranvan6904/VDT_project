#pragma once
#include <cstdint>

namespace input_state_cache
{

struct RawEkf {
  float x = 0.0f, y = 0.0f, z = 0.0f;
  float vx = 0.0f, vy = 0.0f, vz = 0.0f;
  bool received = false;
};

struct RawTracking {
  bool expired = true;
  bool received = false;
};

struct RawOdom {
  float x = 0.0f, y = 0.0f, z = 0.0f;
  bool received = false;
};

struct RawVision {
  bool marker_visible = false;
  float pixel_align_error = 0.0f;
  bool received = false;
};

struct RawAlt {
  float altitude = 0.0f;
  bool touchdown_flag = false;
  bool received = false;
};

struct RawSensors {
  RawEkf ekf;
  RawTracking tracking;
  RawOdom odom;
  RawVision vision;
  RawAlt alt;
};

struct TopicFreshness {
  double last_ekf_time = 0.0;
  double last_tracking_time = 0.0;
  double last_odom_time = 0.0;
  double last_vision_time = 0.0;
  double last_alt_time = 0.0;
  double last_planner_time = 0.0;
};

struct TimeoutThresholds {
  double ekf_timeout_sec = 0.5;
  double odom_timeout_sec = 0.5;
  double vision_timeout_sec = 0.5;
  double alt_timeout_sec = 0.5;
  double planner_timeout_sec = 1.0;
};

struct SnapshotData {
  bool ekf_valid = false;
  bool ekf_timeout = true;
  bool tracking_timeout = true;
  bool ekf_expired = true;
  float pos_x = 0.0f, pos_y = 0.0f, pos_z = 0.0f;
  float vel_x = 0.0f, vel_y = 0.0f, vel_z = 0.0f;

  bool odom_valid = false;
  bool odom_timeout = true;

  bool vision_valid = false;
  bool vision_timeout = true;
  bool marker_detected = false;
  float pixel_align_error = 0.0f;

  bool alt_valid = false;
  bool alt_timeout = true;
  float altitude = 0.0f;
  bool touchdown_flag = false;
  bool touchdown = false;

  bool planner_timeout = true;

  float delta_h = 0.0f;
  float d_horiz = 0.0f;
  float align_error = 0.0f;
  bool valid = false;
  bool all_sensors_valid = false;
};

}  // namespace input_state_cache