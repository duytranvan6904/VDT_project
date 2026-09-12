#include "input_state_cache/input_cache_logic.hpp"
#include <cmath>

namespace input_state_cache
{

SnapshotData build_snapshot(
  const RawSensors & raw,
  const TopicFreshness & fresh,
  const TimeoutThresholds & th,
  double now)
{
  SnapshotData s;

  s.ekf_timeout = (!raw.ekf.received) || ((now - fresh.last_ekf_time) > th.ekf_timeout_sec);
  s.vision_timeout = (!raw.vision.received) || ((now - fresh.last_vision_time) > th.vision_timeout_sec);
  s.alt_timeout = (!raw.alt.received) || ((now - fresh.last_alt_time) > th.alt_timeout_sec);
  s.planner_timeout = (now - fresh.last_planner_time) > th.planner_timeout_sec;

  s.pos_x = raw.ekf.x; s.pos_y = raw.ekf.y; s.pos_z = raw.ekf.z;
  s.vel_x = raw.ekf.vx; s.vel_y = raw.ekf.vy; s.vel_z = raw.ekf.vz;
  s.ekf_valid = !s.ekf_timeout && std::isfinite(s.pos_x) && std::isfinite(s.pos_y) && std::isfinite(s.pos_z);

  s.marker_detected = raw.vision.marker_visible;
  s.pixel_align_error = raw.vision.pixel_align_error;
  s.vision_valid = !s.vision_timeout && std::isfinite(s.pixel_align_error);

  s.altitude = raw.alt.altitude;
  s.touchdown_flag = raw.alt.touchdown_flag;
  s.alt_valid = !s.alt_timeout && std::isfinite(s.altitude);

  if (s.ekf_valid && s.alt_valid) {
    s.delta_h = s.pos_z - s.altitude;
  }
  if (s.ekf_valid) {
    s.d_horiz = std::hypot(s.pos_x, s.pos_y);
  }

  s.all_sensors_valid = s.ekf_valid && s.vision_valid && s.alt_valid;

  return s;
}

}  // namespace input_state_cache