#include "input_state_cache/input_cache_logic.hpp"
#include <cmath>
#include <limits>

namespace input_state_cache
{

namespace
{
constexpr float kNaN = std::numeric_limits<float>::quiet_NaN();

bool is_stale(bool received, double last, double now, double limit)
{
  return !received || (now - last) > limit;
}
}  // namespace

SnapshotData build_snapshot(
  const RawSensors & raw,
  const TopicFreshness & fresh,
  const TimeoutThresholds & th,
  double now)
{
  SnapshotData s;

  s.ekf_timeout = is_stale(raw.ekf.received, fresh.last_ekf_time, now, th.ekf_timeout_sec);
  s.tracking_timeout =
    is_stale(raw.tracking.received, fresh.last_tracking_time, now, th.ekf_timeout_sec);
  s.odom_timeout = is_stale(raw.odom.received, fresh.last_odom_time, now, th.odom_timeout_sec);
  s.vision_timeout =
    is_stale(raw.vision.received, fresh.last_vision_time, now, th.vision_timeout_sec);
  s.alt_timeout = is_stale(raw.alt.received, fresh.last_alt_time, now, th.alt_timeout_sec);
  s.planner_timeout =
    fresh.last_planner_time <= 0.0 || (now - fresh.last_planner_time) > th.planner_timeout_sec;

  s.pos_x = raw.ekf.x; s.pos_y = raw.ekf.y; s.pos_z = raw.ekf.z;
  s.vel_x = raw.ekf.vx; s.vel_y = raw.ekf.vy; s.vel_z = raw.ekf.vz;
  s.ekf_expired = raw.tracking.expired;
  s.ekf_valid = !s.ekf_timeout && !s.tracking_timeout && !s.ekf_expired &&
    std::isfinite(s.pos_x) && std::isfinite(s.pos_y) && std::isfinite(s.pos_z);

  s.odom_valid = !s.odom_timeout && std::isfinite(raw.odom.x) &&
    std::isfinite(raw.odom.y) && std::isfinite(raw.odom.z);

  s.vision_valid = !s.vision_timeout;
  s.pixel_align_error = raw.vision.pixel_align_error;
  s.marker_detected = s.vision_valid && raw.vision.marker_visible &&
    std::isfinite(raw.vision.pixel_align_error);

  s.alt_valid = !s.alt_timeout && std::isfinite(raw.alt.altitude);
  s.altitude = s.alt_valid ? raw.alt.altitude : kNaN;
  s.touchdown_flag = raw.alt.touchdown_flag;
  s.touchdown = s.alt_valid && raw.alt.touchdown_flag;

  s.delta_h = kNaN;
  s.d_horiz = kNaN;
  s.align_error = kNaN;
  if (s.ekf_valid && s.odom_valid) {
    const float dx = s.pos_x - raw.odom.x;
    const float dy = s.pos_y - raw.odom.y;
    const float d_horiz = std::hypot(dx, dy);
    const float delta_h = raw.odom.z - s.pos_z;
    if (std::isfinite(d_horiz) && std::isfinite(delta_h)) {
      s.d_horiz = d_horiz;
      s.delta_h = delta_h;
      s.align_error = d_horiz;
    }
  }

  s.valid = s.odom_valid && s.alt_valid;
  s.all_sensors_valid = s.valid && s.ekf_valid && s.vision_valid;

  return s;
}

}  // namespace input_state_cache