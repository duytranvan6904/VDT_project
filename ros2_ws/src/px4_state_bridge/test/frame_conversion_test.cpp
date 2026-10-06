#include <cassert>
#include <cmath>
#include <cstdio>
#include "px4_state_bridge/frame_conversion.hpp"
using namespace px4_state_bridge;

static bool near(double a, double b) { return std::fabs(a - b) < 1e-6; }
static bool near3(const Vec3 & a, const Vec3 & b)
{
  return near(a[0], b[0]) && near(a[1], b[1]) && near(a[2], b[2]);
}

int main()
{
  const Vec3 p = ned_to_enu({1.0, 2.0, -3.0});
  assert(near3(p, {2.0, 1.0, 3.0}));

  Quat q = frd_ned_to_flu_enu({1.0, 0.0, 0.0, 0.0});
  assert(near(yaw_of(q), kPi / 2.0));

  q = frd_ned_to_flu_enu(quat_from_rpy(0.0, 0.0, kPi / 2.0));
  assert(near(yaw_of(q), 0.0));

  q = frd_ned_to_flu_enu(quat_from_rpy(0.0, 10.0 * kPi / 180.0, 0.0));
  assert(near(pitch_of(q), -10.0 * kPi / 180.0));

  const Quat facing_east = frd_ned_to_flu_enu(quat_from_rpy(0.0, 0.0, kPi / 2.0));
  const Quat opt = optical_from_link();
  assert(near3(quat_rotate(opt, {0, 0, 1}), {1, 0, 0}));
  assert(near3(quat_rotate(opt, {1, 0, 0}), {0, -1, 0}));
  assert(near3(quat_rotate(opt, {0, 1, 0}), {0, 0, -1}));

  const double deg[] = {0.0, -30.0, -90.0};
  for (double d : deg) {
    const Quat total = quat_mul(quat_mul(facing_east, gimbal_rotation(d)), opt);
    const Vec3 fwd = quat_rotate(total, {0, 0, 1});
    const double a = d * kPi / 180.0;
    assert(near3(fwd, {std::cos(a), 0.0, std::sin(a)}));
  }

  std::puts("all ok");
  return 0;
}