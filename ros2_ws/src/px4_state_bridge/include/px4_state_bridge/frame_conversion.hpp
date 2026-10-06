#pragma once
#include <array>
#include <cmath>

namespace px4_state_bridge
{

using Quat = std::array<double, 4>;
using Vec3 = std::array<double, 3>;

constexpr double kPi = 3.14159265358979323846;

inline Quat quat_mul(const Quat & a, const Quat & b)
{
  return {
    a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3],
    a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2],
    a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1],
    a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0]};
}

inline Quat quat_conj(const Quat & q)
{
  return {q[0], -q[1], -q[2], -q[3]};
}

inline double quat_norm(const Quat & q)
{
  return std::sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
}

inline Quat quat_normalize(const Quat & q)
{
  const double n = quat_norm(q);
  return {q[0] / n, q[1] / n, q[2] / n, q[3] / n};
}

inline Quat quat_from_rpy(double roll, double pitch, double yaw)
{
  const Quat qx = {std::cos(roll / 2.0), std::sin(roll / 2.0), 0.0, 0.0};
  const Quat qy = {std::cos(pitch / 2.0), 0.0, std::sin(pitch / 2.0), 0.0};
  const Quat qz = {std::cos(yaw / 2.0), 0.0, 0.0, std::sin(yaw / 2.0)};
  return quat_mul(quat_mul(qz, qy), qx);
}

inline Vec3 quat_rotate(const Quat & q, const Vec3 & v)
{
  const Quat p = {0.0, v[0], v[1], v[2]};
  const Quat r = quat_mul(quat_mul(q, p), quat_conj(q));
  return {r[1], r[2], r[3]};
}

inline Vec3 ned_to_enu(const Vec3 & v)
{
  return {v[1], v[0], -v[2]};
}

inline Vec3 frd_to_flu(const Vec3 & v)
{
  return {v[0], -v[1], -v[2]};
}

inline Quat frd_ned_to_flu_enu(const Quat & q_ned_frd)
{
  const Quat q_enu_ned = quat_from_rpy(kPi, 0.0, kPi / 2.0);
  const Quat q_frd_flu = quat_from_rpy(kPi, 0.0, 0.0);
  return quat_normalize(quat_mul(quat_mul(q_enu_ned, q_ned_frd), q_frd_flu));
}

inline Quat gimbal_rotation(double angle_deg)
{
  return quat_from_rpy(0.0, -angle_deg * kPi / 180.0, 0.0);
}

inline Quat optical_from_link()
{
  return quat_from_rpy(-kPi / 2.0, 0.0, -kPi / 2.0);
}

inline double yaw_of(const Quat & q)
{
  return std::atan2(
    2.0 * (q[0] * q[3] + q[1] * q[2]), 1.0 - 2.0 * (q[2] * q[2] + q[3] * q[3]));
}

inline double pitch_of(const Quat & q)
{
  return std::asin(2.0 * (q[0] * q[2] - q[3] * q[1]));
}

}  // namespace px4_state_bridge