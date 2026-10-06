#pragma once
#include <cmath>
#include <cstdint>

namespace vision_interface_bridge
{

inline float alignment_error(double u, double v, double width, double height)
{
  if (!(width > 0.0) || !(height > 0.0) || !std::isfinite(u) || !std::isfinite(v)) {
    return std::nanf("");
  }
  const double eu = (u - width / 2.0) / (width / 2.0);
  const double ev = (v - height / 2.0) / (height / 2.0);
  return static_cast<float>(std::hypot(eu, ev));
}

inline const char * phase_name(uint8_t fsm_state)
{
  switch (fsm_state) {
    case 0: return "SEARCH";
    case 1: return "FOLLOW";
    case 2: return "APPROACH";
    case 3: return "LAND";
    default: return "IDLE";
  }
}

}  // namespace vision_interface_bridge