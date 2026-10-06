#include <cassert>
#include <cmath>
#include <cstdio>
#include <cstring>
#include "vision_interface_bridge/bridge_logic.hpp"
using namespace vision_interface_bridge;

int main()
{
  assert(std::fabs(alignment_error(320, 240, 640, 480)) < 1e-6f);
  assert(std::fabs(alignment_error(640, 240, 640, 480) - 1.0f) < 1e-6f);
  assert(std::fabs(alignment_error(640, 480, 640, 480) - std::sqrt(2.0f)) < 1e-5f);
  assert(std::isnan(alignment_error(320, 240, 0, 480)));
  assert(std::isnan(alignment_error(std::nan(""), 240, 640, 480)));

  assert(!std::strcmp(phase_name(0), "SEARCH"));
  assert(!std::strcmp(phase_name(1), "FOLLOW"));
  assert(!std::strcmp(phase_name(2), "APPROACH"));
  assert(!std::strcmp(phase_name(3), "LAND"));
  assert(!std::strcmp(phase_name(4), "IDLE"));
  assert(!std::strcmp(phase_name(200), "IDLE"));

  std::puts("all ok");
  return 0;
}