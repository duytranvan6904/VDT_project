#pragma once
#include <array>

namespace rc_parser
{

enum class SwitchPos
{
  LOW,
  MID,
  HIGH
};

struct RcConfig
{
  int land_channel = 4;
  int kill_channel = 5;
  int low_threshold = 1200;
  int high_threshold = 1800;
  double frame_timeout = 0.5;
};

struct RcChannels
{
  std::array<int, 16> ch{};
  bool valid = false;
  bool failsafe = false;
};

}  // namespace rc_parser