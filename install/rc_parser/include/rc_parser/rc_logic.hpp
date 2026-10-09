#pragma once
#include "rc_parser/rc_types.hpp"

namespace rc_parser
{

RcChannels rc_validate(
  RcChannels channels, const RcConfig & cfg, double last_valid_time, double now);
SwitchPos rc_get_switch_pos(const RcChannels & channels, int ch_idx, const RcConfig & cfg);
bool rc_get_land_trigger(const RcChannels & channels, const RcConfig & cfg);
bool rc_get_kill_switch(const RcChannels & channels, const RcConfig & cfg);

}  // namespace rc_parser