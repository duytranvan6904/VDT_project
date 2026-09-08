#include "rc_parser/rc_logic.hpp"

namespace rc_parser
{

RcChannels rc_validate(
  RcChannels channels, const RcConfig & cfg, double last_valid_time, double now)
{
  if (last_valid_time <= 0.0 || now - last_valid_time > cfg.frame_timeout) {
    channels.valid = false;
  }
  if (channels.failsafe) {
    channels.valid = false;
  }
  for (int v : channels.ch) {
    if (v < 800 || v > 2200) {
      channels.valid = false;
    }
  }
  return channels;
}

SwitchPos rc_get_switch_pos(const RcChannels & channels, int ch_idx, const RcConfig & cfg)
{
  const int val = channels.ch[ch_idx];
  if (val < cfg.low_threshold) {
    return SwitchPos::LOW;
  }
  if (val > cfg.high_threshold) {
    return SwitchPos::HIGH;
  }
  return SwitchPos::MID;
}

bool rc_get_land_trigger(const RcChannels & channels, const RcConfig & cfg)
{
  return rc_get_switch_pos(channels, cfg.land_channel, cfg) == SwitchPos::HIGH;
}

bool rc_get_kill_switch(const RcChannels & channels, const RcConfig & cfg)
{
  return rc_get_switch_pos(channels, cfg.kill_channel, cfg) == SwitchPos::HIGH;
}

}  // namespace rc_parser