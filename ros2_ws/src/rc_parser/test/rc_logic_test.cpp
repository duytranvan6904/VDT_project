#include <cassert>
#include "rc_parser/rc_logic.hpp"

using namespace rc_parser;

int main()
{
  RcConfig config;
  RcChannels channels;
  channels.valid = true;
  channels.ch.fill(1500);
  const RcChannels before_first_frame = rc_validate(channels, config, 0.0, 0.1);
  assert(!before_first_frame.valid);

  const RcChannels fresh = rc_validate(channels, config, 1.0, 1.1);
  assert(fresh.valid);

  const RcChannels stale = rc_validate(channels, config, 1.0, 1.6);
  assert(!stale.valid);
  return 0;
}
