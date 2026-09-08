#pragma once

namespace kill_switch
{

struct KillSwitchContext
{
  bool triggered = false;
  double last_trigger_time = 0.0;
  int debounce_count = 0;
};

}  // namespace kill_switch