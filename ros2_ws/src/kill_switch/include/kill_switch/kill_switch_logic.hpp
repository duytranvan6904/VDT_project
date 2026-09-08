#pragma once
#include "kill_switch/kill_switch_types.hpp"

namespace kill_switch
{

bool kill_switch_debounce(KillSwitchContext & ctx, bool raw_triggered, int threshold);

}  // namespace kill_switch