#include "kill_switch/kill_switch_logic.hpp"

namespace kill_switch
{

bool kill_switch_debounce(KillSwitchContext & ctx, bool raw_triggered, int threshold)
{
  if (raw_triggered) {
    ctx.debounce_count += 1;
  } else {
    ctx.debounce_count = 0;
  }
  return ctx.debounce_count >= threshold;
}

}  // namespace kill_switch