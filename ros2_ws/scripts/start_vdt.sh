#!/usr/bin/env bash
SESSION=vdt
DEV=/dev/ttyACM0
T_NSH=30
D_AGENT=8
D_RC=8
D_TAKEOFF=15
D_SYSTEM=15
D_STOP=5

SRC='source /opt/ros/$ROS_DISTRO/setup.bash; source ~/ros2_ws/install/setup.bash'
WS='cd ~/VDT_project/ros2_ws && flock /tmp/vdt_build.lock colcon build && source install/setup.bash'

send() { tmux send-keys -t "$SESSION:$1" "$2" Enter; }
win() { tmux new-window -t "$SESSION" -n "$1"; send "$1" "$SRC"; }
has() { tmux capture-pane -p -t "$SESSION:$1" | grep -q "$2"; }

start() {
  tmux has-session -t "$SESSION" 2>/dev/null && { echo "session $SESSION dang chay"; exit 1; }
  tmux new-session -d -s "$SESSION" -n px4
  send px4 "$SRC"
  send px4 "python3 mavlink_shell.py $DEV --baudrate 57600"
  for _ in $(seq "$T_NSH"); do
    sleep 1
    tmux send-keys -t "$SESSION:px4" Enter
    has px4 'nsh>' && break
  done
  has px4 'nsh>' || { echo "khong vao duoc nsh>"; exit 1; }
  send px4 "uxrce_dds_client start -t serial -d $DEV -b 921600"
  sleep 1
  send px4 "mavlink stop -d $DEV"
  sleep 1
  tmux send-keys -t "$SESSION:px4" C-c
  sleep 1
  send px4 "MicroXRCEAgent serial --dev $DEV -b 921600 -v6"
  sleep "$D_AGENT"

  win rc
  send rc "$WS && python3 land_arbiter.py --msg-type vdt_msgs/msg/RcFsmInput"
  sleep "$D_RC"

  win takeoff
  send takeoff "$WS && ./takeoff.sh --alt 3"
  sleep "$D_TAKEOFF"

  win system
  send system "$WS && ros2 launch bringup vdt_system.launch.py start_hardware:=false"
  sleep "$D_SYSTEM"

  win check
  send check "$WS"
  send check 'ros2 topic list | grep fmu/out; ros2 topic info /rc/fsm_input; ros2 topic echo takeoff_cmd/state --once; python3 preflight_check.py; echo "exit=$?"'
  tmux send-keys -t "$SESSION:check" "./cmd.sh takeoff"

  win land
  send land "cd ~/VDT_project/ros2_ws"
  tmux send-keys -t "$SESSION:land" "./cmd.sh emergency_land"

  tmux select-window -t "$SESSION:check"
  tmux attach -t "$SESSION"
}

stop() {
  for w in system takeoff rc px4; do
    tmux send-keys -t "$SESSION:$w" C-c
    sleep "$D_STOP"
  done
  tmux kill-session -t "$SESSION"
}

case "$1" in
  stop) stop ;;
  *) start ;;
esac