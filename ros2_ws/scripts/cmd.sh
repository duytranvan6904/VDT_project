#!/usr/bin/env bash
set -e
case "$1" in
  takeoff|land|emergency_land) ;;
  *) echo "usage: $0 takeoff|land|emergency_land" >&2; exit 2 ;;
esac
ROS_SETUP="$(ls -d /opt/ros/*/setup.bash | head -n1)"
WS="${VDT_WS:-$HOME/VDT_project/ros2_ws}"
source "$ROS_SETUP"
source "$WS/install/setup.bash"
exec ros2 topic pub -t 5 -r 5 /takeoff_cmd/command std_msgs/msg/String "{data: $1}"
