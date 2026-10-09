#!/usr/bin/env bash
set -e
ROS_SETUP="$(ls -d /opt/ros/*/setup.bash | head -n1)"
WS="${VDT_WS:-$HOME/ros2_ws}"
source "$ROS_SETUP"
source "$WS/install/setup.bash"
exec ros2 run takeoff takeoff "$@"
