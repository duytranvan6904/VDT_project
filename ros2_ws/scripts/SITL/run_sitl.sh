#!/usr/bin/env bash
set -u
PX4_DIR=${PX4_DIR:-$HOME/PX4-Autopilot}
WS=${WS:-$HOME/ros2_ws}
HERE=$(cd "$(dirname "$0")" && pwd)
LOG=$HERE/logs
mkdir -p "$LOG"
MODULES=("$@")
[ ${#MODULES[@]} -eq 0 ] && MODULES=($(cd "$HERE" && ls test_sitl_*.py))

source /opt/ros/${ROS_DISTRO:-humble}/setup.bash
source "$WS/install/setup.bash"

PIDS=()
stop_all() {
  for p in "${PIDS[@]:-}"; do [ -n "$p" ] && kill -INT "$p" 2>/dev/null; done
  sleep 3
  pkill -f MicroXRCEAgent 2>/dev/null
  pkill -f "bin/px4" 2>/dev/null
  PIDS=()
  sleep 2
}
trap stop_all EXIT

start_all() {
  (cd "$PX4_DIR" && sleep infinity | make px4_sitl sihsim_quadx) >"$LOG/px4.log" 2>&1 &
  PIDS+=($!)
  for _ in $(seq 120); do grep -q "Ready for takeoff" "$LOG/px4.log" && break; sleep 1; done
  grep -q "Ready for takeoff" "$LOG/px4.log" || { echo "PX4 SITL khong san sang"; return 1; }

  ros2 launch vdt_bringup vdt_system.launch.py start_hardware:=false >"$LOG/bringup.log" 2>&1 &
  PIDS+=($!)
  ros2 launch ekf_adapter ekf.launch.py publish_odom_tf:=false publish_camera_tf:=false >"$LOG/ekf.log" 2>&1 &
  PIDS+=($!)
  ros2 run ibvs ibvs_controller --ros-args -r /ekf/target_state:=/hpad/state_filtered >"$LOG/ibvs.log" 2>&1 &
  PIDS+=($!)
  ros2 launch apf_planner apf_planner.launch.py planner_type:=iapf >"$LOG/apf.log" 2>&1 &
  PIDS+=($!)
  sleep 10
}

RC=0
for m in "${MODULES[@]}"; do
  echo "=== $m ==="
  start_all || { RC=1; stop_all; continue; }
  python3 -m pytest -v -x --junitxml="$LOG/${m%.py}.xml" "$HERE/$m" || RC=1
  stop_all
done
exit $RC