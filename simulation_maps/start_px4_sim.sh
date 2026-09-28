#!/usr/bin/env bash
# ==============================================================================
# Script khởi động an toàn PX4 SITL & Gazebo Harmonic cho dự án VDT
# - Tự động dọn sạch các tiến trình cũ còn treo (tránh lỗi kẹt cổng, không lên GUI)
# - Nạp đầy đủ biến môi trường Gazebo & ROS 2
# ==============================================================================

set -e

echo "[1/3] Dọn dẹp các tiến trình mô phỏng cũ còn chạy ngầm..."
killall -9 gz-sim gz px4 2>/dev/null || true
ros2 daemon stop >/dev/null 2>&1 || true
# PX4 can leave an instance lock behind if the previous SITL/Gazebo process
# was terminated abruptly.  Remove only PX4 runtime locks after confirming
# that no PX4 process remains; otherwise a real running instance is never
# disturbed.
if ! pgrep -x px4 >/dev/null 2>&1; then
  for px4_lock in /tmp/px4_lock-*; do
    [ -e "$px4_lock" ] || continue
    rm -f -- "$px4_lock"
    echo "Removed stale PX4 lock: $px4_lock"
  done
fi
sleep 0.5

echo "[2/3] Thiết lập biến môi trường mô phỏng..."
export GZ_PARTITION=vdt_harmonic
export ROS_DOMAIN_ID=0
export GZ_SIM_RESOURCE_PATH="/home/duy/VDT_project/PX4-Autopilot/Tools/simulation/gz/models:${GZ_SIM_RESOURCE_PATH:-}"

echo "[3/3] Khởi động PX4 SITL & Gazebo Harmonic (world: obstacle_avoidance)..."
cd /home/duy/VDT_project/PX4-Autopilot
PX4_GZ_WORLD=obstacle_avoidance make px4_sitl gz_x500_depth
