#!/usr/bin/env python3
"""Batch Flight Simulation Orchestrator.

Tự động hóa hoàn toàn quy trình bay mô phỏng đa lần (Multi-Flight Batch Simulation):
1. Vòng lặp N lần bay độc lập (Monte Carlo / Scenario variations)
2. Mỗi lần bay tự động:
   - Dọn sạch tiến trình cũ (gz-sim, px4, ros2 daemon, stale locks)
   - Khởi động PX4 SITL & Gazebo Harmonic (hỗ trợ Headless server-only hoặc GUI)
   - Khởi động ROS 2 Bridges & 9 Nodes tự hành (Autonomous Pipeline)
   - Khởi động Telemetry Logger ghi nhận dữ liệu thời gian thực
   - Khởi động Auto Mission Commander điều khiển chu trình bay khép kín
   - Giám sát an toàn bằng Timeout Watchdog
   - Thu dọn tiến trình, lưu file telemetry.csv & summary.json
3. Tự động gọi evaluate_flight_metrics.py để tính 7 KPI và vẽ biểu đồ.
"""

from __future__ import annotations

import argparse
import json
import os
import signal
import subprocess
import sys
import time
from pathlib import Path

# Paths
REPO_ROOT = Path(__file__).resolve().parents[2]
PX4_DIR = REPO_ROOT / "PX4-Autopilot"
SIM_LAUNCH = REPO_ROOT / "simulation" / "launch" / "launch_simulation.py"
LOGGER_SCRIPT = REPO_ROOT / "simulation" / "utils" / "flight_telemetry_logger.py"
COMMANDER_SCRIPT = REPO_ROOT / "simulation" / "utils" / "auto_mission_commander.py"
EVAL_SCRIPT = REPO_ROOT / "simulation" / "utils" / "evaluate_flight_metrics.py"
WORLD_SDF = PX4_DIR / "Tools" / "simulation" / "gz" / "worlds" / "obstacle_avoidance.sdf"


def get_system_resources() -> dict:
    """Read available RAM and disk space."""
    import shutil
    disk_free_mb = shutil.disk_usage("/").free // (1024 * 1024)
    ram_avail_mb = 0
    try:
        with open('/proc/meminfo', 'r') as f:
            for line in f:
                if 'MemAvailable:' in line:
                    ram_avail_mb = int(line.split()[1]) // 1024
                    break
    except Exception:
        pass
    return {'ram_avail_mb': ram_avail_mb, 'disk_free_mb': disk_free_mb}


def clean_simulation_processes():
    """Kill all stale Gazebo, PX4, and ROS 2 python processes and remove runtime lock files."""
    print("🧹 [CleanUp] Dọn dẹp triệt để các tiến trình mô phỏng và giải phóng bộ nhớ...")
    # 1. Kill ROS 2 Python nodes (do not kill batch_simulation_runner itself!)
    subprocess.run("pkill -9 -f '(launch_simulation|aruco_sim_node|ekf_ros_adapter|covariance_gate|apf_planner|smc_guidance|touchdown_detector|ibvs_controller|mission_fsm_node|offboard_commander|auto_mission_commander|flight_telemetry_logger|depth_to_image_node)' 2>/dev/null || true", shell=True)
    # 2. Kill Gazebo and PX4
    subprocess.run("killall -9 gz-sim gz px4 ruby parameter_bridge ros_gz_bridge 2>/dev/null || true", shell=True)
    subprocess.run("ros2 daemon stop >/dev/null 2>&1 || true", shell=True)

    # Remove stale locks
    for p in Path("/tmp").glob("px4_lock-*"):
        try:
            p.unlink()
        except Exception:
            pass
    time.sleep(1.5)


def kill_proc_tree(proc: subprocess.Popen | None):
    """Safely terminate an entire process group."""
    if proc is not None and proc.poll() is None:
        try:
            pgid = os.getpgid(proc.pid)
            os.killpg(pgid, signal.SIGTERM)
            proc.wait(timeout=2.0)
        except Exception:
            try:
                pgid = os.getpgid(proc.pid)
                os.killpg(pgid, signal.SIGKILL)
            except Exception:
                pass


def wait_for_mavlink(timeout_s: float = 30.0) -> bool:
    """Check if PX4 MAVLink UDP (127.0.0.1:14540) is emitting heartbeats."""
    os.environ['MAVLINK_DIALECT'] = 'common'
    px4_mav = str(PX4_DIR / "src" / "modules" / "mavlink" / "mavlink")
    if px4_mav not in sys.path:
        sys.path.insert(0, px4_mav)
    try:
        from pymavlink import mavutil
        start = time.time()
        while time.time() - start < timeout_s:
            try:
                conn = mavutil.mavlink_connection('udp:127.0.0.1:14540')
                msg = conn.wait_heartbeat(timeout=1.5)
                if msg:
                    conn.close()
                    return True
            except Exception:
                pass
            time.sleep(0.5)
    except Exception as e:
        print(f"Warning in MAVLink check: {e}")
    return False


def get_ros_env() -> dict:
    """Extract full ROS 2 Humble environment variables."""
    cmd = 'source /opt/ros/humble/setup.bash 2>/dev/null && env'
    proc = subprocess.run(['bash', '-c', cmd], capture_output=True, text=True)
    env = os.environ.copy()
    for line in proc.stdout.splitlines():
        if '=' in line:
            k, v = line.split('=', 1)
            env[k] = v
    return env


def run_single_flight(run_idx: int, run_dir: Path, headless: bool = False, planner: str = "iapf", move_target: bool = False, target_speed: float = 0.5) -> bool:
    res = get_system_resources()
    print("\n" + "=" * 70)
    print(f"🚀 BẮT ĐẦU CHUYẾN BAY THỬ NGHIỆM #{run_idx:02d} -> {run_dir.name}")
    print(f"   Kịch bản: {'BÃI ĐÁP DI ĐỘNG (' + str(target_speed) + ' m/s)' if move_target else 'BÃI ĐÁP CỐ ĐỊNH'} | Planner: {planner.upper()} | Headless: {headless}")
    print(f"   💾 Tài nguyên hệ thống: RAM trống {res['ram_avail_mb']} MB | Disk trống {res['disk_free_mb']} MB")
    print("=" * 70)

    run_dir.mkdir(parents=True, exist_ok=True)
    csv_file = run_dir / "telemetry.csv"
    summary_file = run_dir / "summary.json"

    # Step 1: Clean
    clean_simulation_processes()

    # Step 2: Environment
    env = get_ros_env()
    env['GZ_PARTITION'] = 'vdt_harmonic'
    env['ROS_DOMAIN_ID'] = '0'
    env['GZ_SIM_RESOURCE_PATH'] = f"{PX4_DIR}/Tools/simulation/gz/models:{env.get('GZ_SIM_RESOURCE_PATH', '')}"
    env['PYTHONPATH'] = f"{REPO_ROOT}:{env.get('PYTHONPATH', '')}"
    if headless:
        env['HEADLESS'] = '1'

    # Step 3: Start PX4 SITL + Gazebo
    print("▶ [1/5] Khởi động PX4 SITL & Gazebo Harmonic...")
    px4_cmd = ["make", "px4_sitl", "gz_x500_depth"]
    env['PX4_GZ_WORLD'] = 'obstacle_avoidance'
    px4_proc = subprocess.Popen(
        px4_cmd, cwd=str(PX4_DIR), env=env,
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
        preexec_fn=os.setsid
    )

    print("   Đang chờ PX4 khởi tạo và phát Heartbeat MAVLink...")
    if not wait_for_mavlink(timeout_s=35.0):
        print("❌ [LỖI] Quá thời gian chờ PX4 MAVLink! Hủy chuyến bay này.")
        kill_proc_tree(px4_proc)
        clean_simulation_processes()
        return False
    print("   ✅ PX4 SITL đã sẵn sàng trên cổng UDP 14540.")

    # Step 4: Start ROS 2 autonomous pipeline
    print("▶ [2/5] Khởi động ROS 2 Bridges & Autonomous Pipeline...")
    sim_log_file = open(run_dir / "sim_pipeline.log", "w")
    sim_cmd = [sys.executable, str(SIM_LAUNCH), "--autonomous", "--no-rviz", f"--planner={planner}"]
    sim_proc = subprocess.Popen(
        sim_cmd, env=env,
        stdout=sim_log_file, stderr=subprocess.STDOUT,
        preexec_fn=os.setsid
    )
    time.sleep(4.0)

    # Step 5: Start Telemetry Logger
    print("▶ [3/5] Khởi động Telemetry Logger...")
    log_cmd = [
        sys.executable, str(LOGGER_SCRIPT),
        "--csv", str(csv_file),
        "--json", str(summary_file),
        "--world-sdf", str(WORLD_SDF)
    ]
    log_proc = subprocess.Popen(log_cmd, env=env, preexec_fn=os.setsid)
    time.sleep(1.0)

    # Step 6: Start Mission Commander
    print("▶ [4/5] Kích hoạt Auto Mission Commander...")
    cmd_args = [
        sys.executable, str(COMMANDER_SCRIPT),
        "--takeoff-alt", "3.0",
        "--follow-duration", "8.0",
        "--max-time", "160.0"
    ]
    if move_target:
        cmd_args.extend(["--move-target", "--target-speed", str(target_speed)])

    commander_proc = subprocess.Popen(cmd_args, env=env, preexec_fn=os.setsid)

    # Wait for flight completion
    print("   Đang thực thi chu trình bay tự động (Takeoff -> Follow -> Glide Slope -> Land)...")
    commander_proc.wait()
    print("   ✅ Chu trình bay đã hoàn tất.")

    # Step 7: Teardown and Save
    print("▶ [5/5] Thu thập dữ liệu và dọn dẹp tài nguyên triệt để...")
    time.sleep(1.5)
    kill_proc_tree(commander_proc)
    kill_proc_tree(log_proc)
    kill_proc_tree(sim_proc)
    kill_proc_tree(px4_proc)
    try:
        sim_log_file.close()
    except Exception:
        pass

    clean_simulation_processes()

    if summary_file.exists():
        with open(summary_file, 'r') as f:
            meta = json.load(f)
        succ = meta.get('mission_success', False)
        rxy = meta.get('final_landing_error_rxy_cm', 'N/A')
        vz = meta.get('touchdown_vz_m_s', 'N/A')
        print(f"🎉 KẾT QUẢ RUN #{run_idx:02d}: Success={succ} | Sai số tiếp đất Rxy={rxy}cm | Vận tốc vz={vz}m/s")
        return succ
    else:
        print(f"⚠️ Run #{run_idx:02d} không sinh file summary.json.")
        return False


def main():
    parser = argparse.ArgumentParser(description="Batch Flight Simulation Orchestrator")
    parser.add_argument('--runs', type=int, default=10, help="Số lần bay cần thực hiện (mặc định: 10)")
    parser.add_argument('--headless', action='store_true', default=False, help="Chạy Gazebo server-only không GUI")
    parser.add_argument('--gui', dest='headless', action='store_false', help="Mở GUI Gazebo khi bay")
    parser.add_argument('--output-dir', default="simulation_results/full_system_10runs_evaluation", help="Thư mục xuất dữ liệu")
    parser.add_argument('--planner', default="iapf", choices=["iapf", "apf"], help="Thuật toán quy hoạch APF/IAPF")
    parser.add_argument('--scenario', default="mixed", choices=["static", "moving", "mixed"], help="Kịch bản di chuyển bãi đáp")
    args = parser.parse_args()

    batch_dir = Path(args.output_dir)
    batch_dir.mkdir(parents=True, exist_ok=True)

    print("=" * 75)
    print("  HỆ THỐNG THỰC THI MÔ PHỎNG TỰ ĐỘNG ĐA LẦN BAY - QUADROTOR VDT")
    print(f"  Tổng số lần bay dự kiến: {args.runs} | Headless: {args.headless} | Output: {batch_dir}")
    print("=" * 75)

    scenario_matrix = [
        {"name": "Bãi tĩnh Baseline (3.0, 1.0)", "move": False, "speed": 0.0},
        {"name": "Bãi di động thẳng nhẹ (0.3 m/s)", "move": True, "speed": 0.3},
        {"name": "Bãi tĩnh lặp lại #1", "move": False, "speed": 0.0},
        {"name": "Bãi di động vừa (0.4 m/s)", "move": True, "speed": 0.4},
        {"name": "Bãi tĩnh lặp lại #2", "move": False, "speed": 0.0},
        {"name": "Bãi di động tiêu chuẩn (0.5 m/s, quỹ đạo uốn)", "move": True, "speed": 0.5},
        {"name": "Bãi tĩnh lặp lại #3", "move": False, "speed": 0.0},
        {"name": "Bãi di động tốc độ cao (0.6 m/s)", "move": True, "speed": 0.6},
        {"name": "Bãi tĩnh lặp lại #4", "move": False, "speed": 0.0},
        {"name": "Bãi di động hỗn hợp (0.5 m/s)", "move": True, "speed": 0.5},
    ]

    successful = 0
    for i in range(1, args.runs + 1):
        run_dir = batch_dir / f"run_{i:02d}"
        if i <= len(scenario_matrix):
            sc = scenario_matrix[i - 1]
            move = sc['move']
            speed = sc['speed']
        else:
            move = (i % 2 == 0)
            speed = 0.5

        if args.scenario == "moving":
            move = True
            speed = 0.4
        elif args.scenario == "static":
            move = False
            speed = 0.0

        ok = run_single_flight(
            run_idx=i,
            run_dir=run_dir,
            headless=args.headless,
            planner=args.planner,
            move_target=move,
            target_speed=speed
        )
        if ok:
            successful += 1
        time.sleep(2.0)

    print("\n" + "=" * 75)
    print(f"🏁 HOÀN TẤT ĐỢT BAY THỬ NGHIỆM: {successful}/{args.runs} THÀNH CÔNG ({(successful/args.runs)*100.0:.1f}%)")
    print("=" * 75)

    # Run evaluation
    print("\n📊 Tiến hành phân tích thống kê 7 chỉ số KPI và sinh đồ thị nghiệm thu...")
    eval_cmd = [sys.executable, str(EVAL_SCRIPT), "--batch-dir", str(batch_dir)]
    subprocess.run(eval_cmd)


if __name__ == '__main__':
    main()
