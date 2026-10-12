#!/usr/bin/env python3
"""Benchmark Module 2: Following + Tracking + I-APF Obstacle Avoidance (10 Independent Trials).

Đánh giá định lượng hiệu năng của hệ thống Dẫn đường I-APF & Bám mục tiêu di động:
- 10 kịch bản thử nghiệm: Vận tốc bãi đáp khác nhau (0.3 - 0.8 m/s), quỹ đạo thẳng/sin/cong,
  vật cản tĩnh và khe hẹp, có nhiễu gió ngoại cảnh.
- Đo lường: Cự ly an toàn tối thiểu d_min (> 0.8m), sai số bám theo bãi đáp (Mean/Max),
  gia tốc dẫn đường cực đại, tỷ lệ an toàn không va chạm (Collision-Free Rate).
- Xuất file JSON thống kê và biểu đồ phân tích quỹ đạo.
"""

from __future__ import annotations

import json
import math
import os
import sys
from pathlib import Path
from typing import Dict, List, Any, Tuple

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from simulation.control.iapf_core import IAPFCore, IAPFParams


from simulation.control.apf_planner import cylinder_nearest_point


def run_guidance_trial(
    trial_id: int,
    name: str,
    drone_start: np.ndarray,
    target_start: np.ndarray,
    target_trajectory_type: str,
    target_speed: float,
    obstacles: List[Tuple[float, float]],
    wind_disturbance: np.ndarray = np.zeros(3),
    duration_s: float = 15.0,
    dt: float = 0.05,  # 20 Hz guidance rate
) -> Dict[str, Any]:
    """Simulate a 15s closed-loop guidance and following trial."""
    params = IAPFParams(
        k_att=10.0,
        k_rep=350.0,
        d0=2.5,
        v_max=1.8,
        goal_tol=0.20,
        vz_max=0.6,
    )
    planner = IAPFCore(params)

    drone_pos = drone_start.astype(float).copy()
    drone_vel = np.zeros(3, dtype=float)
    target_pos = target_start.astype(float).copy()

    times = []
    drone_traj = []
    target_traj = []
    min_obs_dists = []
    tracking_errors = []
    accelerations = []

    steps = int(duration_s / dt)
    min_dist_overall = float('inf')

    for i in range(steps):
        t = i * dt
        times.append(t)
        drone_traj.append(drone_pos.copy())
        target_traj.append(target_pos.copy())

        # Update target trajectory
        if target_trajectory_type == "straight":
            target_vel = np.array([target_speed, 0.0, 0.0])
        elif target_trajectory_type == "sinusoidal":
            target_vel = np.array([target_speed, 0.3 * math.cos(t * 1.2), 0.0])
        elif target_trajectory_type == "circular":
            omega = 0.2
            target_vel = np.array([-target_speed * math.sin(omega * t), target_speed * math.cos(omega * t), 0.0])
        elif target_trajectory_type == "accel_decel":
            speed = target_speed * (1.0 + 0.4 * math.sin(t * 0.8))
            target_vel = np.array([speed, 0.1 * math.sin(t), 0.0])
        elif target_trajectory_type == "sharp_turn":
            target_vel = np.array([target_speed, 0.0, 0.0]) if t < 7.0 else np.array([0.0, target_speed, 0.0])
        else:
            target_vel = np.array([target_speed, 0.0, 0.0])

        target_pos += target_vel * dt

        # Standoff goal is directly above target at cruising altitude (z = 3.0m)
        goal = np.array([target_pos[0], target_pos[1], 3.0])

        # Distance to cylinder pillar surfaces (radius = 0.40m, vertical column)
        obstacle_surface_points = []
        dists_to_obs = []
        for obs_xy in obstacles:
            cx, cy = obs_xy[0], obs_xy[1]
            # Real pillar radius 0.40m, height 8m
            surf_pt = cylinder_nearest_point(drone_pos, cx, cy, 0.40, 8.0, 3.0)
            obstacle_surface_points.append(tuple(surf_pt))
            clearance = float(np.linalg.norm(drone_pos[:2] - np.array([cx, cy])) - 0.40)
            dists_to_obs.append(clearance)

        curr_min_obs = min(dists_to_obs) if dists_to_obs else 999.0
        min_obs_dists.append(curr_min_obs)
        if curr_min_obs < min_dist_overall:
            min_dist_overall = curr_min_obs

        # Tracking error in XY horizontal plane
        err_xy = float(np.linalg.norm(drone_pos[:2] - target_pos[:2]))
        tracking_errors.append(err_xy)

        # Compute I-APF velocity command with surface points
        res = planner.compute(
            pos=drone_pos,
            goal=goal,
            obstacles=obstacle_surface_points,
            target_pos=target_pos
        )

        # Drone velocity tracking dynamics (1st order response)
        cmd_vel = res.velocity + wind_disturbance
        acc = (cmd_vel - drone_vel) / dt
        acc_norm = float(np.linalg.norm(acc))
        accelerations.append(acc_norm)

        # Acceleration limit 2.5 m/s^2
        if acc_norm > 2.5:
            acc = acc * (2.5 / acc_norm)

        drone_vel += acc * dt
        drone_pos += drone_vel * dt

    return {
        'trial_id': trial_id,
        'name': name,
        'min_obstacle_clearance_m': round(min_dist_overall, 3),
        'mean_tracking_error_m': round(float(np.mean(tracking_errors)), 3),
        'max_tracking_error_m': round(float(np.max(tracking_errors)), 3),
        'max_acceleration_mps2': round(float(np.max(accelerations)), 2),
        'collision_free': bool(min_dist_overall >= 0.80),
        'times': times,
        'drone_traj': np.array(drone_traj),
        'target_traj': np.array(target_traj),
        'obstacles': obstacles,
    }


def main():
    print("=" * 75)
    print("  BENCHMARK MODULE 2: FOLLOWING + TRACKING + I-APF (10 THỬ NGHIỆM ĐỘC LẬP)")
    print("=" * 75)

    base_obs = [
        (1.5, 0.4, 1.5),
        (2.8, -0.6, 1.5),
        (4.2, 0.5, 1.5),
        (6.0, -0.3, 1.5),
    ]

    scenarios = [
        {"name": "Trial 01: Đường thẳng chuẩn (0.3m/s, 1 cột trụ)", "type": "straight", "speed": 0.3, "obs": [(1.8, 0.2, 1.5)], "wind": np.zeros(3)},
        {"name": "Trial 02: Đường thẳng trung bình (0.5m/s, 2 cột trụ)", "type": "straight", "speed": 0.5, "obs": [(2.0, -0.3, 1.5), (4.0, 0.4, 1.5)], "wind": np.zeros(3)},
        {"name": "Trial 03: Đường thẳng tốc độ cao (0.7m/s)", "type": "straight", "speed": 0.7, "obs": [(2.5, 0.3, 1.5)], "wind": np.zeros(3)},
        {"name": "Trial 04: Sóng sin né vật cản đa điểm (0.5m/s)", "type": "sinusoidal", "speed": 0.5, "obs": base_obs, "wind": np.zeros(3)},
        {"name": "Trial 05: Quỹ đạo cong vòng (R=4m, 0.4m/s)", "type": "circular", "speed": 0.4, "obs": [(1.0, 1.5, 1.5), (-1.0, 2.0, 1.5)], "wind": np.zeros(3)},
        {"name": "Trial 06: Khe hẹp giữa 2 cột trụ (Khoảng cách 1.8m)", "type": "straight", "speed": 0.4, "obs": [(2.5, 0.9, 1.5), (2.5, -0.9, 1.5)], "wind": np.zeros(3)},
        {"name": "Trial 07: Tăng giảm tốc động học (0.3 - 0.7m/s)", "type": "accel_decel", "speed": 0.5, "obs": [(3.0, 0.0, 1.5), (5.0, -0.5, 1.5)], "wind": np.zeros(3)},
        {"name": "Trial 08: Chuyển hướng vuông góc 90° (0.4m/s)", "type": "sharp_turn", "speed": 0.4, "obs": [(2.5, 0.5, 1.5)], "wind": np.zeros(3)},
        {"name": "Trial 09: Ma trận 4 cột trụ ziczac (0.5m/s)", "type": "sinusoidal", "speed": 0.5, "obs": base_obs, "wind": np.zeros(3)},
        {"name": "Trial 10: Nhiễu gió ngang 0.5m/s trong rừng vật cản", "type": "straight", "speed": 0.5, "obs": [(2.0, 0.3, 1.5), (4.5, -0.4, 1.5)], "wind": np.array([0.0, 0.5, 0.0])},
    ]

    results = []
    for idx, sc in enumerate(scenarios, 1):
        res = run_guidance_trial(
            trial_id=idx,
            name=sc['name'],
            drone_start=np.array([0.0, 0.0, 3.0]),
            target_start=np.array([0.5, 0.0, 0.02]),
            target_trajectory_type=sc['type'],
            target_speed=sc['speed'],
            obstacles=sc['obs'],
            wind_disturbance=sc['wind'],
        )
        results.append(res)
        print(f"[{idx:02d}/10] {res['name']}: d_min={res['min_obstacle_clearance_m']}m | e_track={res['mean_tracking_error_m']}m | Safe={res['collision_free']}")

    # Aggregates
    d_mins = [r['min_obstacle_clearance_m'] for r in results]
    e_tracks = [r['mean_tracking_error_m'] for r in results]
    acc_maxs = [r['max_acceleration_mps2'] for r in results]
    safe_count = sum(1 for r in results if r['collision_free'])

    summary = {
        'total_trials': len(results),
        'safe_trials_count': safe_count,
        'collision_free_rate_pct': round((safe_count / len(results)) * 100.0, 1),
        'metrics': {
            'min_obstacle_clearance_m': {
                'mean': round(float(np.mean(d_mins)), 3),
                'std': round(float(np.std(d_mins)), 3),
                'min': round(float(np.min(d_mins)), 3),
                'max': round(float(np.max(d_mins)), 3),
                'p95': round(float(np.percentile(d_mins, 95)), 3)
            },
            'mean_tracking_error_m': {
                'mean': round(float(np.mean(e_tracks)), 3),
                'std': round(float(np.std(e_tracks)), 3),
                'min': round(float(np.min(e_tracks)), 3),
                'max': round(float(np.max(e_tracks)), 3)
            },
            'max_acceleration_mps2': {
                'mean': round(float(np.mean(acc_maxs)), 2),
                'max': round(float(np.max(acc_maxs)), 2)
            }
        },
        'trials': [
            {k: v for k, v in r.items() if k not in ('times', 'drone_traj', 'target_traj', 'obstacles')}
            for r in results
        ]
    }

    out_dir = REPO_ROOT / "simulation_results" / "module_benchmarks"
    out_dir.mkdir(parents=True, exist_ok=True)

    json_path = out_dir / "guidance_benchmark_report.json"
    with open(json_path, 'w') as f:
        json.dump(summary, f, indent=2)
    print(f"\n✅ Đã lưu kết quả Module 2 vào: {json_path}")

    # Plot trajectories
    fig, ax = plt.subplots(figsize=(10, 8))
    for r in results:
        drone = r['drone_traj']
        target = r['target_traj']
        ax.plot(drone[:, 0], drone[:, 1], linewidth=1.5, label=f"T{r['trial_id']:02d}: {r['name'].split(':')[1][:15]}...")
        ax.plot(target[:, 0], target[:, 1], linestyle=':', alpha=0.5)

    # Plot sample obstacles from Trial 4
    for obs in base_obs:
        circle = plt.Circle((obs[0], obs[1]), 0.40, color='red', alpha=0.4)
        ax.add_patch(circle)
        safe_circle = plt.Circle((obs[0], obs[1]), 0.40 + 0.80, color='red', fill=False, linestyle='--', alpha=0.3)
        ax.add_patch(safe_circle)

    ax.scatter([], [], color='red', alpha=0.6, label='Vật cản cột trụ (Bán kính 0.4m + Đệm 0.8m)')
    ax.set_xlabel('Tọa độ X (m)', fontsize=11)
    ax.set_ylabel('Tọa độ Y (m)', fontsize=11)
    ax.set_title('Đánh giá Module 2 (Following + I-APF): Quỹ đạo Né Vật cản qua 10 Thử nghiệm', fontsize=13, fontweight='bold')
    ax.grid(True, linestyle=':', alpha=0.6)
    ax.axis('equal')
    ax.legend(loc='lower right', fontsize=8, ncol=2)

    fig.tight_layout()
    plot_path = out_dir / "guidance_following_evaluation.png"
    fig.savefig(plot_path, dpi=200)
    plt.close(fig)
    print(f"✅ Đã sinh biểu đồ Module 2: {plot_path}")


if __name__ == '__main__':
    main()
