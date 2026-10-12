#!/usr/bin/env python3
"""Benchmark Module 1: IBVS Tracking & Visual Servoing (10 Independent Trials).

Đánh giá định lượng hiệu năng của hệ thống IBVS Controller:
- 10 kịch bản thử nghiệm: Thay đổi góc lệch ban đầu (Yaw offsets), khoảng cách,
  vận tốc bãi đáp di chuyển, và nhiễu quang học camera.
- Đo lường: Sai số góc Yaw (Mean & Max), sai số pixel mặt phẳng ảnh,
  thời gian ổn định (settling time), tốc độ đáp ứng gimbal pitch.
- Xuất file JSON thống kê và biểu đồ trực quan hóa.
"""

from __future__ import annotations

import json
import math
import os
import sys
from pathlib import Path
from typing import Dict, List, Any

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from simulation.control.tracking_control import (
    update_tracking_yaw_command,
    landing_pitch_from_geometry,
    update_search_pitch_command,
    _wrap_angle
)


def run_ibvs_trial(
    trial_id: int,
    name: str,
    initial_yaw_offset_deg: float,
    initial_pitch_deg: float,
    target_distance: float,
    target_vel_xy: tuple[float, float],
    noise_sigma_px: float = 0.0,
    duration_s: float = 10.0,
    dt: float = 0.033,  # 30 Hz camera rate
) -> Dict[str, Any]:
    """Simulate a 10s closed-loop IBVS tracking episode."""
    focal_x = 466.0
    focal_y = 466.0
    u0 = 320.0
    v0 = 240.0
    pixel_gain = 0.5
    pitch_gain = 0.8
    yaw_rate_limit = 0.80  # rad/s

    drone_yaw = 0.0
    yaw_cmd = math.radians(initial_yaw_offset_deg)
    current_pitch = math.radians(initial_pitch_deg)

    # Target motion setup
    target_x = target_distance * math.cos(math.radians(initial_yaw_offset_deg))
    target_y = target_distance * math.sin(math.radians(initial_yaw_offset_deg))
    target_z = -target_distance * math.sin(math.radians(abs(initial_pitch_deg)))

    times = []
    yaw_errors_deg = []
    pixel_errors_u = []
    pixel_errors_v = []
    pitch_cmds_deg = []

    steps = int(duration_s / dt)
    settled_time = None

    for i in range(steps):
        t = i * dt
        times.append(t)

        # Update target ground-truth position
        target_x += target_vel_xy[0] * dt
        target_y += target_vel_xy[1] * dt

        # Relative geometry in drone body frame
        dx_w = target_x
        dy_w = target_y
        dz_w = target_z

        # Rotate to body frame
        cos_y = math.cos(drone_yaw)
        sin_y = math.sin(drone_yaw)
        dx_b = cos_y * dx_w + sin_y * dy_w
        dy_b = -sin_y * dx_w + cos_y * dy_w
        dz_b = dz_w

        # Angle to target in horizontal plane (LOS yaw error)
        los_yaw = math.atan2(dy_w, dx_w)
        yaw_err = math.degrees(abs(_wrap_angle(drone_yaw - los_yaw)))
        yaw_errors_deg.append(yaw_err)

        if settled_time is None and yaw_err <= 2.0 and t > 0.5:
            settled_time = t

        # Project into pinhole camera model with gimbal pitch
        cos_p = math.cos(current_pitch)
        sin_p = math.sin(current_pitch)
        # Point in camera frame after pitch down
        x_c = cos_p * dx_b - sin_p * dz_b
        y_c = dy_b
        z_c = sin_p * dx_b + cos_p * dz_b

        if x_c > 0.1:
            u_proj = u0 - (focal_x * y_c / x_c) + np.random.normal(0, noise_sigma_px)
            v_proj = v0 + (focal_y * z_c / x_c) + np.random.normal(0, noise_sigma_px)
        else:
            u_proj = u0
            v_proj = v0

        pixel_errors_u.append(abs(u_proj - u0))
        pixel_errors_v.append(abs(v_proj - v0))

        # IBVS Yaw update
        yaw_cmd = update_tracking_yaw_command(
            yaw_command=yaw_cmd,
            drone_yaw=drone_yaw,
            pixel_u=u_proj,
            image_center_u=u0,
            focal_x=focal_x,
            pixel_gain=pixel_gain,
            target_delta_xy=(dx_w, dy_w),
            target_velocity_xy=target_vel_xy,
            tracking_mode="TRACKING",
            target_state_age=0.01,
            target_state_timeout=0.5,
            dt=dt,
            yaw_rate_limit=yaw_rate_limit
        )

        # Drone yaw dynamics: 1st order lag towards command
        max_yaw_step = yaw_rate_limit * dt
        drone_yaw += np.clip(_wrap_angle(yaw_cmd - drone_yaw), -max_yaw_step, max_yaw_step)

        # Gimbal Pitch update
        dist_xy = math.hypot(dx_w, dy_w)
        rel_alt = abs(dz_w)
        target_pitch = landing_pitch_from_geometry(rel_alt, 0.0, dist_xy)
        current_pitch += 0.25 * (target_pitch - current_pitch)
        pitch_cmds_deg.append(math.degrees(current_pitch))

    settle_val = settled_time if settled_time is not None else duration_s
    # In moving scenarios, evaluate steady-state yaw error after settling
    ss_errors = yaw_errors_deg[int(len(yaw_errors_deg)*0.3):]

    return {
        'trial_id': trial_id,
        'name': name,
        'initial_offset_deg': initial_yaw_offset_deg,
        'target_speed_mps': round(math.hypot(*target_vel_xy), 2),
        'mean_yaw_error_deg': round(float(np.mean(ss_errors)), 2),
        'max_yaw_error_deg': round(float(np.max(yaw_errors_deg)), 2),
        'mean_pixel_error_px': round(float(np.mean(pixel_errors_u)), 1),
        'settling_time_s': round(settle_val, 2),
        'times': times,
        'yaw_errors': yaw_errors_deg,
        'pitch_cmds': pitch_cmds_deg,
    }


def main():
    print("=" * 75)
    print("  BENCHMARK MODULE 1: ĐÁNH GIÁ TRACKING IBVS (10 THỬ NGHIỆM ĐỘC LẬP)")
    print("=" * 75)

    scenarios = [
        {"name": "Trial 01: Bãi tĩnh, lệch nhẹ (+15°)", "offset": 15.0, "pitch": -45.0, "dist": 3.0, "vel": (0.0, 0.0), "noise": 0.0},
        {"name": "Trial 02: Bãi tĩnh, lệch nhẹ (-15°)", "offset": -15.0, "pitch": -45.0, "dist": 3.0, "vel": (0.0, 0.0), "noise": 0.0},
        {"name": "Trial 03: Bãi tĩnh, lệch lớn (+30°)", "offset": 30.0, "pitch": -35.0, "dist": 4.0, "vel": (0.0, 0.0), "noise": 0.0},
        {"name": "Trial 04: Bãi tĩnh, lệch lớn (-30°)", "offset": -30.0, "pitch": -35.0, "dist": 4.0, "vel": (0.0, 0.0), "noise": 0.0},
        {"name": "Trial 05: Bãi di động tịnh tiến (0.3m/s, 0°)", "offset": 0.0, "pitch": -45.0, "dist": 3.0, "vel": (0.3, 0.0), "noise": 0.5},
        {"name": "Trial 06: Bãi di động (+20°, 0.5m/s)", "offset": 20.0, "pitch": -40.0, "dist": 3.5, "vel": (0.5, 0.1), "noise": 0.5},
        {"name": "Trial 07: Bãi di động cắt ngang (-20°, 0.4m/s)", "offset": -20.0, "pitch": -40.0, "dist": 3.5, "vel": (0.1, 0.4), "noise": 0.5},
        {"name": "Trial 08: Bãi di động sóng sin (0.5m/s, lệch 25°)", "offset": 25.0, "pitch": -45.0, "dist": 3.0, "vel": (0.5, -0.2), "noise": 1.0},
        {"name": "Trial 09: Bãi di động tốc độ cao (0.8m/s, +35°)", "offset": 35.0, "pitch": -35.0, "dist": 4.0, "vel": (0.8, 0.0), "noise": 1.0},
        {"name": "Trial 10: Nhiễu thị giác cao (σ=3px, 0.5m/s)", "offset": -25.0, "pitch": -45.0, "dist": 3.0, "vel": (0.5, 0.2), "noise": 3.0},
    ]

    results = []
    for idx, sc in enumerate(scenarios, 1):
        res = run_ibvs_trial(
            trial_id=idx,
            name=sc['name'],
            initial_yaw_offset_deg=sc['offset'],
            initial_pitch_deg=sc['pitch'],
            target_distance=sc['dist'],
            target_vel_xy=sc['vel'],
            noise_sigma_px=sc['noise'],
        )
        results.append(res)
        print(f"[{idx:02d}/10] {res['name']}: Settle={res['settling_time_s']}s | Yaw Err={res['mean_yaw_error_deg']}° | Pixel Err={res['mean_pixel_error_px']}px")

    # Aggregate stats
    yaw_means = [r['mean_yaw_error_deg'] for r in results]
    settle_times = [r['settling_time_s'] for r in results]
    pixel_means = [r['mean_pixel_error_px'] for r in results]

    summary = {
        'total_trials': len(results),
        'metrics': {
            'mean_yaw_error_deg': {
                'mean': round(float(np.mean(yaw_means)), 2),
                'std': round(float(np.std(yaw_means)), 2),
                'min': round(float(np.min(yaw_means)), 2),
                'max': round(float(np.max(yaw_means)), 2),
                'p95': round(float(np.percentile(yaw_means, 95)), 2)
            },
            'settling_time_s': {
                'mean': round(float(np.mean(settle_times)), 2),
                'std': round(float(np.std(settle_times)), 2),
                'min': round(float(np.min(settle_times)), 2),
                'max': round(float(np.max(settle_times)), 2),
                'p95': round(float(np.percentile(settle_times, 95)), 2)
            },
            'pixel_error_px': {
                'mean': round(float(np.mean(pixel_means)), 2),
                'std': round(float(np.std(pixel_means)), 2),
                'max': round(float(np.max(pixel_means)), 2)
            }
        },
        'trials': [
            {k: v for k, v in r.items() if k not in ('times', 'yaw_errors', 'pitch_cmds')}
            for r in results
        ]
    }

    out_dir = REPO_ROOT / "simulation_results" / "module_benchmarks"
    out_dir.mkdir(parents=True, exist_ok=True)

    json_path = out_dir / "ibvs_benchmark_report.json"
    with open(json_path, 'w') as f:
        json.dump(summary, f, indent=2)
    print(f"\n✅ Đã lưu kết quả Module 1 vào: {json_path}")

    # Plot
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8), sharex=True)
    for r in results:
        ax1.plot(r['times'], r['yaw_errors'], label=f"T{r['trial_id']:02d}: {r['name'].split(':')[1]}", alpha=0.8, linewidth=1.5)
        ax2.plot(r['times'], r['pitch_cmds'], alpha=0.8, linewidth=1.5)

    ax1.axhline(5.0, color='r', linestyle='--', linewidth=1.5, label='Tiêu chuẩn KPI ($e_{\\psi} \\leq 5^\\circ$)')
    ax1.axhline(2.0, color='g', linestyle=':', linewidth=1.5, label='Vùng khóa ổn định ($e_{\\psi} \\leq 2^\\circ$)')
    ax1.set_ylabel('Sai số góc Yaw ($^\\circ$)', fontsize=11)
    ax1.set_title('Đánh giá Module 1 (IBVS): Diễn biến Góc Yaw qua 10 Kịch bản Thử nghiệm', fontsize=13, fontweight='bold')
    ax1.legend(loc='upper right', fontsize=8, ncol=2)
    ax1.grid(True, linestyle=':', alpha=0.6)

    ax2.set_xlabel('Thời gian (s)', fontsize=11)
    ax2.set_ylabel('Góc Gimbal Pitch ($^\\circ$)', fontsize=11)
    ax2.set_title('Đáp ứng Góc Gimbal Pitch (Duy trì mốc trong FOV)', fontsize=12, fontweight='bold')
    ax2.grid(True, linestyle=':', alpha=0.6)

    fig.tight_layout()
    plot_path = out_dir / "ibvs_tracking_evaluation.png"
    fig.savefig(plot_path, dpi=200)
    plt.close(fig)
    print(f"✅ Đã sinh biểu đồ Module 1: {plot_path}")


if __name__ == '__main__':
    main()
