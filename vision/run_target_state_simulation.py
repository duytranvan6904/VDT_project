#!/usr/bin/env python3
"""Run synthetic EKF verification and save metrics/plots without hardware."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from .target_state_ekf import TargetStateEKF
from .target_state_ct_ekf import CoordinatedTurnEKF
from .target_state_imm import TargetStateIMM
from .target_state_simulation import (
    SimulationConfig,
    calculate_metrics,
    iter_default_scenarios,
    replay_estimator,
    simulate_measurements,
)


def plot_result(output_path: Path, data, replay, title: str) -> None:
    truth = data.truth_state
    valid_measurements = data.detected
    valid_state = replay.initialized
    error = np.linalg.norm(replay.states[:, :3] - truth[:, :3], axis=1)

    fig, axes = plt.subplots(2, 2, figsize=(13, 8))
    axes[0, 0].plot(truth[:, 0], truth[:, 1], "k-", label="ground truth")
    axes[0, 0].scatter(data.measurements[valid_measurements, 0], data.measurements[valid_measurements, 1], s=7, alpha=0.35, label="raw measurement")
    axes[0, 0].plot(replay.states[valid_state, 0], replay.states[valid_state, 1], "b-", label="filtered")
    axes[0, 0].set_title("XY trajectory")
    axes[0, 0].set_xlabel("x (m)")
    axes[0, 0].set_ylabel("y (m)")
    axes[0, 0].axis("equal")
    axes[0, 0].legend()

    for axis, name in enumerate(("x", "y", "z")):
        axes[0, 1].plot(data.timestamps, truth[:, axis], label=f"truth {name}")
        axes[0, 1].plot(data.timestamps[valid_state], replay.states[valid_state, axis], "--", label=f"filtered {name}")
    axes[0, 1].set_title("Position state")
    axes[0, 1].set_xlabel("time (s)")
    axes[0, 1].set_ylabel("m")
    axes[0, 1].legend(ncol=2, fontsize=8)

    axes[1, 0].plot(data.timestamps[valid_state], error[valid_state], label="3-D error")
    axes[1, 0].axhline(1.0, color="r", linestyle="--", label="project RMSE KPI")
    for start, end in _dropout_blocks(data.timestamps, data.detected):
        axes[1, 0].axvspan(start, end, color="orange", alpha=0.2)
    axes[1, 0].set_title("Estimation error (orange = lost measurement)")
    axes[1, 0].set_xlabel("time (s)")
    axes[1, 0].set_ylabel("m")
    axes[1, 0].legend()

    axes[1, 1].plot(data.timestamps[valid_state], np.sum(replay.covariance_diagonal[valid_state, :3], axis=1), label="trace(P_position)")
    axes[1, 1].scatter(data.timestamps[data.detected], replay.nis[data.detected], s=8, label="NIS")
    axes[1, 1].axhline(16.27, color="r", linestyle="--", label="NIS gate")
    axes[1, 1].set_title("Uncertainty and measurement gate")
    axes[1, 1].set_xlabel("time (s)")
    axes[1, 1].legend()

    fig.suptitle(title)
    fig.tight_layout()
    fig.savefig(output_path, dpi=150)
    plt.close(fig)


def _dropout_blocks(timestamps: np.ndarray, detected: np.ndarray) -> list[tuple[float, float]]:
    blocks: list[tuple[float, float]] = []
    start = None
    for timestamp, is_detected in zip(timestamps, detected):
        if not is_detected and start is None:
            start = float(timestamp)
        if is_detected and start is not None:
            blocks.append((start, float(timestamp)))
            start = None
    if start is not None:
        blocks.append((start, float(timestamps[-1])))
    return blocks


def main() -> None:
    parser = argparse.ArgumentParser(description="Verify the hardware-independent target-state EKF")
    parser.add_argument("--trajectory", choices=("all", "straight", "circle", "figure8", "zigzag", "stop_go"), default="all")
    parser.add_argument("--model", choices=("cv", "ct", "imm", "all"), default="all", help="model to run (default: all three)")
    parser.add_argument("--output-dir", default="simulation_results/target_state_comparison")
    parser.add_argument("--seed", type=int, default=None, help="optional seed; omit for new random trajectories each run")
    random_group = parser.add_mutually_exclusive_group()
    random_group.add_argument("--random-trajectories", dest="random_trajectories", action="store_true", help="randomize trajectory parameters (default)")
    random_group.add_argument("--fixed-trajectories", dest="random_trajectories", action="store_false", help="use fixed legacy trajectories")
    parser.set_defaults(random_trajectories=True)
    args = parser.parse_args()
    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    scenarios = list(iter_default_scenarios(randomize=args.random_trajectories, seed=args.seed))
    if args.trajectory != "all":
        scenarios = [(name, config) for name, config in scenarios if name == args.trajectory]
    model_names = ("cv", "ct", "imm") if args.model == "all" else (args.model,)
    report = {model_name: {} for model_name in model_names}
    measurement_covariance = np.diag((0.10, 0.10, 0.08)) ** 2
    for name, config in scenarios:
        # Generate each random trajectory/measurement sequence once, then feed
        # that exact data to every model so no method gets a different draw.
        data = simulate_measurements(name, config)
        for model_name in model_names:
            if model_name == "ct":
                estimator = CoordinatedTurnEKF(acceleration_variance=(0.8, 0.8, 0.4), turn_rate_variance=0.08)
            elif model_name == "imm":
                estimator = TargetStateIMM()
            else:
                estimator = TargetStateEKF(process_accel_variance=(1.0, 1.0, 0.5))
            replay = replay_estimator(data, estimator=estimator, measurement_covariance=measurement_covariance)
            metrics = calculate_metrics(data, replay)
            report[model_name][name] = metrics
            model_output = output_dir / model_name if len(model_names) > 1 else output_dir
            model_output.mkdir(parents=True, exist_ok=True)
            plot_result(model_output / f"{name}.png", data, replay, f"{model_name.upper()} estimator: {name}")
            print(f"{model_name.upper():3s} | {name:8s} | filtered RMSE={metrics['filtered_position_rmse_m']:.3f} m | dropout max={metrics['max_error_during_dropout_m']:.3f} m")

    report_path = output_dir / "metrics.json"
    report_path.write_text(json.dumps({"seed": args.seed, "random_trajectories": args.random_trajectories, "models": report}, indent=2) + "\n", encoding="utf-8")
    print(f"Saved metrics and plots to {output_dir}")


if __name__ == "__main__":
    main()
