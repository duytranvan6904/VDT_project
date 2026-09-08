import argparse
import json

from landing_diagnostics.classify import DriftCause, classify_drift
from landing_diagnostics.metrics import compute_stats
from landing_diagnostics.tuning import propose_param_change

CAUSE_MESSAGE = {
    DriftCause.UNDERDAMPED:
        'Dao dong quanh setpoint (underdamped) -> can giam P hoac tang D',
    DriftCause.STEADY_STATE_OFFSET:
        'Lech on dinh mot huong (co the do gio) -> can tang I',
    DriftCause.GROUND_EFFECT:
        'Sai so tang khi gan mat dat -> giam toc do ha canh',
    DriftCause.UNKNOWN:
        'Khong xac dinh la do PID -> kiem tra setpoint tu offboard_manager hoac EKF truoc',
}


def load_current_params(path):
    if path is None:
        return {}
    with open(path) as f:
        return json.load(f)


def print_report(stats, cause, suggestions):
    print('--- Landing tracking error ---')
    print(f'mean_err_xy   = {stats.mean_err_xy:.3f} m/s')
    print(f'max_err_xy    = {stats.max_err_xy:.3f} m/s')
    print(f'std_err_xy    = {stats.std_err_xy:.3f} m/s')
    print(f'mean_vx/vy    = {stats.mean_vx:.3f} / {stats.mean_vy:.3f} m/s')
    print(f'oscillation   = {stats.oscillation_freq_hz:.2f} Hz')
    print(f'altitude_corr = {stats.altitude_error_corr:.2f}')
    print(f'horiz_drift   = {stats.horizontal_drift_m:.3f} m')
    print()
    print(f'Chan doan: {cause.value}')
    print(CAUSE_MESSAGE[cause])

    if suggestions:
        print()
        print('De xuat tham so moi (thu tren SITL truoc):')
        for name, value in suggestions.items():
            print(f'  {name} = {value:.4f}')


def main():
    parser = argparse.ArgumentParser(description='Phan tich do lech khi ha canh offboard')
    parser.add_argument('csv_path')
    parser.add_argument('--params-json', default=None)
    args = parser.parse_args()

    stats = compute_stats(args.csv_path)
    cause = classify_drift(stats)
    current_params = load_current_params(args.params_json)
    suggestions = propose_param_change(cause, current_params)

    print_report(stats, cause, suggestions)


if __name__ == '__main__':
    main()