/**
 * @file TargetStateEKF.hpp
 * @brief 6-State Constant Velocity Extended Kalman Filter for Landing Pad Tracking
 * State vector: [x, y, z, vx, vy, vz]^T in World ENU Frame
 */

#pragma once

#include "Types.hpp"
#include <array>
#include <cmath>
#include <algorithm>

namespace vdt::landing {

class TargetStateEKF {
public:
    struct Config {
        double process_accel_variance_xy{1.0}; // q_xy (m^2/s^3)
        double process_accel_variance_z{0.5};  // q_z (m^2/s^3)
        double measurement_variance_xy{0.04};  // r_xy (m^2)
        double measurement_variance_z{0.09};   // r_z (m^2)
        double gate_threshold{16.27};          // Mahalanobis chi-squared 3DOF (p=0.001)
        double v_max_xy{2.5};                  // Max horizontal speed clamp (m/s)
        double v_max_z{1.5};                   // Max vertical speed clamp (m/s)
        double max_dv_per_step{0.40};          // Limit velocity innovation jump per measurement (m/s)
        double stale_timeout_s{1.2};           // Switch from TRACKING -> EXPIRED
    };

    TargetStateEKF() : TargetStateEKF(Config{}) {}
    explicit TargetStateEKF(const Config& cfg) : cfg_(cfg) {
        reset();
    }

    void reset() {
        state_.fill(0.0);
        covariance_.fill(0.0);
        // Initial diagonal covariance
        covariance_[0] = 1.0;  // var(x)
        covariance_[7] = 1.0;  // var(y)
        covariance_[14] = 1.0; // var(z)
        covariance_[21] = 1.0; // var(vx)
        covariance_[28] = 1.0; // var(vy)
        covariance_[35] = 0.5; // var(vz)
        initialized_ = false;
        last_timestamp_s_ = 0.0;
        tracking_mode_ = TrackingMode::IDLE;
    }

    void initialize(const Vector3& pos, double timestamp_s) {
        state_[0] = pos.x;
        state_[1] = pos.y;
        state_[2] = pos.z;
        state_[3] = 0.0;
        state_[4] = 0.0;
        state_[5] = 0.0;
        last_timestamp_s_ = timestamp_s;
        initialized_ = true;
        tracking_mode_ = TrackingMode::TRACKING;
    }

    /**
     * @brief Predict state forward by dt.
     * State transition: x_{k+1} = x_k + v_k * dt, v_{k+1} = v_k
     */
    void predict(double dt) {
        if (!initialized_ || dt <= 0.0) return;
        state_[0] += state_[3] * dt;
        state_[1] += state_[4] * dt;
        state_[2] += state_[5] * dt;

        clamp_velocity();

        // Discrete White Noise Acceleration Model (CWNA)
        double dt2 = dt * dt;
        double dt3 = dt2 * dt;

        for (int i = 0; i < 3; ++i) {
            double q = (i < 2) ? cfg_.process_accel_variance_xy : cfg_.process_accel_variance_z;
            double p_pp = get_cov(i, i);
            double p_pv = get_cov(i, i + 3);
            double p_vv = get_cov(i + 3, i + 3);

            double p_pp_next = p_pp + 2.0 * dt * p_pv + dt2 * p_vv + (dt3 / 3.0) * q;
            double p_pv_next = p_pv + dt * p_vv + (dt2 / 2.0) * q;
            double p_vv_next = p_vv + dt * q;

            set_cov(i, i, p_pp_next);
            set_cov(i, i + 3, p_pv_next);
            set_cov(i + 3, i, p_pv_next);
            set_cov(i + 3, i + 3, p_vv_next);
        }
    }

    /**
     * @brief Update filter with 3D position measurement in World frame.
     * Includes Mahalanobis distance gating to reject outliers.
     */
    bool update(const Vector3& z_meas, double timestamp_s) {
        if (!initialized_) {
            initialize(z_meas, timestamp_s);
            return true;
        }

        double dt = timestamp_s - last_timestamp_s_;
        if (dt > 0.0) {
            predict(dt);
            last_timestamp_s_ = timestamp_s;
        }

        // Residual y = z - H * x
        double res[3] = {
            z_meas.x - state_[0],
            z_meas.y - state_[1],
            z_meas.z - state_[2]
        };

        // Innovation covariance S = H * P * H^T + R
        double s_xx = get_cov(0, 0) + cfg_.measurement_variance_xy;
        double s_yy = get_cov(1, 1) + cfg_.measurement_variance_xy;
        double s_zz = get_cov(2, 2) + cfg_.measurement_variance_z;

        // Mahalanobis distance d^2 = y^T * S^-1 * y
        double mahalanobis = (res[0] * res[0]) / s_xx + (res[1] * res[1]) / s_yy + (res[2] * res[2]) / s_zz;
        if (mahalanobis > cfg_.gate_threshold) {
            // Outlier rejected!
            return false;
        }

        // Kalman gain and state update per axis
        double r_var[3] = {
            cfg_.measurement_variance_xy,
            cfg_.measurement_variance_xy,
            cfg_.measurement_variance_z
        };

        for (int i = 0; i < 3; ++i) {
            double p_pp = get_cov(i, i);
            double p_pv = get_cov(i, i + 3);
            double p_vv = get_cov(i + 3, i + 3);
            double s = p_pp + r_var[i];

            double k_p = p_pp / s;
            double k_v = p_pv / s;

            state_[i] += k_p * res[i];

            double delta_v = k_v * res[i];
            if (i < 2) {
                delta_v = std::clamp(delta_v, -cfg_.max_dv_per_step, cfg_.max_dv_per_step);
            }
            state_[i + 3] += delta_v;

            // P = (I - K*H) * P
            set_cov(i, i, (1.0 - k_p) * p_pp);
            set_cov(i, i + 3, (1.0 - k_p) * p_pv);
            set_cov(i + 3, i, (1.0 - k_p) * p_pv);
            set_cov(i + 3, i + 3, std::max(1e-4, p_vv - k_v * p_pv));
        }

        clamp_velocity();
        tracking_mode_ = TrackingMode::TRACKING;
        return true;
    }

    [[nodiscard]] Vector3 get_position() const {
        return {state_[0], state_[1], state_[2]};
    }

    [[nodiscard]] Vector3 get_velocity() const {
        return {state_[3], state_[4], state_[5]};
    }

    [[nodiscard]] const std::array<double, 36>& get_covariance() const {
        return covariance_;
    }

    [[nodiscard]] TrackingMode get_tracking_mode() const {
        return tracking_mode_;
    }

    [[nodiscard]] bool is_initialized() const {
        return initialized_;
    }

private:
    double get_cov(int r, int c) const {
        return covariance_[r * 6 + c];
    }
    void set_cov(int r, int c, double val) {
        covariance_[r * 6 + c] = val;
    }

    void clamp_velocity() {
        double v_xy = std::hypot(state_[3], state_[4]);
        if (v_xy > cfg_.v_max_xy && v_xy > 1e-6) {
            state_[3] *= (cfg_.v_max_xy / v_xy);
            state_[4] *= (cfg_.v_max_xy / v_xy);
        }
        state_[5] = std::clamp(state_[5], -cfg_.v_max_z, cfg_.v_max_z);
    }

    Config cfg_;
    std::array<double, 6> state_{};
    std::array<double, 36> covariance_{};
    bool initialized_{false};
    double last_timestamp_s_{0.0};
    TrackingMode tracking_mode_{TrackingMode::IDLE};
};

} // namespace vdt::landing
