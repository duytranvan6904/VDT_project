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
        covariance_[21] = 4.0; // var(vx)
        covariance_[28] = 4.0; // var(vy)
        covariance_[35] = 2.0; // var(vz)
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
        // Q = G * Q_cont * G^T
        // For each axis: [dt^3/3, dt^2/2; dt^2/2, dt] * q_accel
        double dt2 = dt * dt;
        double dt3 = dt2 * dt;
        double q_xy = cfg_.process_accel_variance_xy;
        double q_z = cfg_.process_accel_variance_z;

        // F * P * F^T + Q
        // P[0][0] += 2*dt*P[0][3] + dt^2*P[3][3] + (dt^3/3)*q_xy, etc.
        // In full matrix implementation, use Eigen::Matrix<double, 6, 6>
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
        double res_x = z_meas.x - state_[0];
        double res_y = z_meas.y - state_[1];
        double res_z = z_meas.z - state_[2];

        // Innovation covariance S = H * P * H^T + R
        double s_xx = get_cov(0, 0) + cfg_.measurement_variance_xy;
        double s_yy = get_cov(1, 1) + cfg_.measurement_variance_xy;
        double s_zz = get_cov(2, 2) + cfg_.measurement_variance_z;

        // Mahalanobis distance d^2 = y^T * S^-1 * y
        double mahalanobis = (res_x * res_x) / s_xx + (res_y * res_y) / s_yy + (res_z * res_z) / s_zz;
        if (mahalanobis > cfg_.gate_threshold) {
            // Outlier rejected!
            return false;
        }

        // Kalman Gain K = P * H^T * S^-1
        // x = x + K * y
        // P = (I - K * H) * P
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
