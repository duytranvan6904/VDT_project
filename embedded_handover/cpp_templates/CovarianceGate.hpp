/**
 * @file CovarianceGate.hpp
 * @brief Relative Dual-EKF Covariance Gating & Safety Monitor
 * Fuses P_relative = P_target + P_drone to evaluate 2-sigma ellipse against landing pad.
 */

#pragma once

#include "Types.hpp"
#include <cmath>
#include <algorithm>
#include <string>

namespace vdt::landing {

class CovarianceGate {
public:
    struct Config {
        double pad_radius_m{0.25};                // Physical pad radius (m)
        double confidence_sigma{2.0};             // 2-sigma boundary
        double max_uncertainty_enter_glide{0.45}; // Enter GLIDE_SLOPE limit (m)
        double max_uncertainty_continue_glide{0.50};
        double max_uncertainty_final_descent{0.35}; // Enter FINAL_DESCENT limit (m)
        double conical_gate_angle_deg{30.0};      // Half-angle of safety cone (deg)
        double rswitch_min{4.5};                  // Min adaptive R_switch (m)
        double rswitch_max{10.0};                 // Max adaptive R_switch (m)
        double rswitch_base_m{5.0};
        double rswitch_alpha{2.0};
        double sliding_beta{1.5};
        double max_measurement_age_s{0.50};
    };

    CovarianceGate() : CovarianceGate(Config{}) {}
    explicit CovarianceGate(const Config& cfg) : cfg_(cfg) {}

    /**
     * @brief Evaluate relative covariance safety.
     * @param p_drone Drone position in World ENU frame
     * @param p_target Target pad position in World ENU frame
     * @param cov_drone_xy Covariance matrix elements [var_xx, cov_xy, cov_yx, var_yy] of drone
     * @param cov_target_xy Covariance matrix elements [var_xx, cov_xy, cov_yx, var_yy] of target
     * @param measurement_age_s Age of target measurement (s)
     */
    CovarianceStatusResult evaluate(
        const Vector3& p_drone,
        const Vector3& p_target,
        const std::array<double, 4>& cov_drone_xy,
        const std::array<double, 4>& cov_target_xy,
        double measurement_age_s
    ) const {
        CovarianceStatusResult res{};
        res.measurement_age_s = measurement_age_s;

        // Relative position
        double dx = p_drone.x - p_target.x;
        double dy = p_drone.y - p_target.y;
        double dz = std::abs(p_drone.z - p_target.z);
        res.relative_dist_xy = std::hypot(dx, dy);
        res.relative_dist_z = dz;

        // Sum relative covariance in XY
        double s_xx = cov_drone_xy[0] + cov_target_xy[0];
        double s_xy = cov_drone_xy[1] + cov_target_xy[1];
        double s_yy = cov_drone_xy[3] + cov_target_xy[3];

        // Eigenvalues of symmetric 2x2 matrix:
        // lambda = (tr / 2) +- sqrt((tr/2)^2 - det)
        double tr = s_xx + s_yy;
        double det = s_xx * s_yy - s_xy * s_xy;
        double disc = std::max(0.0, (tr * tr * 0.25) - det);
        double lambda_max = 0.5 * tr + std::sqrt(disc);
        res.lambda_max_2d = lambda_max;

        // 2-sigma uncertainty radius
        res.r_uncertainty_2sigma = cfg_.confidence_sigma * std::sqrt(std::max(0.0, lambda_max));

        // Adaptive switching radius
        res.rswitch_adaptive = std::clamp(
            cfg_.rswitch_base_m + cfg_.rswitch_alpha * res.r_uncertainty_2sigma,
            cfg_.rswitch_min,
            cfg_.rswitch_max
        );

        // Sliding surface weight
        res.sliding_weight = std::exp(-cfg_.sliding_beta * res.r_uncertainty_2sigma);

        // Safety gating checks
        if (measurement_age_s > cfg_.max_measurement_age_s) {
            res.safe_to_land = false;
            res.reject_reason = "Target measurement expired";
            return res;
        }

        if (res.r_uncertainty_2sigma > cfg_.max_uncertainty_enter_glide) {
            res.safe_to_land = false;
            res.reject_reason = "Uncertainty radius exceeds threshold";
            return res;
        }

        // Conical safety gate check: theta_cone = atan2(r_xy, dz) <= 30 deg
        if (dz > 0.1) {
            double cone_angle_rad = std::atan2(res.relative_dist_xy, dz);
            double cone_angle_deg = cone_angle_rad * 180.0 / M_PI;
            if (cone_angle_deg > cfg_.conical_gate_angle_deg) {
                res.safe_to_land = false;
                res.reject_reason = "Drone outside 30 deg conical safety funnel";
                return res;
            }
        }

        res.safe_to_land = true;
        return res;
    }

private:
    Config cfg_;
};

} // namespace vdt::landing
