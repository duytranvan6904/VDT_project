/**
 * @file SMCGuidance.hpp
 * @brief Sliding Mode Control (SMC) Guidance for Precision Drone Landing
 * Implements 45° Glide Slope Descent, Power Reaching Law, and Soft Touchdown.
 */

#pragma once

#include "Types.hpp"
#include <cmath>
#include <algorithm>

namespace vdt::landing {

class SMCGuidance {
public:
    struct Config {
        double ka{0.20};            // Sliding surface S1 gain
        double kb{0.60};            // Sliding surface S2 gain (elevation)
        double kc{0.40};            // Sliding surface S3 gain (azimuth)
        double k1{0.1395};          // Reaching law gain 1
        double k2{0.1784};          // Reaching law gain 2
        double k3{0.0442};          // Reaching law gain 3
        double power_n_over_m{0.6}; // Power reaching index: 3/5 = 0.6
        double theta_des_rad{M_PI / 4.0}; // 45 deg glide slope
        double v_max_approach{1.50};      // m/s
        double v_final_xy_max{0.35};      // m/s
        double v_descend_fast{0.35};      // m/s in GLIDE_SLOPE
        double v_descend_touch{0.15};     // m/s soft touchdown in FINAL_DESCENT
        double final_centering_kp{0.70};  // P gain for XY centering
        double final_descent_alt_m{0.40}; // Transition alt to FINAL_DESCENT (m)
        double final_descent_rxy_m{0.40}; // Max horizontal distance to enter FINAL_DESCENT (m)
    };

    SMCGuidance() : SMCGuidance(Config{}) {}
    explicit SMCGuidance(const Config& cfg) : cfg_(cfg) {}

    /**
     * @brief Compute 3D velocity command in World ENU frame.
     * @param p_drone Drone position in World ENU frame
     * @param p_target Target pad position in World ENU frame
     * @param v_target Target pad velocity in World ENU frame (feedforward)
     * @param rswitch_adaptive Adaptive switching radius from CovarianceGate (m)
     */
    SMCResult compute(
        const Vector3& p_drone,
        const Vector3& p_target,
        const Vector3& v_target,
        double rswitch_adaptive
    ) {
        SMCResult res{};
        double dx = p_target.x - p_drone.x;
        double dy = p_target.y - p_drone.y;
        double dz = p_target.z - p_drone.z; // negative when drone is above pad

        double r_xy = std::hypot(dx, dy);
        double alt = std::abs(dz);
        res.r_xy = r_xy;
        res.r_z = alt;

        // Sub-phase determination
        bool enter_final = (alt <= cfg_.final_descent_alt_m && r_xy <= cfg_.final_descent_rxy_m);
        if (enter_final) {
            res.sub_phase = SMCSubPhase::FINAL_DESCENT;
        } else {
            res.sub_phase = SMCSubPhase::GLIDE_SLOPE;
        }

        if (res.sub_phase == SMCSubPhase::FINAL_DESCENT) {
            // Closed-loop precision XY centering directly over pad center
            double cmd_vx = std::clamp(cfg_.final_centering_kp * dx, -cfg_.v_final_xy_max, cfg_.v_final_xy_max) + v_target.x;
            double cmd_vy = std::clamp(cfg_.final_centering_kp * dy, -cfg_.v_final_xy_max, cfg_.v_final_xy_max) + v_target.y;
            // Gentle steady downward velocity to touch pad
            double cmd_vz = -cfg_.v_descend_touch;

            res.velocity_cmd = {cmd_vx, cmd_vy, cmd_vz};
            res.yaw_rate_cmd = 0.0;
            return res;
        }

        // GLIDE_SLOPE: 45° descent synchronized with horizontal pursuit
        // Elevation angle theta = atan2(alt, r_xy)
        double theta = std::atan2(alt, std::max(0.05, r_xy));
        double s2 = theta - cfg_.theta_des_rad; // Elevation sliding surface

        // Power reaching law: dot_s = -k2 * |s2|^0.6 * sgn(s2)
        double sgn_s2 = (s2 >= 0.0) ? 1.0 : -1.0;
        double reaching_s2 = -cfg_.k2 * std::pow(std::abs(s2), cfg_.power_n_over_m) * sgn_s2;

        res.sliding_surface[1] = s2;

        // Horizontal unit direction toward target
        double u_x = (r_xy > 1e-4) ? (dx / r_xy) : 0.0;
        double u_y = (r_xy > 1e-4) ? (dy / r_xy) : 0.0;

        // Regulate horizontal pursuit speed
        double v_h = std::clamp(cfg_.final_centering_kp * r_xy, 0.15, cfg_.v_max_approach);
        double cmd_vx = v_h * u_x + v_target.x;
        double cmd_vy = v_h * u_y + v_target.y;

        // Vertical descent is locked to 45 deg glide slope (v_z = -v_h for 45°)
        // with correction from reaching law
        double cmd_vz = -std::clamp(v_h + reaching_s2, 0.10, cfg_.v_descend_fast);

        res.velocity_cmd = {cmd_vx, cmd_vy, cmd_vz};
        res.yaw_rate_cmd = 0.0;
        return res;
    }

private:
    Config cfg_;
};

} // namespace vdt::landing
