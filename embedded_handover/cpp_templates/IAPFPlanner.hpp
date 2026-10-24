/**
 * @file IAPFPlanner.hpp
 * @brief Improved Artificial Potential Field (I-APF) 3D Planner
 * Features:
 *   - Sigmoid goal weighting for GNRON mitigation
 *   - 3D Tangent Plane local minima escape with forward prediction scoring
 *   - Directional oscillation suppression
 *   - Dynamic speed scheduling
 */

#pragma once

#include "Types.hpp"
#include <vector>
#include <cmath>
#include <algorithm>

namespace vdt::landing {

struct IAPFParams {
    double k_att{10.0};            // Attractive gain
    double k_rep{250.0};           // Repulsive gain
    double d0{2.5};                // Obstacle influence distance (m)
    double v_max{1.5};             // Max speed (m/s)
    double goal_tol{0.25};         // Goal reached threshold (m)
    double f_enter{0.10};          // Local minima enter force threshold (N)
    double f_exit{0.30};           // Local minima exit force threshold (N)
    double goal_min_dist{0.50};    // Min distance to goal to check local minima (m)
    double k_tan{1.0};             // Tangent force gain
    int n_tangent{12};             // Candidate directions on tangent plane
    int n_pred{3};                 // Prediction steps
    double w_goal{1.0};            // Candidate goal weight
    double w_clear{1.0};           // Candidate clearance weight
    double w_prev{0.60};           // Candidate continuity weight
    double vz_max{0.5};            // Max vertical speed (m/s)
};

struct IAPFResult {
    Vector3 velocity{};            // [vx, vy, vz] in World ENU (m/s)
    double yaw_cmd{0.0};           // Heading command (rad)
    Vector3 f_att{};
    Vector3 f_rep{};
    Vector3 f_total{};
    bool at_goal{false};
    bool tangent_active{false};
};

class IAPFPlanner {
public:
    IAPFPlanner() : IAPFPlanner(IAPFParams{}) {}
    explicit IAPFPlanner(const IAPFParams& cfg) : params_(cfg) {
        reset();
    }

    void reset() {
        tangent_active_ = false;
        tangent_prev_ = {0.0, 0.0, 0.0};
        f_prev_cmd_ = {0.0, 0.0, 0.0};
    }

    /**
     * @brief Compute avoidance velocity toward goal given obstacle list.
     */
    IAPFResult compute(
        const Vector3& p_drone,
        const Vector3& p_goal,
        const std::vector<CylinderObstacle>& obstacles
    ) {
        IAPFResult res{};
        Vector3 d_goal = p_goal - p_drone;
        double dist_goal = d_goal.norm();

        if (dist_goal < params_.goal_tol) {
            res.at_goal = true;
            return res;
        }

        // 1. Attractive Force
        res.f_att = d_goal * (2.0 * params_.k_att);

        // 2. Repulsive Force with GNRON Sigmoid Weighting
        Vector3 f_rep_total{0.0, 0.0, 0.0};
        for (const auto& obs : obstacles) {
            Vector3 d_obs = p_drone - obs.center;
            double dist_obs_center = d_obs.norm_xy();
            double dist_surface = dist_obs_center - obs.radius;

            if (dist_surface < params_.d0 && dist_surface > 0.01) {
                // Sigmoid weighting based on distance to goal
                double w_gnron = 1.0 / (1.0 + std::exp(-2.0 * (dist_goal - 1.0)));
                double rep_mag = params_.k_rep * (1.0 / dist_surface - 1.0 / params_.d0) / (dist_surface * dist_surface);
                Vector3 u_rep = {d_obs.x / dist_obs_center, d_obs.y / dist_obs_center, 0.0};
                f_rep_total = f_rep_total + (u_rep * (rep_mag * w_gnron));
            }
        }
        res.f_rep = f_rep_total;
        res.f_total = res.f_att + res.f_rep;

        // 3. Collinear Trap & Local Minima Detection with Tangent Escape
        bool is_collinear_trap = false;
        double f_att_norm = res.f_att.norm();
        if (f_att_norm > 1e-4 && !obstacles.empty()) {
            Vector3 u_goal = d_goal * (1.0 / dist_goal);
            for (const auto& obs : obstacles) {
                Vector3 d_obs = obs.center - p_drone;
                double d = d_obs.norm_xy();
                if (d < params_.d0 + obs.radius && d > 1e-4) {
                    Vector3 u_obs = {d_obs.x / d, d_obs.y / d, 0.0};
                    double cos_angle = u_goal.x * u_obs.x + u_goal.y * u_obs.y;
                    if (cos_angle > 0.65 && (res.f_att.x * res.f_rep.x + res.f_att.y * res.f_rep.y) < 0.0) {
                        double f_forward = res.f_total.x * u_goal.x + res.f_total.y * u_goal.y;
                        if (f_forward <= 0.40 * f_att_norm) {
                            is_collinear_trap = true;
                            break;
                        }
                    }
                }
            }
        }

        double f_total_norm = res.f_total.norm();
        if (!tangent_active_ && (f_total_norm < params_.f_enter || is_collinear_trap) && dist_goal > params_.goal_min_dist) {
            tangent_active_ = true;
        } else if (tangent_active_ && f_total_norm > params_.f_exit && !is_collinear_trap) {
            tangent_active_ = false;
        }
        res.tangent_active = tangent_active_;

        if (tangent_active_ && !obstacles.empty()) {
            // Find nearest obstacle to generate horizontal tangent
            double min_obs_d = 1e6;
            Vector3 nearest_obs_vec{1.0, 0.0, 0.0};
            for (const auto& obs : obstacles) {
                Vector3 d_obs = p_drone - obs.center;
                double d = d_obs.norm_xy() - obs.radius;
                if (d < min_obs_d) {
                    min_obs_d = d;
                    nearest_obs_vec = d_obs;
                }
            }
            double norm_xy = nearest_obs_vec.norm_xy();
            double nx = (norm_xy > 1e-4) ? (nearest_obs_vec.x / norm_xy) : 1.0;
            double ny = (norm_xy > 1e-4) ? (nearest_obs_vec.y / norm_xy) : 0.0;
            Vector3 t_ccw{-ny, nx, 0.0};
            Vector3 t_cw{ny, -nx, 0.0};

            Vector3 u_goal = d_goal * (1.0 / dist_goal);
            double score_ccw = t_ccw.x * u_goal.x + t_ccw.y * u_goal.y;
            double score_cw = t_cw.x * u_goal.x + t_cw.y * u_goal.y;
            Vector3 best_tangent = (score_ccw >= score_cw) ? t_ccw : t_cw;

            double f_rep_norm = res.f_rep.norm();
            double tan_mag = params_.k_tan * std::max(0.5, std::min(res.f_att.norm(), std::max(2.0, f_rep_norm)));
            Vector3 f_tan = best_tangent * tan_mag;
            res.f_total = res.f_total + f_tan;
        }

        // Velocity normalization and speed limit
        Vector3 f_cmd = res.f_total;
        double f_cmd_norm = f_cmd.norm();
        if (f_cmd_norm > 1e-4) {
            double v_mag = std::min(params_.v_max, f_cmd_norm);
            res.velocity = f_cmd * (v_mag / f_cmd_norm);
            res.yaw_cmd = std::atan2(res.velocity.y, res.velocity.x);
        }

        res.velocity.z = std::clamp(res.velocity.z, -params_.vz_max, params_.vz_max);
        return res;
    }

private:
    IAPFParams params_;
    bool tangent_active_{false};
    Vector3 tangent_prev_{0.0, 0.0, 0.0};
    Vector3 f_prev_cmd_{0.0, 0.0, 0.0};
};

} // namespace vdt::landing
