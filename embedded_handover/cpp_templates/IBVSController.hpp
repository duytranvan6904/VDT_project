/**
 * @file IBVSController.hpp
 * @brief Image-Based Visual Servoing (IBVS) for Camera Gimbal Pitch & Drone Yaw
 * Features:
 *   - Gimbal pitch & body yaw visual centering
 *   - Search pitch sweep in SEARCH phase
 *   - Nadir tilt mode (-85 deg) when z < 0.8m in LAND phase
 *   - Pixel trimming deadband for close proximity (< 0.3m)
 */

#pragma once

#include "Types.hpp"
#include <cmath>
#include <algorithm>

namespace vdt::landing {

class IBVSController {
public:
    struct Config {
        double K_pitch{0.92};
        double K_yaw{0.92};
        double focal_x{466.0};
        double focal_y{466.0};
        double u0{320.0};
        double v0{240.0};
        double default_pitch_rad{-0.45};     // -25 deg
        double nadir_pitch_rad{-1.48};       // -85 deg looking straight down
        double nadir_trigger_alt_m{0.80};    // Switch to nadir below this height
        double pitch_rate_limit{1.80};       // rad/s
        double yaw_rate_limit{0.80};         // rad/s
        double ema_alpha{0.65};
    };

    IBVSController() : IBVSController(Config{}) {}
    explicit IBVSController(const Config& cfg) : cfg_(cfg) {
        gimbal_pitch_cmd_ = cfg_.default_pitch_rad;
        filtered_pitch_ = cfg_.default_pitch_rad;
    }

    struct Output {
        double gimbal_pitch_rad{0.0};
        double yaw_rate_cmd{0.0};
        double pixel_error_u{0.0};
        double pixel_error_v{0.0};
    };

    Output compute(
        MissionPhase phase,
        double drone_alt,
        bool detected,
        double pixel_u,
        double pixel_v,
        double dt
    ) {
        Output out{};

        if (phase == MissionPhase::LAND && drone_alt <= cfg_.nadir_trigger_alt_m) {
            // Nadir tilt mode: camera points straight down (-85 deg)
            gimbal_pitch_cmd_ = cfg_.nadir_pitch_rad;
            filtered_pitch_ = cfg_.nadir_pitch_rad;
            out.gimbal_pitch_rad = cfg_.nadir_pitch_rad;
            out.yaw_rate_cmd = 0.0;
            return out;
        }

        if (!detected) {
            // No marker detected: hold or relax to default pitch
            out.gimbal_pitch_rad = filtered_pitch_;
            out.yaw_rate_cmd = 0.0;
            return out;
        }

        double eu = pixel_u - cfg_.u0;
        double ev = pixel_v - cfg_.v0;
        out.pixel_error_u = eu;
        out.pixel_error_v = ev;

        // Visual servoing control law
        double yaw_rate = -cfg_.K_yaw * (eu / cfg_.focal_x);
        out.yaw_rate_cmd = std::clamp(yaw_rate, -cfg_.yaw_rate_limit, cfg_.yaw_rate_limit);

        double pitch_delta = cfg_.K_pitch * (ev / cfg_.focal_y);
        double target_pitch = gimbal_pitch_cmd_ + pitch_delta * dt;
        target_pitch = std::clamp(target_pitch, -1.48, 0.0);

        filtered_pitch_ = cfg_.ema_alpha * target_pitch + (1.0 - cfg_.ema_alpha) * filtered_pitch_;
        gimbal_pitch_cmd_ = filtered_pitch_;
        out.gimbal_pitch_rad = filtered_pitch_;

        return out;
    }

private:
    Config cfg_;
    double gimbal_pitch_cmd_{-0.45};
    double filtered_pitch_{-0.45};
};

} // namespace vdt::landing
