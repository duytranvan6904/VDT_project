/**
 * @file TrackingControl.hpp
 * @brief Pure control helpers for Target Tracking and Motion Feedforward
 */

#pragma once

#include "Types.hpp"
#include <cmath>
#include <algorithm>

namespace vdt::landing {

class TargetMotionGate {
public:
    explicit TargetMotionGate(
        double enter_speed = 0.35,
        double exit_speed = 0.18,
        int enter_confirm_frames = 3,
        int exit_confirm_frames = 4
    )
        : enter_speed_(enter_speed),
          exit_speed_(exit_speed),
          enter_confirm_frames_(enter_confirm_frames),
          exit_confirm_frames_(exit_confirm_frames) {}

    void reset() {
        active_ = false;
        enter_count_ = 0;
        exit_count_ = 0;
    }

    /**
     * @brief Update gate with estimated target horizontal velocity [vx, vy].
     * @return Filtered velocity: returns [vx, vy] if active, or [0, 0] if stationary.
     */
    Vector3 update(const Vector3& target_vel_xy) {
        double speed = target_vel_xy.norm_xy();

        if (active_) {
            enter_count_ = 0;
            if (speed <= exit_speed_) {
                exit_count_++;
                if (exit_count_ >= exit_confirm_frames_) {
                    reset();
                }
            } else {
                exit_count_ = 0;
            }
        } else {
            exit_count_ = 0;
            if (speed >= enter_speed_) {
                enter_count_++;
                if (enter_count_ >= enter_confirm_frames_) {
                    active_ = true;
                    enter_count_ = 0;
                }
            } else {
                enter_count_ = 0;
            }
        }

        if (active_) {
            return {target_vel_xy.x, target_vel_xy.y, 0.0};
        }
        return {0.0, 0.0, 0.0};
    }

    [[nodiscard]] bool is_active() const { return active_; }

private:
    double enter_speed_{0.35};
    double exit_speed_{0.18};
    int enter_confirm_frames_{3};
    int exit_confirm_frames_{4};
    bool active_{false};
    int enter_count_{0};
    int exit_count_{0};
};

inline Vector3 compose_follow_velocity(
    const Vector3& guidance_xy,
    const Vector3& target_ff_xy,
    double alignment_scale,
    double max_speed
) {
    double scale = std::clamp(alignment_scale, 0.0, 1.0);
    Vector3 vel{
        scale * guidance_xy.x + target_ff_xy.x,
        scale * guidance_xy.y + target_ff_xy.y,
        0.0
    };
    double speed = vel.norm_xy();
    if (speed > max_speed && speed > 1e-6) {
        vel = vel * (max_speed / speed);
    }
    return vel;
}

} // namespace vdt::landing
