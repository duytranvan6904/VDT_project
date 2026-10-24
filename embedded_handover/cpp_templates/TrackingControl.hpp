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
        double enter_speed = 0.38,
        double exit_speed = 0.18,
        int enter_confirm_frames = 15,
        int exit_confirm_frames = 8
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
                taper_scale_ = std::clamp(
                    1.0 - static_cast<double>(exit_count_) / std::max(1, exit_confirm_frames_),
                    0.0, 1.0
                );
                if (exit_count_ >= exit_confirm_frames_) {
                    reset();
                }
            } else {
                exit_count_ = 0;
                taper_scale_ = 1.0;
            }
        } else {
            exit_count_ = 0;
            taper_scale_ = 1.0;
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
            return {target_vel_xy.x * taper_scale_, target_vel_xy.y * taper_scale_, 0.0};
        }
        return {0.0, 0.0, 0.0};
    }

    [[nodiscard]] bool is_active() const { return active_; }

private:
    double enter_speed_{0.38};
    double exit_speed_{0.18};
    int enter_confirm_frames_{15};
    int exit_confirm_frames_{8};
    bool active_{false};
    int enter_count_{0};
    int exit_count_{0};
    double taper_scale_{1.0};
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

inline double wrap_angle(double angle_rad) {
    while (angle_rad > M_PI) angle_rad -= 2.0 * M_PI;
    while (angle_rad < -M_PI) angle_rad += 2.0 * M_PI;
    return angle_rad;
}

inline double landing_pitch_from_geometry(
    double drone_altitude,
    double target_surface_altitude,
    double horizontal_range
) {
    double dz = drone_altitude - target_surface_altitude;
    double horiz = std::max(1e-3, horizontal_range);
    return -std::atan2(dz, horiz);
}

inline double landing_pitch_target(
    double drone_altitude,
    double target_surface_altitude,
    double horizontal_range,
    bool target_state_fresh,
    double pixel_v = 0.0,
    bool has_pixel = false,
    double image_center_v = 240.0,
    double focal_y = 466.0,
    double pixel_gain = 0.92,
    double pixel_trim_gain = 0.30
) {
    if (!target_state_fresh) {
        return -M_PI / 4.0; // default 45 deg if not fresh
    }
    double target = landing_pitch_from_geometry(
        drone_altitude, target_surface_altitude, horizontal_range
    );
    if (has_pixel) {
        double fy = std::max(1.0, focal_y);
        double error = pixel_v - image_center_v;
        double deadband = 6.0;
        if (std::abs(error) > deadband) {
            double effective_error = error - std::copysign(deadband, error);
            target -= pixel_gain * (effective_error / fy) * pixel_trim_gain;
        }
    }
    return std::clamp(target, -M_PI / 2.0, -10.0 * M_PI / 180.0);
}

class FollowYawReferenceSelector {
public:
    explicit FollowYawReferenceSelector(
        double enter_speed = 0.35,
        double exit_speed = 0.18,
        double enter_confirm_duration_s = 0.75,
        double exit_confirm_duration_s = 0.50,
        double velocity_filter_time_constant_s = 0.25,
        double course_consistency_deg = 40.0
    )
        : enter_speed_(enter_speed),
          exit_speed_(exit_speed),
          enter_confirm_duration_(enter_confirm_duration_s),
          exit_confirm_duration_(exit_confirm_duration_s),
          velocity_filter_time_constant_(std::max(1e-3, velocity_filter_time_constant_s)),
          course_consistency_(course_consistency_deg * M_PI / 180.0) {}

    void reset(double initial_yaw) {
        moving_ = false;
        target_course_yaw_ = initial_yaw;
        filtered_vx_ = 0.0;
        filtered_vy_ = 0.0;
        has_last_sample_ = false;
        has_candidate_ = false;
        entry_elapsed_ = 0.0;
        exit_elapsed_ = 0.0;
    }

    [[nodiscard]] bool following_target_course() const { return moving_; }

    double update(
        double ibvs_yaw,
        double target_vx,
        double target_vy,
        double current_yaw,
        double previous_ref_yaw,
        bool tracking_valid,
        bool target_state_fresh,
        double sample_time_s
    ) {
        bool velocity_valid = std::isfinite(target_vx) && std::isfinite(target_vy);
        bool may_update = tracking_valid && target_state_fresh && velocity_valid && std::isfinite(sample_time_s);

        if (may_update) {
            double dt = has_last_sample_ ? std::clamp(sample_time_s - last_sample_time_, 0.0, 0.20) : 0.0;
            last_sample_time_ = sample_time_s;
            has_last_sample_ = true;

            double alpha = 1.0 - std::exp(-dt / velocity_filter_time_constant_);
            filtered_vx_ += alpha * (target_vx - filtered_vx_);
            filtered_vy_ += alpha * (target_vy - filtered_vy_);
            double speed = std::hypot(filtered_vx_, filtered_vy_);

            if (moving_) {
                if (speed <= exit_speed_) {
                    exit_elapsed_ += dt;
                    if (exit_elapsed_ >= exit_confirm_duration_) {
                        moving_ = false;
                        has_candidate_ = false;
                        entry_elapsed_ = 0.0;
                        exit_elapsed_ = 0.0;
                    }
                } else {
                    exit_elapsed_ = 0.0;
                    target_course_yaw_ = std::atan2(filtered_vy_, filtered_vx_);
                }
            } else if (speed >= enter_speed_) {
                double candidate_yaw = std::atan2(filtered_vy_, filtered_vx_);
                if (!has_candidate_ || std::abs(wrap_angle(candidate_yaw - candidate_course_yaw_)) > course_consistency_) {
                    candidate_course_yaw_ = candidate_yaw;
                    has_candidate_ = true;
                    entry_elapsed_ = 0.0;
                } else {
                    candidate_course_yaw_ = wrap_angle(candidate_course_yaw_ + 0.25 * wrap_angle(candidate_yaw - candidate_course_yaw_));
                    entry_elapsed_ += dt;
                }
                if (entry_elapsed_ >= enter_confirm_duration_) {
                    moving_ = true;
                    exit_elapsed_ = 0.0;
                    target_course_yaw_ = candidate_course_yaw_;
                }
            } else {
                has_candidate_ = false;
                entry_elapsed_ = 0.0;
            }
        }

        if (!tracking_valid) {
            return current_yaw;
        }
        if (!target_state_fresh) {
            return moving_ ? target_course_yaw_ : previous_ref_yaw;
        }
        if (moving_) {
            return target_course_yaw_;
        }
        return ibvs_yaw;
    }

private:
    double enter_speed_{0.35};
    double exit_speed_{0.18};
    double enter_confirm_duration_{0.75};
    double exit_confirm_duration_{0.50};
    double velocity_filter_time_constant_{0.25};
    double course_consistency_{40.0 * M_PI / 180.0};
    bool moving_{false};
    double target_course_yaw_{0.0};
    double filtered_vx_{0.0};
    double filtered_vy_{0.0};
    double last_sample_time_{0.0};
    bool has_last_sample_{false};
    double candidate_course_yaw_{0.0};
    bool has_candidate_{false};
    double entry_elapsed_{0.0};
    double exit_elapsed_{0.0};
};

} // namespace vdt::landing
