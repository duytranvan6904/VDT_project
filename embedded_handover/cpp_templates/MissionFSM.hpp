/**
 * @file MissionFSM.hpp
 * @brief End-to-End Mission Finite State Machine for Autonomous Precision Landing
 * States: IDLE -> SEARCH -> FOLLOW -> APPROACH -> GLIDE_SLOPE -> LAND -> TOUCHDOWN
 */

#pragma once

#include "Types.hpp"
#include <cmath>
#include <algorithm>

namespace vdt::landing {

struct MissionFSMConfig {
    double follow_distance{3.5};        // Standoff tracking distance (m)
    double alignment_threshold{0.3};    // Horizontal threshold to align (m)
    double takeoff_altitude{3.0};       // Target takeoff altitude (m)
    double search_timeout_s{1.2};       // Lost target timeout in FOLLOW -> SEARCH
    double approach_timeout_s{3.0};     // Lost target timeout in APPROACH -> FOLLOW
    double reacquire_confirm_time_s{0.25};
};

class MissionFSM {
public:
    explicit MissionFSM(const MissionFSMConfig& cfg = MissionFSMConfig{}) : cfg_(cfg) {
        reset();
    }

    void reset() {
        phase_ = MissionPhase::IDLE;
        timer_s_ = 0.0;
    }

    /**
     * @brief Step the state machine based on current observations.
     */
    MissionPhase step(
        double drone_alt,
        bool target_detected,
        TrackingMode tracking_mode,
        bool operator_land_cmd,
        bool safe_to_land,
        double uncertainty_2sigma,
        double h_err,
        bool touchdown_latched,
        double dt
    ) {
        timer_s_ += dt;

        switch (phase_) {
            case MissionPhase::IDLE:
                // Once drone reaches takeoff altitude (e.g., > 2.0m), switch to SEARCH
                if (drone_alt >= 0.8 * cfg_.takeoff_altitude) {
                    phase_ = MissionPhase::SEARCH;
                    timer_s_ = 0.0;
                }
                break;

            case MissionPhase::SEARCH:
                // When ArUco is stably detected and EKF is TRACKING
                if (target_detected && tracking_mode == TrackingMode::TRACKING) {
                    phase_ = MissionPhase::FOLLOW;
                    timer_s_ = 0.0;
                }
                break;

            case MissionPhase::FOLLOW:
                if (tracking_mode == TrackingMode::EXPIRED && timer_s_ > cfg_.search_timeout_s) {
                    phase_ = MissionPhase::SEARCH;
                    timer_s_ = 0.0;
                } else if (operator_land_cmd) {
                    phase_ = MissionPhase::APPROACH;
                    timer_s_ = 0.0;
                }
                break;

            case MissionPhase::APPROACH:
                if (!operator_land_cmd) {
                    // Operator abort
                    phase_ = MissionPhase::FOLLOW;
                    timer_s_ = 0.0;
                } else if (safe_to_land && uncertainty_2sigma <= 0.45 && h_err <= 1.5) {
                    // Handover to SMC Glide Slope
                    phase_ = MissionPhase::GLIDE_SLOPE;
                    timer_s_ = 0.0;
                } else if (tracking_mode == TrackingMode::EXPIRED && timer_s_ > cfg_.approach_timeout_s) {
                    // Timeout lost marker -> abort to FOLLOW
                    phase_ = MissionPhase::FOLLOW;
                    timer_s_ = 0.0;
                }
                break;

            case MissionPhase::GLIDE_SLOPE:
                if (!operator_land_cmd) {
                    phase_ = MissionPhase::FOLLOW;
                    timer_s_ = 0.0;
                } else if (drone_alt <= 0.40 && h_err <= 0.40 && uncertainty_2sigma <= 0.35) {
                    // Transition to final vertical landing
                    phase_ = MissionPhase::LAND;
                    timer_s_ = 0.0;
                } else if (h_err > 0.80 || uncertainty_2sigma > 0.55) {
                    // Wave-off abort to APPROACH
                    phase_ = MissionPhase::APPROACH;
                    timer_s_ = 0.0;
                }
                break;

            case MissionPhase::LAND:
                if (touchdown_latched || (drone_alt <= 0.12 && timer_s_ > 0.8)) {
                    phase_ = MissionPhase::TOUCHDOWN;
                    timer_s_ = 0.0;
                }
                break;

            case MissionPhase::TOUCHDOWN:
                // Motors force-disarmed via MAVLink (param2=21196.0)
                // Stay in TOUCHDOWN / return to IDLE
                break;
        }

        return phase_;
    }

    [[nodiscard]] MissionPhase get_phase() const { return phase_; }

private:
    MissionFSMConfig cfg_;
    MissionPhase phase_{MissionPhase::IDLE};
    double timer_s_{0.0};
};

} // namespace vdt::landing
