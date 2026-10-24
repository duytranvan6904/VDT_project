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
    double follow_distance{2.2};        // Standoff tracking distance (m)
    double alignment_threshold{0.3};    // Horizontal threshold to align (m)
    double takeoff_altitude{3.0};       // Target takeoff altitude (m)
    double search_timeout_s{5.0};       // Lost target timeout in FOLLOW -> SEARCH
    double approach_timeout_s{7.0};     // Lost target timeout in APPROACH -> FOLLOW
    double reacquire_confirm_time_s{0.20};
    double final_descent_alt_m{0.60};   // Transition altitude to LAND (m)
    double final_descent_rxy_m{0.85};   // Transition horizontal radius to LAND (m)
    bool auto_reland_after_abort{false};// Safety policy: require new operator command after waveoff/abort
};

class MissionFSM {
public:
    explicit MissionFSM(const MissionFSMConfig& cfg = MissionFSMConfig{}) : cfg_(cfg) {
        reset();
    }

    void reset() {
        phase_ = MissionPhase::IDLE;
        timer_s_ = 0.0;
        land_latched_ = false;
    }

    void request_landing() { land_latched_ = true; }
    void cancel_landing() { land_latched_ = false; }
    [[nodiscard]] bool is_landing_latched() const { return land_latched_; }

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
        if (operator_land_cmd) {
            land_latched_ = true;
        }

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
                } else if (land_latched_) {
                    phase_ = MissionPhase::APPROACH;
                    timer_s_ = 0.0;
                }
                break;

            case MissionPhase::APPROACH:
                if (!land_latched_) {
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
                    if (!cfg_.auto_reland_after_abort) {
                        land_latched_ = false;
                    }
                }
                break;

            case MissionPhase::GLIDE_SLOPE:
                if (!land_latched_) {
                    phase_ = MissionPhase::FOLLOW;
                    timer_s_ = 0.0;
                } else if (drone_alt <= cfg_.final_descent_alt_m && h_err <= cfg_.final_descent_rxy_m && uncertainty_2sigma <= 0.90) {
                    // Transition to final vertical landing
                    phase_ = MissionPhase::LAND;
                    timer_s_ = 0.0;
                } else if (h_err > 1.25 || uncertainty_2sigma > 0.95) {
                    // Wave-off abort to APPROACH
                    phase_ = MissionPhase::APPROACH;
                    timer_s_ = 0.0;
                    if (!cfg_.auto_reland_after_abort) {
                        land_latched_ = false;
                        phase_ = MissionPhase::FOLLOW; // Return all the way to FOLLOW to hold safely at altitude
                    }
                }
                break;

            case MissionPhase::LAND:
                if (touchdown_latched || (drone_alt <= 0.14 && timer_s_ > 0.35)) {
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
    bool land_latched_{false};
};

} // namespace vdt::landing
