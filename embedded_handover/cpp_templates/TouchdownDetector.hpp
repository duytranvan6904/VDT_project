/**
 * @file TouchdownDetector.hpp
 * @brief Drift-Invariant Touchdown Detector for Precision Drone Landing
 * Confirms ground contact via Kinematic Stoppage + Descent Command + 0.35s Persistence Latch.
 */

#pragma once

#include "Types.hpp"
#include <cmath>
#include <string>

namespace vdt::landing {

class TouchdownDetector {
public:
    struct Config {
        double altitude_ceiling_m{0.14};        // Maximum actual altitude to consider touching down (m)
        double stoppage_vz_max_mps{0.12};       // Actual vertical velocity threshold (m/s)
        double stoppage_dz_dt_max_mps{0.08};    // Max rate of altitude change (m/s)
        double descent_cmd_threshold_mps{-0.10};// Must be commanding downward motion (m/s)
        double confirmation_duration_s{0.18};   // Continuous duration required to latch (s)
        double fast_confirmation_duration_s{0.10}; // Fast confirmation when impact or firmware landed flag confirmed (s)
    };

    TouchdownDetector() : TouchdownDetector(Config{}) {}
    explicit TouchdownDetector(const Config& cfg) : cfg_(cfg) {}

    void reset() {
        latched_touchdown_ = false;
        confirm_timer_s_ = 0.0;
        last_z_ = 0.0;
        has_last_z_ = false;
    }

    /**
     * @brief Update detector state.
     * @param actual_z Drone altitude (m)
     * @param actual_vz Drone vertical velocity in ENU (m/s)
     * @param commanded_vz Commanded vertical velocity in ENU (m/s)
     * @param dt Time delta since last update (s)
     * @param fast_confirm_trigger Optional fast confirmation trigger (impact jerk or PX4 landed flag)
     */
    TouchdownResult update(
        double actual_z,
        double actual_vz,
        double commanded_vz,
        double dt,
        bool fast_confirm_trigger = false
    ) {
        TouchdownResult res{};
        res.actual_vz = actual_vz;
        res.commanded_vz = commanded_vz;

        if (latched_touchdown_) {
            res.touchdown_confirmed = true;
            res.diagnostic_reason = "Touchdown already latched";
            return res;
        }

        // Calculate dz/dt
        double dz_dt = 0.0;
        if (has_last_z_ && dt > 1e-4) {
            dz_dt = (actual_z - last_z_) / dt;
        }
        last_z_ = actual_z;
        has_last_z_ = true;

        // Kinematic Stoppage Criteria:
        // 1. Drone is commanded downwards (v_cmd <= -0.1 m/s)
        // 2. Drone has reached ground altitude (z <= 0.25 m)
        // 3. Drone vertical velocity has flattened (|v_z| <= 0.08 m/s)
        // 4. Rate of altitude change is near zero (|dz/dt| <= 0.05 m/s)
        bool cmd_down = (commanded_vz <= cfg_.descent_cmd_threshold_mps);
        bool near_ground = (actual_z <= cfg_.altitude_ceiling_m);
        bool vz_stopped = (std::abs(actual_vz) <= cfg_.stoppage_vz_max_mps);
        bool dz_stopped = (std::abs(dz_dt) <= cfg_.stoppage_dz_dt_max_mps);

        bool kinematic_stopped = cmd_down && near_ground && vz_stopped && dz_stopped;
        res.kinematic_stopped = kinematic_stopped;

        if (kinematic_stopped) {
            confirm_timer_s_ += dt;
            res.confirm_duration_s = confirm_timer_s_;
            double required_confirm = fast_confirm_trigger ? cfg_.fast_confirmation_duration_s : cfg_.confirmation_duration_s;
            if (confirm_timer_s_ >= required_confirm) {
                latched_touchdown_ = true;
                res.touchdown_confirmed = true;
                res.diagnostic_reason = "Touchdown confirmed via kinematic stoppage & debounce";
                return res;
            }
            res.diagnostic_reason = "Kinematic stopped; accumulating confirmation timer";
        } else {
            confirm_timer_s_ = 0.0;
            res.diagnostic_reason = "Descending or in flight";
        }

        res.touchdown_confirmed = false;
        return res;
    }

    [[nodiscard]] bool is_latched() const {
        return latched_touchdown_;
    }

private:
    Config cfg_;
    bool latched_touchdown_{false};
    double confirm_timer_s_{0.0};
    double last_z_{0.0};
    bool has_last_z_{false};
};

} // namespace vdt::landing
