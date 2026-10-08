/**
 * @file Types.hpp
 * @brief Common Types and Data Structures for Precision Landing Pipeline (C++20)
 * @author Duy (Algorithm Lead) - VDT Quadrotor H-Pad Precision Landing
 */

#pragma once

#include <array>
#include <cmath>
#include <cstdint>
#include <string>
#include <string_view>
#include <vector>

namespace vdt::landing {

struct Vector3 {
    double x{0.0};
    double y{0.0};
    double z{0.0};

    [[nodiscard]] double norm() const {
        return std::sqrt(x * x + y * y + z * z);
    }
    [[nodiscard]] double norm_xy() const {
        return std::sqrt(x * x + y * y);
    }
};

inline Vector3 operator+(const Vector3& a, const Vector3& b) {
    return {a.x + b.x, a.y + b.y, a.z + b.z};
}
inline Vector3 operator-(const Vector3& a, const Vector3& b) {
    return {a.x - b.x, a.y - b.y, a.z - b.z};
}
inline Vector3 operator*(const Vector3& a, double s) {
    return {a.x * s, a.y * s, a.z * s};
}
inline Vector3 operator*(double s, const Vector3& a) {
    return a * s;
}

struct Quaternion {
    double w{1.0};
    double x{0.0};
    double y{0.0};
    double z{0.0};
};

struct OdometryState {
    double timestamp_s{0.0};
    Vector3 position{};       // World ENU frame (m)
    Vector3 velocity{};       // World ENU frame (m/s)
    Quaternion orientation{}; // World ENU frame
    double yaw{0.0};          // Heading angle (rad)
    std::array<double, 36> pose_covariance{};
    std::array<double, 36> twist_covariance{};
};

enum class MissionPhase {
    IDLE,
    SEARCH,
    FOLLOW,
    APPROACH,
    GLIDE_SLOPE,
    LAND,
    TOUCHDOWN
};

inline std::string_view to_string(MissionPhase phase) {
    switch (phase) {
        case MissionPhase::IDLE: return "IDLE";
        case MissionPhase::SEARCH: return "SEARCH";
        case MissionPhase::FOLLOW: return "FOLLOW";
        case MissionPhase::APPROACH: return "APPROACH";
        case MissionPhase::GLIDE_SLOPE: return "GLIDE_SLOPE";
        case MissionPhase::LAND: return "LAND";
        case MissionPhase::TOUCHDOWN: return "TOUCHDOWN";
    }
    return "UNKNOWN";
}

enum class TrackingMode {
    IDLE,
    TRACKING,
    DEAD_RECKONING,
    PREDICTING_DEGRADED,
    EXPIRED
};

inline std::string_view to_string(TrackingMode mode) {
    switch (mode) {
        case TrackingMode::IDLE: return "IDLE";
        case TrackingMode::TRACKING: return "TRACKING";
        case TrackingMode::DEAD_RECKONING: return "DEAD_RECKONING";
        case TrackingMode::PREDICTING_DEGRADED: return "PREDICTING_DEGRADED";
        case TrackingMode::EXPIRED: return "EXPIRED";
    }
    return "UNKNOWN";
}

struct ArUcoDetection {
    int id{-1};
    bool detected{false};
    double timestamp_s{0.0};
    Vector3 pos_camera{};     // Camera optical frame: X right, Y down, Z forward (m)
    Vector3 pos_world{};      // World ENU frame: X east, Y north, Z up (m)
    Quaternion rot_camera{};
    double pixel_u{0.0};      // Center pixel X
    double pixel_v{0.0};      // Center pixel Y
    double bbox_width{0.0};
    double bbox_height{0.0};
};

struct CylinderObstacle {
    Vector3 center{};         // (x, y, z) center
    double radius{0.5};       // Radius in meters
    double height{3.0};       // Height in meters
};

struct CovarianceStatusResult {
    bool safe_to_land{false};
    double r_uncertainty_2sigma{0.0}; // 2-sigma relative uncertainty radius (m)
    double lambda_max_2d{0.0};
    double lambda_max_3d{0.0};
    double rswitch_adaptive{4.5};     // Adaptive glide slope switching radius (m)
    double sliding_weight{1.0};       // Surface sliding weight w_s in [0, 1]
    double relative_dist_xy{0.0};
    double relative_dist_z{0.0};
    double measurement_age_s{0.0};
    std::string reject_reason{};
};

enum class SMCSubPhase {
    GLIDE_SLOPE,
    FINAL_DESCENT,
    FINAL_GATE_WAIT
};

struct SMCResult {
    Vector3 velocity_cmd{};    // [vx, vy, vz] in ENU world frame (m/s)
    double yaw_rate_cmd{0.0};  // rad/s
    SMCSubPhase sub_phase{SMCSubPhase::GLIDE_SLOPE};
    std::array<double, 3> sliding_surface{0.0, 0.0, 0.0}; // S = [S1, S2, S3]
    double r_xy{0.0};          // Horizontal distance to target (m)
    double r_z{0.0};           // Vertical relative height (m)
    bool in_deadband{false};
};

struct TouchdownResult {
    bool touchdown_confirmed{false}; // Latched True upon ground contact
    bool kinematic_stopped{false};
    bool optical_near{false};
    bool impact_detected{false};
    double confirm_duration_s{0.0};
    double actual_vz{0.0};
    double commanded_vz{0.0};
    double optical_z{0.0};
    std::string diagnostic_reason{};
};

} // namespace vdt::landing
