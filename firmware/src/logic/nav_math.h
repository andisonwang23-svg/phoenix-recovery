// ============================================================================
// PHOENIX RECOVERY — Navigation math (pure C++, zero hardware deps).
// All functions are constexpr/inline where possible. Host-testable.
// ============================================================================
#pragma once

#include <cmath>
#include <cstdint>

namespace logic {

// ---------------------------------------------------------------------------
// Constants
// ---------------------------------------------------------------------------
constexpr double DEG_TO_RAD_D = M_PI / 180.0;
constexpr double RAD_TO_DEG_D = 180.0 / M_PI;
constexpr double EARTH_RADIUS_M = 6371000.0;

// ---------------------------------------------------------------------------
// Angle normalization to [-180, +180] degrees
// ---------------------------------------------------------------------------
inline double normalizeAngle(double angle_deg) {
    while (angle_deg > 180.0) angle_deg -= 360.0;
    while (angle_deg <= -180.0) angle_deg += 360.0;
    return angle_deg;
}

// ---------------------------------------------------------------------------
// Distance between two lat/lon points (Haversine formula)
// Returns distance in meters
// ---------------------------------------------------------------------------
inline double distanceMeters(double lat1_deg, double lon1_deg,
                             double lat2_deg, double lon2_deg) {
    double lat1 = lat1_deg * DEG_TO_RAD_D;
    double lon1 = lon1_deg * DEG_TO_RAD_D;
    double lat2 = lat2_deg * DEG_TO_RAD_D;
    double lon2 = lon2_deg * DEG_TO_RAD_D;

    double dlat = lat2 - lat1;
    double dlon = lon2 - lon1;

    double a = std::sin(dlat * 0.5) * std::sin(dlat * 0.5) +
               std::cos(lat1) * std::cos(lat2) *
               std::sin(dlon * 0.5) * std::sin(dlon * 0.5);

    double c = 2.0 * std::atan2(std::sqrt(a), std::sqrt(1.0 - a));
    return EARTH_RADIUS_M * c;
}

// ---------------------------------------------------------------------------
// Bearing from point 1 to point 2 (degrees, 0..360)
// 0 = North, 90 = East, 180 = South, 270 = West
// ---------------------------------------------------------------------------
inline double bearingDegrees(double lat1_deg, double lon1_deg,
                             double lat2_deg, double lon2_deg) {
    double lat1 = lat1_deg * DEG_TO_RAD_D;
    double lon1 = lon1_deg * DEG_TO_RAD_D;
    double lat2 = lat2_deg * DEG_TO_RAD_D;
    double lon2 = lon2_deg * DEG_TO_RAD_D;

    double dlon = lon2 - lon1;

    double y = std::sin(dlon) * std::cos(lat2);
    double x = std::cos(lat1) * std::sin(lat2) -
               std::sin(lat1) * std::cos(lat2) * std::cos(dlon);

    double brng = std::atan2(y, x) * RAD_TO_DEG_D;
    if (brng < 0.0) brng += 360.0;
    return brng;
}

// ---------------------------------------------------------------------------
// Heading error: target_bearing - current_course, normalized to [-180, +180]
// Positive = target is to the right, Negative = target is to the left
// ---------------------------------------------------------------------------
inline double headingErrorDegrees(double current_course_deg, double target_bearing_deg) {
    return normalizeAngle(target_bearing_deg - current_course_deg);
}

// ---------------------------------------------------------------------------
// Cross-track error (meters perpendicular to desired track)
// Approximation: sin(heading_error) * distance
// ---------------------------------------------------------------------------
inline double crossTrackErrorMeters(double distance_m, double heading_error_deg) {
    return distance_m * std::sin(heading_error_deg * DEG_TO_RAD_D);
}

// ---------------------------------------------------------------------------
// Along-track error (meters parallel to desired track)
// Approximation: cos(heading_error) * distance
// ---------------------------------------------------------------------------
inline double alongTrackErrorMeters(double distance_m, double heading_error_deg) {
    return distance_m * std::cos(heading_error_deg * DEG_TO_RAD_D);
}

} // namespace logic