#pragma once

#ifdef CONFIG_GNSS_MOCK

#include "../lib/uart_mock.h"

/**
 * @brief Predefined GNSS test scenarios for mock GPS data
 */
namespace GNSSMockScenarios {

/**
 * @brief Good GPS fix with strong signal (San Francisco)
 */
inline void stationary_fix() {
    g_mock_gnss.latitude = 37.7749;        // San Francisco
    g_mock_gnss.longitude = -122.4194;
    g_mock_gnss.altitude = 15.0f;
    g_mock_gnss.satellites = 10;
    g_mock_gnss.hdop = 0.8f;               // Excellent precision
    g_mock_gnss.fix_valid = true;
    g_mock_gnss.fix_quality = 1;           // GPS fix
    g_mock_gnss.speed_knots = 0.0f;
    g_mock_gnss.course_degrees = 0.0f;
    g_mock_gnss.signal_available = true;
}

/**
 * @brief Poor GPS signal (few satellites, poor precision)
 */
inline void weak_signal() {
    g_mock_gnss.latitude = 37.7749;
    g_mock_gnss.longitude = -122.4194;
    g_mock_gnss.altitude = 15.0f;
    g_mock_gnss.satellites = 4;            // Minimum for fix
    g_mock_gnss.hdop = 4.5f;              // Poor precision
    g_mock_gnss.fix_valid = true;
    g_mock_gnss.fix_quality = 1;
    g_mock_gnss.speed_knots = 0.0f;
    g_mock_gnss.course_degrees = 0.0f;
    g_mock_gnss.signal_available = true;
}

/**
 * @brief No GPS signal (indoor/blocked)
 */
inline void no_signal() {
    g_mock_gnss.satellites = 0;
    g_mock_gnss.hdop = 99.99f;
    g_mock_gnss.fix_valid = false;
    g_mock_gnss.fix_quality = 0;           // No fix
    g_mock_gnss.speed_knots = 0.0f;
    g_mock_gnss.course_degrees = 0.0f;
    g_mock_gnss.signal_available = false;
}

/**
 * @brief High precision DGPS fix
 */
inline void dgps_fix() {
    g_mock_gnss.latitude = 37.7749;
    g_mock_gnss.longitude = -122.4194;
    g_mock_gnss.altitude = 15.0f;
    g_mock_gnss.satellites = 12;
    g_mock_gnss.hdop = 0.5f;              // Excellent precision
    g_mock_gnss.fix_valid = true;
    g_mock_gnss.fix_quality = 2;          // DGPS fix
    g_mock_gnss.speed_knots = 0.0f;
    g_mock_gnss.course_degrees = 0.0f;
    g_mock_gnss.signal_available = true;
}

/**
 * @brief Moving vehicle scenario (driving north at 35 mph)
 */
inline void moving_north() {
    g_mock_gnss.latitude = 37.7749;
    g_mock_gnss.longitude = -122.4194;
    g_mock_gnss.altitude = 15.0f;
    g_mock_gnss.satellites = 9;
    g_mock_gnss.hdop = 1.1f;
    g_mock_gnss.fix_valid = true;
    g_mock_gnss.fix_quality = 1;
    g_mock_gnss.speed_knots = 30.4f;       // ~35 mph in knots
    g_mock_gnss.course_degrees = 0.0f;     // Due north
    g_mock_gnss.signal_available = true;
}

/**
 * @brief Walking speed scenario (3 mph east)
 */
inline void walking_east() {
    g_mock_gnss.latitude = 37.7749;
    g_mock_gnss.longitude = -122.4194;
    g_mock_gnss.altitude = 15.0f;
    g_mock_gnss.satellites = 8;
    g_mock_gnss.hdop = 1.5f;
    g_mock_gnss.fix_valid = true;
    g_mock_gnss.fix_quality = 1;
    g_mock_gnss.speed_knots = 2.6f;        // ~3 mph in knots
    g_mock_gnss.course_degrees = 90.0f;    // Due east
    g_mock_gnss.signal_available = true;
}

/**
 * @brief High altitude scenario (mountain/aircraft)
 */
inline void high_altitude() {
    g_mock_gnss.latitude = 39.7392;        // Denver area
    g_mock_gnss.longitude = -104.9903;
    g_mock_gnss.altitude = 3500.0f;        // 3500m above sea level
    g_mock_gnss.satellites = 11;
    g_mock_gnss.hdop = 0.9f;
    g_mock_gnss.fix_valid = true;
    g_mock_gnss.fix_quality = 1;
    g_mock_gnss.speed_knots = 0.0f;
    g_mock_gnss.course_degrees = 0.0f;
    g_mock_gnss.signal_available = true;
}

/**
 * @brief Acquiring signal scenario (cold start simulation)
 */
inline void acquiring_signal() {
    g_mock_gnss.satellites = 2;            // Not enough for fix yet
    g_mock_gnss.hdop = 25.0f;             // Poor precision during acquisition
    g_mock_gnss.fix_valid = false;
    g_mock_gnss.fix_quality = 0;
    g_mock_gnss.speed_knots = 0.0f;
    g_mock_gnss.course_degrees = 0.0f;
    g_mock_gnss.signal_available = true;   // Signal present but not ready
}

/**
 * @brief Set custom location
 */
inline void custom_location(double lat, double lon, float alt = 10.0f) {
    g_mock_gnss.latitude = lat;
    g_mock_gnss.longitude = lon;
    g_mock_gnss.altitude = alt;
    g_mock_gnss.satellites = 8;
    g_mock_gnss.hdop = 1.2f;
    g_mock_gnss.fix_valid = true;
    g_mock_gnss.fix_quality = 1;
    g_mock_gnss.speed_knots = 0.0f;
    g_mock_gnss.course_degrees = 0.0f;
    g_mock_gnss.signal_available = true;
}

/**
 * @brief Set movement parameters
 */
inline void set_movement(float speed_knots, float course_degrees) {
    g_mock_gnss.speed_knots = speed_knots;
    g_mock_gnss.course_degrees = course_degrees;
}

/**
 * @brief Set time (UTC format: HHMMSS and DDMMYY)
 */
inline void set_time(uint32_t utc_time, uint32_t utc_date) {
    g_mock_gnss.utc_time = utc_time;
    g_mock_gnss.utc_date = utc_date;
}

} // namespace GNSSMockScenarios

#endif // CONFIG_GNSS_MOCK