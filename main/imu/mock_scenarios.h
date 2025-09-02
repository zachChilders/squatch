#pragma once

#ifdef CONFIG_IMU_MOCK

#include "mock.h"

/**
 * @brief Predefined test scenarios for mock MPU6050
 */
namespace MockScenarios {

/**
 * @brief Device at rest with normal gravity (1g on Z-axis)
 */
inline void stationary() {
    mock_set_accel_g(0.0f, 0.0f, 1.0f);  // Gravity on Z
    mock_set_gyro_dps(0.0f, 0.0f, 0.0f); // No rotation
    mock_set_temperature_c(25.0f);        // Room temperature
}

/**
 * @brief Device tilted 45 degrees
 */
inline void tilted_45deg() {
    mock_set_accel_g(0.707f, 0.0f, 0.707f);  // 45° tilt
    mock_set_gyro_dps(0.0f, 0.0f, 0.0f);     // No rotation
    mock_set_temperature_c(25.0f);
}

/**
 * @brief Device rotating around Z-axis
 */
inline void rotating() {
    mock_set_accel_g(0.0f, 0.0f, 1.0f);   // Gravity on Z
    mock_set_gyro_dps(0.0f, 0.0f, 10.0f); // 10°/s rotation
    mock_set_temperature_c(25.0f);
}

/**
 * @brief High temperature scenario  
 */
inline void hot() {
    mock_set_accel_g(0.0f, 0.0f, 1.0f);
    mock_set_gyro_dps(0.0f, 0.0f, 0.0f);
    mock_set_temperature_c(85.0f);         // Hot temperature
}

/**
 * @brief Zero gravity simulation
 */
inline void zero_g() {
    mock_set_accel_g(0.0f, 0.0f, 0.0f);   // No gravity
    mock_set_gyro_dps(0.0f, 0.0f, 0.0f);
    mock_set_temperature_c(25.0f);
}

/**
 * @brief Vibration/movement scenario
 */
inline void vibrating() {
    mock_set_accel_g(0.1f, -0.05f, 1.02f); // Small vibrations around 1g
    mock_set_gyro_dps(2.5f, -1.2f, 0.8f);  // Small rotations
    mock_set_temperature_c(28.0f);
}

} // namespace MockScenarios

#endif // CONFIG_IMU_MOCK