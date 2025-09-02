#pragma once

extern "C" {
#include "esp_err.h"
#include "esp_log.h"
}

#include "registers.h"
#include <array>
#include <expected>

#ifdef CONFIG_IMU_MOCK

/**
 * @brief Mock MPU6050 sensor data for testing/simulation
 */
struct MockSensorData {
    // Raw sensor values (same format as MPU6050)
    int16_t accel_x = 0;        // 0g on X axis
    int16_t accel_y = 0;        // 0g on Y axis  
    int16_t accel_z = 16384;    // 1g on Z axis (gravity)
    int16_t temp_raw = -521;    // 25°C: (25 - 36.53) * 340 = -3920
    int16_t gyro_x = 0;         // No rotation
    int16_t gyro_y = 0;
    int16_t gyro_z = 0;
    
    // Device state
    bool sleep_mode = true;
    uint8_t who_am_i = 0x68;
};

// Global mock data - can be modified for testing scenarios
extern MockSensorData g_mock_sensor;

/**
 * @brief Set mock sensor values from physical units
 */
inline void mock_set_accel_g(float x, float y, float z) {
    g_mock_sensor.accel_x = static_cast<int16_t>(x * 16384.0f);
    g_mock_sensor.accel_y = static_cast<int16_t>(y * 16384.0f);
    g_mock_sensor.accel_z = static_cast<int16_t>(z * 16384.0f);
}

inline void mock_set_gyro_dps(float x, float y, float z) {
    g_mock_sensor.gyro_x = static_cast<int16_t>(x * 131.0f);
    g_mock_sensor.gyro_y = static_cast<int16_t>(y * 131.0f);
    g_mock_sensor.gyro_z = static_cast<int16_t>(z * 131.0f);
}

inline void mock_set_temperature_c(float temp_c) {
    g_mock_sensor.temp_raw = static_cast<int16_t>((temp_c - 36.53f) * 340.0f);
}

/**
 * @brief Mock register read function - replaces real I2C reads
 */
template<size_t N>
std::expected<std::array<uint8_t, N>, esp_err_t> register_read(uint8_t reg_addr) {
    std::array<uint8_t, N> result{};
    
    for (size_t i = 0; i < N; ++i) {
        uint8_t current_reg = reg_addr + i;
        
        switch (current_reg) {
        case MPU6050_WHO_AM_I:
            result[i] = g_mock_sensor.who_am_i;
            break;
            
        case MPU6050_PWR_MGMT_1:
            result[i] = g_mock_sensor.sleep_mode ? 0x40 : 0x00;
            break;
            
        // Accelerometer data (14-byte bulk read starting from 0x3B)
        case MPU6050_ACCEL_XOUT_H:
            result[i] = (g_mock_sensor.accel_x >> 8) & 0xFF;
            break;
        case MPU6050_ACCEL_XOUT_L:
            result[i] = g_mock_sensor.accel_x & 0xFF;
            break;
        case MPU6050_ACCEL_YOUT_H:
            result[i] = (g_mock_sensor.accel_y >> 8) & 0xFF;
            break;
        case MPU6050_ACCEL_YOUT_L:
            result[i] = g_mock_sensor.accel_y & 0xFF;
            break;
        case MPU6050_ACCEL_ZOUT_H:
            result[i] = (g_mock_sensor.accel_z >> 8) & 0xFF;
            break;
        case MPU6050_ACCEL_ZOUT_L:
            result[i] = g_mock_sensor.accel_z & 0xFF;
            break;
            
        // Temperature data
        case MPU6050_TEMP_OUT_H:
            result[i] = (g_mock_sensor.temp_raw >> 8) & 0xFF;
            break;
        case MPU6050_TEMP_OUT_L:
            result[i] = g_mock_sensor.temp_raw & 0xFF;
            break;
            
        // Gyroscope data  
        case MPU6050_GYRO_XOUT_H:
            result[i] = (g_mock_sensor.gyro_x >> 8) & 0xFF;
            break;
        case MPU6050_GYRO_XOUT_L:
            result[i] = g_mock_sensor.gyro_x & 0xFF;
            break;
        case MPU6050_GYRO_YOUT_H:
            result[i] = (g_mock_sensor.gyro_y >> 8) & 0xFF;
            break;
        case MPU6050_GYRO_YOUT_L:
            result[i] = g_mock_sensor.gyro_y & 0xFF;
            break;
        case MPU6050_GYRO_ZOUT_H:
            result[i] = (g_mock_sensor.gyro_z >> 8) & 0xFF;
            break;
        case MPU6050_GYRO_ZOUT_L:
            result[i] = g_mock_sensor.gyro_z & 0xFF;
            break;
            
        default:
            result[i] = 0x00;
            break;
        }
    }
    
    return result;
}

/**
 * @brief Mock register write function - replaces real I2C writes  
 */
inline esp_err_t register_write_byte(uint8_t reg_addr, uint8_t data) {
    switch (reg_addr) {
    case MPU6050_PWR_MGMT_1:
        g_mock_sensor.sleep_mode = (data & 0x40) != 0;
        break;
    default:
        // Ignore other writes
        break;
    }
    
    return ESP_OK;
}

/**
 * @brief Mock raw register read function
 */
inline esp_err_t register_read_raw(uint8_t reg_addr, uint8_t *data, size_t len) {
    for (size_t i = 0; i < len; ++i) {
        auto result = register_read<1>(reg_addr + i);
        if (!result) {
            return result.error();
        }
        data[i] = result.value()[0];
    }
    return ESP_OK;
}

// Mock I2C initialization (no-op in mock mode)
inline esp_err_t i2c_master_init(void) {
    return ESP_OK;
}

#endif // CONFIG_IMU_MOCK