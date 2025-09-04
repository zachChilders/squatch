#pragma once

#include <expected>
#include <iostream>
#include <string>
#include <unistd.h>
#include "esp_log.h"
#include "esp_err.h"
#include "models.h"
#include "../imu/imu.h"
#include "../gnss/gnss.h"


namespace logging {

/// @brief Structured logger for sensor data transmission
class StructuredLogger {
public:
    /// @brief Initialize the structured logger
    /// @return ESP_OK on success
    std::expected<void, esp_err_t> init();
    
    /// @brief Log sensor packet as JSON to stdout
    /// @param packet Sensor data packet to log
    /// @return ESP_OK on success
    std::expected<void, esp_err_t> log(const SensorPacket& packet);

private:
    static constexpr const char* TAG = "StructuredLogger";
};

// Helper function implementations
inline IMUModel create_imu_model(const IMUData& imu_data) {
    AccelModel accel(imu_data.accel.x, imu_data.accel.y, imu_data.accel.z);
    GyroModel gyro(imu_data.gyro.x, imu_data.gyro.y, imu_data.gyro.z);
    return IMUModel(accel, gyro, imu_data.temperature);
}

inline GNSSModel create_gnss_model(const gnss::GNSSData& gnss_data) {
    return GNSSModel(gnss_data.latitude, gnss_data.longitude, gnss_data.altitude,
                     gnss_data.satellites, gnss_data.hdop, gnss_data.fix_valid,
                     gnss_data.timestamp, gnss_data.date);
}

// Implementation

inline std::expected<void, esp_err_t> StructuredLogger::init() {
    ESP_LOGI(TAG, "Structured logger initialized");
    return {};
}


inline std::expected<void, esp_err_t> StructuredLogger::log(const SensorPacket& packet) {
    nlohmann::json j = packet;
    std::string json_output = j.dump();  // Single-line JSON
    
    std::cout << json_output << std::endl;
    std::cout.flush();
    return {};
}

} // namespace logging