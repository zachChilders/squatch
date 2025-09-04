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

// ANSI color codes
#define COLOR_RESET   "\033[0m"
#define COLOR_KEY     "\033[36m"  // Cyan for keys  
#define COLOR_STRING  "\033[32m"  // Green for string values
#define COLOR_NUMBER  "\033[33m"  // Yellow for numbers
#define COLOR_BOOL    "\033[35m"  // Magenta for booleans

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
    
    /// @brief Colorize JSON string for terminal display
    /// @param json_str JSON string to colorize
    /// @return Colorized string
    std::string colorize_json(const std::string& json_str);
    
    /// @brief Helper function for string replacement
    /// @param str String to modify
    /// @param from Substring to find
    /// @param to Replacement substring
    void replace_all(std::string& str, const std::string& from, const std::string& to);
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

inline void StructuredLogger::replace_all(std::string& str, const std::string& from, const std::string& to) {
    size_t pos = 0;
    while ((pos = str.find(from, pos)) != std::string::npos) {
        str.replace(pos, from.length(), to);
        pos += to.length();
    }
}

inline std::string StructuredLogger::colorize_json(const std::string& json_str) {
    std::string result = json_str;
    
    // Color common JSON keys we know about
    replace_all(result, "\"timestamp\":", COLOR_KEY "\"timestamp\":" COLOR_RESET);
    replace_all(result, "\"valid\":", COLOR_KEY "\"valid\":" COLOR_RESET);
    replace_all(result, "\"accel\":", COLOR_KEY "\"accel\":" COLOR_RESET);
    replace_all(result, "\"gyro\":", COLOR_KEY "\"gyro\":" COLOR_RESET);
    replace_all(result, "\"gnss\":", COLOR_KEY "\"gnss\":" COLOR_RESET);
    replace_all(result, "\"imu\":", COLOR_KEY "\"imu\":" COLOR_RESET);
    replace_all(result, "\"x\":", COLOR_KEY "\"x\":" COLOR_RESET);
    replace_all(result, "\"y\":", COLOR_KEY "\"y\":" COLOR_RESET);
    replace_all(result, "\"z\":", COLOR_KEY "\"z\":" COLOR_RESET);
    replace_all(result, "\"temperature\":", COLOR_KEY "\"temperature\":" COLOR_RESET);
    replace_all(result, "\"latitude\":", COLOR_KEY "\"latitude\":" COLOR_RESET);
    replace_all(result, "\"longitude\":", COLOR_KEY "\"longitude\":" COLOR_RESET);
    replace_all(result, "\"altitude\":", COLOR_KEY "\"altitude\":" COLOR_RESET);
    replace_all(result, "\"satellites\":", COLOR_KEY "\"satellites\":" COLOR_RESET);
    replace_all(result, "\"hdop\":", COLOR_KEY "\"hdop\":" COLOR_RESET);
    replace_all(result, "\"fix_valid\":", COLOR_KEY "\"fix_valid\":" COLOR_RESET);
    replace_all(result, "\"time\":", COLOR_KEY "\"time\":" COLOR_RESET);
    replace_all(result, "\"date\":", COLOR_KEY "\"date\":" COLOR_RESET);
    
    // Color boolean values
    replace_all(result, ": true", ": " COLOR_BOOL "true" COLOR_RESET);
    replace_all(result, ": false", ": " COLOR_BOOL "false" COLOR_RESET);
    
    return result;
}

inline std::expected<void, esp_err_t> StructuredLogger::log(const SensorPacket& packet) {
    nlohmann::json j = packet;
    std::string json_output = j.dump();  // Single-line JSON
    
    // Disable colors for machine consumption
    bool use_colors = false;
    if (use_colors) {
        json_output = colorize_json(json_output);
    }
    
    std::cout << json_output << std::endl;
    std::cout.flush();
    return {};
}

} // namespace logging