#pragma once

#include <esp_log.h>
#include <esp_err.h>
#include <expected>
#include <string>
#include "../imu/imu.h"
#include "../gnss/gnss.h"

extern "C" {
#include <driver/twai.h>
}

namespace logging {

/**
 * @brief Dual-output sensor data logger that handles both console and ROS publishing
 * 
 * Provides a unified interface for logging sensor data to console (for debugging)
 * and publishing to ROS topics (for integration with ROS systems). Supports 
 * configurable output modes and graceful degradation when ROS is unavailable.
 */
class SensorLogger {
public:
    /**
     * @brief Output mode configuration for the logger
     */
    enum class OutputMode {
        CONSOLE_ONLY,  ///< Log only to console via ESP_LOGI
        ROS_ONLY,      ///< Publish only to ROS topics
        DUAL_OUTPUT    ///< Both console logging and ROS publishing (default)
    };

    /**
     * @brief Construct logger with specified output mode
     * @param mode Output mode for logging operations
     */
    explicit SensorLogger(OutputMode mode = OutputMode::DUAL_OUTPUT);

    /**
     * @brief Destructor - cleanup ROS resources if initialized
     */
    ~SensorLogger();

    /**
     * @brief Log IMU sensor data to configured outputs
     * @param data IMU data containing accelerometer, gyroscope, and temperature
     */
    void log_imu_data(const IMUData& data);

    /**
     * @brief Log GNSS sensor data to configured outputs  
     * @param data GNSS data containing position, fix info, and satellite data
     */
    void log_gnss_data(const gnss::GNSSData& data);

    /**
     * @brief Change the output mode at runtime
     * @param mode New output mode to use
     */
    void set_output_mode(OutputMode mode);

    /**
     * @brief Get current output mode
     * @return Current OutputMode setting
     */
    OutputMode get_output_mode() const { return output_mode_; }

    /**
     * @brief Check if ROS publishing is available
     * @return True if ROS is initialized and ready for publishing
     */
    bool is_ros_available() const { return ros_initialized_; }

private:
    OutputMode output_mode_;
    bool ros_initialized_;
    static constexpr const char* TAG = "SensorLogger";
    
    // CAN bus components
    bool can_initialized_;
    
    // CAN message IDs
    static constexpr uint32_t CAN_ID_IMU_ACCEL = 0x100;
    static constexpr uint32_t CAN_ID_IMU_GYRO = 0x101;
    static constexpr uint32_t CAN_ID_IMU_TEMP = 0x102;
    static constexpr uint32_t CAN_ID_GNSS_POSITION = 0x200;
    static constexpr uint32_t CAN_ID_GNSS_STATUS = 0x201;
    static constexpr uint32_t CAN_ID_HEARTBEAT = 0x7FF;

    /**
     * @brief Initialize CAN communication system
     * @return std::expected with success or error code
     */
    std::expected<void, esp_err_t> init_ros();

    /**
     * @brief Log message to console using ESP-IDF logging
     * @param message Formatted message string to log
     */
    void log_to_console(const std::string& message);

    /**
     * @brief Publish IMU data to CAN bus
     * @param data IMU data to publish
     */
    void publish_ros_imu(const IMUData& data);

    /**
     * @brief Publish GNSS data to CAN bus
     * @param data GNSS data to publish
     */
    void publish_ros_gnss(const gnss::GNSSData& data);

    /**
     * @brief Send CAN message with error handling
     * @param message CAN message to send
     * @return std::expected with success or error code
     */
    std::expected<void, esp_err_t> send_can_message(const twai_message_t& message);

    /**
     * @brief Send periodic heartbeat message
     * @return std::expected with success or error code
     */
    std::expected<void, esp_err_t> send_heartbeat();

    /**
     * @brief Format IMU data for console output
     * @param data IMU data to format
     * @return Formatted string for console logging
     */
    std::string format_imu_console(const IMUData& data);

    /**
     * @brief Format GNSS data for console output
     * @param data GNSS data to format  
     * @return Formatted string for console logging
     */
    std::string format_gnss_console(const gnss::GNSSData& data);
};

} // namespace logging