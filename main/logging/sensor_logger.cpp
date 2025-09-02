#include "sensor_logger.h"
#include <sstream>
#include <iomanip>
#include <cstring>

namespace logging {

SensorLogger::SensorLogger(OutputMode mode) 
    : output_mode_(mode), ros_initialized_(false), can_initialized_(false) {
    if (output_mode_ == OutputMode::ROS_ONLY || output_mode_ == OutputMode::DUAL_OUTPUT) {
        auto result = init_ros();
        if (!result) {
            ESP_LOGW(TAG, "ROS initialization failed: %s", esp_err_to_name(result.error()));
            if (output_mode_ == OutputMode::ROS_ONLY) {
                ESP_LOGW(TAG, "Falling back to console-only mode");
                output_mode_ = OutputMode::CONSOLE_ONLY;
            }
        }
    }
}

SensorLogger::~SensorLogger() {
    // Cleanup ROS resources if needed
    if (ros_initialized_) {
        ESP_LOGI(TAG, "Cleaning up ROS resources");
        ros_initialized_ = false;
    }
}

void SensorLogger::log_imu_data(const IMUData& data) {
    if (output_mode_ == OutputMode::CONSOLE_ONLY || output_mode_ == OutputMode::DUAL_OUTPUT) {
        log_to_console(format_imu_console(data));
    }
    
    if ((output_mode_ == OutputMode::ROS_ONLY || output_mode_ == OutputMode::DUAL_OUTPUT) && ros_initialized_) {
        publish_ros_imu(data);
    }
}

void SensorLogger::log_gnss_data(const gnss::GNSSData& data) {
    if (output_mode_ == OutputMode::CONSOLE_ONLY || output_mode_ == OutputMode::DUAL_OUTPUT) {
        log_to_console(format_gnss_console(data));
    }
    
    if ((output_mode_ == OutputMode::ROS_ONLY || output_mode_ == OutputMode::DUAL_OUTPUT) && ros_initialized_) {
        publish_ros_gnss(data);
    }
}

void SensorLogger::set_output_mode(OutputMode mode) {
    output_mode_ = mode;
    
    if ((mode == OutputMode::ROS_ONLY || mode == OutputMode::DUAL_OUTPUT) && !ros_initialized_) {
        auto result = init_ros();
        if (!result) {
            ESP_LOGW(TAG, "Failed to initialize ROS for new output mode: %s", esp_err_to_name(result.error()));
        }
    }
}

std::expected<void, esp_err_t> SensorLogger::init_ros() {
    // CAN configuration
    twai_general_config_t g_config = {
        .mode = TWAI_MODE_NORMAL,
        .tx_io = GPIO_NUM_21,  // CAN TX pin
        .rx_io = GPIO_NUM_22,  // CAN RX pin
        .clkout_io = TWAI_IO_UNUSED,
        .bus_off_io = TWAI_IO_UNUSED,
        .tx_queue_len = 10,
        .rx_queue_len = 10,
        .alerts_enabled = TWAI_ALERT_NONE,
        .clkout_divider = 0,
        .intr_flags = ESP_INTR_FLAG_LEVEL1
    };

    // CAN timing configuration for 500kbps
    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
    
    // CAN filter configuration (accept all)
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    // Install CAN driver
    esp_err_t result = twai_driver_install(&g_config, &t_config, &f_config);
    if (result != ESP_OK) {
        ESP_LOGE(TAG, "Failed to install CAN driver: %s", esp_err_to_name(result));
        return std::unexpected(result);
    }

    // Start CAN driver
    result = twai_start();
    if (result != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start CAN driver: %s", esp_err_to_name(result));
        twai_driver_uninstall();
        return std::unexpected(result);
    }

    can_initialized_ = true;
    ros_initialized_ = true;  // Keep compatibility with existing interface
    ESP_LOGI(TAG, "CAN bus initialized successfully");
    return {};
}

void SensorLogger::log_to_console(const std::string& message) {
    ESP_LOGI(TAG, "%s", message.c_str());
}

void SensorLogger::publish_ros_imu(const IMUData& data) {
    if (!can_initialized_) return;

    // Send accelerometer data
    twai_message_t accel_msg = {
        .identifier = CAN_ID_IMU_ACCEL,
        .data_length_code = 8,
        .flags = TWAI_MSG_FLAG_NONE,
        .data = {0}
    };
    std::memcpy(accel_msg.data, &data.accel.x, sizeof(float));
    std::memcpy(accel_msg.data + 4, &data.accel.y, sizeof(float));
    send_can_message(accel_msg);

    // Send gyroscope data
    twai_message_t gyro_msg = {
        .identifier = CAN_ID_IMU_GYRO,
        .data_length_code = 8,
        .flags = TWAI_MSG_FLAG_NONE,
        .data = {0}
    };
    std::memcpy(gyro_msg.data, &data.gyro.x, sizeof(float));
    std::memcpy(gyro_msg.data + 4, &data.gyro.y, sizeof(float));
    send_can_message(gyro_msg);

    // Send temperature data
    twai_message_t temp_msg = {
        .identifier = CAN_ID_IMU_TEMP,
        .data_length_code = 4,
        .flags = TWAI_MSG_FLAG_NONE,
        .data = {0}
    };
    std::memcpy(temp_msg.data, &data.temperature, sizeof(float));
    send_can_message(temp_msg);
}

void SensorLogger::publish_ros_gnss(const gnss::GNSSData& data) {
    if (!can_initialized_) return;

    // Send position data
    twai_message_t pos_msg = {
        .identifier = CAN_ID_GNSS_POSITION,
        .data_length_code = 8,
        .flags = TWAI_MSG_FLAG_NONE,
        .data = {0}
    };
    std::memcpy(pos_msg.data, &data.latitude, sizeof(float));
    std::memcpy(pos_msg.data + 4, &data.longitude, sizeof(float));
    send_can_message(pos_msg);

    // Send status data
    twai_message_t status_msg = {
        .identifier = CAN_ID_GNSS_STATUS,
        .data_length_code = 8,
        .flags = TWAI_MSG_FLAG_NONE,
        .data = {0}
    };
    std::memcpy(status_msg.data, &data.altitude, sizeof(float));
    status_msg.data[4] = data.satellites;
    status_msg.data[5] = data.fix_valid ? 1 : 0;
    std::memcpy(status_msg.data + 6, &data.hdop, sizeof(uint16_t));
    send_can_message(status_msg);
}

std::string SensorLogger::format_imu_console(const IMUData& data) {
    std::ostringstream oss;
    oss << "Accel: X=" << std::fixed << std::setprecision(2) << data.accel.x << "g, "
        << "Y=" << data.accel.y << "g, "
        << "Z=" << data.accel.z << "g | "
        << "Gyro: X=" << data.gyro.x << "°/s, "
        << "Y=" << data.gyro.y << "°/s, "
        << "Z=" << data.gyro.z << "°/s | "
        << "Temp: " << data.temperature << "°C";
    return oss.str();
}

std::string SensorLogger::format_gnss_console(const gnss::GNSSData& data) {
    std::ostringstream oss;
    if (data.fix_valid) {
        oss << "GPS: Lat=" << std::fixed << std::setprecision(6) << data.latitude << "°, "
            << "Lon=" << data.longitude << "°, "
            << "Alt=" << std::setprecision(1) << data.altitude << "m, "
            << "Sats=" << static_cast<int>(data.satellites) << " | "
            << "Time=" << std::setfill('0') << std::setw(6) << data.timestamp << ", "
            << "Date=" << std::setw(6) << data.date << ", "
            << "HDOP=" << std::setprecision(2) << data.hdop;
    } else {
        oss << "GPS: No valid fix (Sats=" << static_cast<int>(data.satellites) << ")";
    }
    return oss.str();
}

std::expected<void, esp_err_t> SensorLogger::send_can_message(const twai_message_t& message) {
    esp_err_t result = twai_transmit(&message, pdMS_TO_TICKS(100));
    if (result != ESP_OK) {
        ESP_LOGW(TAG, "Failed to send CAN message ID 0x%X: %s", message.identifier, esp_err_to_name(result));
        return std::unexpected(result);
    }
    return {};
}

std::expected<void, esp_err_t> SensorLogger::send_heartbeat() {
    if (!can_initialized_) return std::unexpected(ESP_ERR_INVALID_STATE);
    
    twai_message_t heartbeat_msg = {
        .identifier = CAN_ID_HEARTBEAT,
        .data_length_code = 8,
        .flags = TWAI_MSG_FLAG_NONE,
        .data = {'S','Q','U','A','T','C','H',0x01}
    };
    
    return send_can_message(heartbeat_msg);
}

} // namespace logging