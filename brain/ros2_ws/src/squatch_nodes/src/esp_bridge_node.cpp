#include "squatch_nodes/esp_bridge.hpp"
#include <chrono>
#include <iostream>
#include <sstream>
#include <cstring>
#include <cerrno>

using namespace std::chrono_literals;

namespace squatch_nodes {

ESPBridge::ESPBridge(const std::string& host, unsigned int port)
    : Node("esp_bridge"), socket_fd_(-1), host_(host), port_(port) {
    
    // Create publishers
    imu_publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("/squatch/imu", 10);
    gnss_publisher_ = this->create_publisher<sensor_msgs::msg::NavSatFix>("/squatch/gnss", 10);
    
    RCLCPP_INFO(this->get_logger(), "ESP Bridge starting on host: %s port: %d", 
                host_.c_str(), port_);
    
    // Start TCP communication thread
    tcp_thread_ = std::thread(&ESPBridge::tcp_read_loop, this);
}

ESPBridge::~ESPBridge() {
    should_stop_ = true;
    if (tcp_thread_.joinable()) {
        tcp_thread_.join();
    }
    if (socket_fd_ >= 0) {
        close(socket_fd_);
    }
}

void ESPBridge::tcp_read_loop() {
    std::string line_buffer;
    
    while (!should_stop_) {
        if (socket_fd_ < 0) {
            if (!connect_tcp()) {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                    "Failed to connect to %s:%d, retrying...", host_.c_str(), port_);
                std::this_thread::sleep_for(RECONNECT_DELAY);
                continue;
            }
        }
        
        try {
            char buffer[TCP_BUFFER_SIZE];
            
            ssize_t bytes_read = recv(socket_fd_, buffer, TCP_BUFFER_SIZE, 0);
                
            if (bytes_read <= 0) {
                RCLCPP_ERROR(this->get_logger(), "TCP read error: %s", strerror(errno));
                close(socket_fd_);
                socket_fd_ = -1;
                continue;
            }
            
            // Debug: show what chunks are being received
            RCLCPP_INFO(this->get_logger(), "Received %zd bytes: [%.*s]", 
                        bytes_read, (int)bytes_read, buffer);
            
            // Process received data character by character to find complete JSON lines
            for (ssize_t i = 0; i < bytes_read; ++i) {
                char c = buffer[i];
                if (c == '\n') {
                    if (!line_buffer.empty()) {
                        process_json_message(line_buffer);
                        line_buffer.clear();
                    }
                } else if (c != '\r') {
                    line_buffer += c;
                }
            }
            
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "TCP communication exception: %s", e.what());
            close(socket_fd_);
            socket_fd_ = -1;
            std::this_thread::sleep_for(RECONNECT_DELAY);
        }
    }
}

bool ESPBridge::connect_tcp() {
    try {
        socket_fd_ = socket(AF_INET, SOCK_STREAM, 0);
        if (socket_fd_ < 0) {
            RCLCPP_DEBUG(this->get_logger(), "Failed to create socket: %s", strerror(errno));
            return false;
        }
        
        struct addrinfo hints, *result;
        memset(&hints, 0, sizeof(hints));
        hints.ai_family = AF_INET;
        hints.ai_socktype = SOCK_STREAM;

        int ret = getaddrinfo(host_.c_str(), std::to_string(port_).c_str(), &hints, &result);
        if (ret != 0) {
            RCLCPP_DEBUG(this->get_logger(), "getaddrinfo failed: %s", gai_strerror(ret));
            close(socket_fd_);
            socket_fd_ = -1;
            return false;
        }
        
        if (connect(socket_fd_, result->ai_addr, result->ai_addrlen) < 0) {
            RCLCPP_DEBUG(this->get_logger(), "Failed to connect to %s:%d: %s", 
                        host_.c_str(), port_, strerror(errno));
            freeaddrinfo(result);
            close(socket_fd_);
            socket_fd_ = -1;
            return false;
        }
        
        freeaddrinfo(result);
        RCLCPP_INFO(this->get_logger(), "Connected to ESP device at %s:%d", host_.c_str(), port_);
        return true;
        
    } catch (const std::exception& e) {
        RCLCPP_DEBUG(this->get_logger(), "Failed to connect to %s:%d: %s", 
                    host_.c_str(), port_, e.what());
        if (socket_fd_ >= 0) {
            close(socket_fd_);
            socket_fd_ = -1;
        }
        return false;
    }
}

void ESPBridge::process_json_message(const std::string& json_line) {
    try {
        auto json_data = nlohmann::json::parse(json_line);
        
        // Extract timestamp
        uint64_t esp_timestamp_us = json_data.value("timestamp", 0UL);
        
        // Convert ESP timestamp to ROS2 timestamp
        auto now = this->get_clock()->now();
        
        // Process IMU data
        if (json_data.contains("imu")) {
            auto imu_msg = create_imu_message(json_data["imu"], esp_timestamp_us);
            imu_msg.header.stamp = now;
            imu_msg.header.frame_id = "esp_imu";
            imu_publisher_->publish(imu_msg);
        }
        
        // Process GNSS data
        if (json_data.contains("gnss")) {
            auto gnss_msg = create_gnss_message(json_data["gnss"], esp_timestamp_us);
            gnss_msg.header.stamp = now;
            gnss_msg.header.frame_id = "esp_gnss";
            gnss_publisher_->publish(gnss_msg);
        }
        
    } catch (const nlohmann::json::parse_error& e) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                            "JSON parse error: %s", e.what());
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Error processing ESP message: %s", e.what());
    }
}

sensor_msgs::msg::Imu ESPBridge::create_imu_message(const nlohmann::json& imu_json, 
                                                   uint64_t timestamp_us) {
    sensor_msgs::msg::Imu imu_msg;
    
    // Linear acceleration (m/s²) - ESP outputs in g, convert to m/s²
    const double G_TO_MS2 = 9.80665;
    imu_msg.linear_acceleration.x = imu_json["accel"]["x"].get<double>() * G_TO_MS2;
    imu_msg.linear_acceleration.y = imu_json["accel"]["y"].get<double>() * G_TO_MS2;
    imu_msg.linear_acceleration.z = imu_json["accel"]["z"].get<double>() * G_TO_MS2;
    
    // Angular velocity (rad/s) - ESP outputs in deg/s, convert to rad/s
    const double DEG_TO_RAD = M_PI / 180.0;
    imu_msg.angular_velocity.x = imu_json["gyro"]["x"].get<double>() * DEG_TO_RAD;
    imu_msg.angular_velocity.y = imu_json["gyro"]["y"].get<double>() * DEG_TO_RAD;
    imu_msg.angular_velocity.z = imu_json["gyro"]["z"].get<double>() * DEG_TO_RAD;
    
    // No orientation data from ESP (would need magnetometer + fusion)
    imu_msg.orientation_covariance[0] = -1.0; // Mark orientation as invalid
    
    // Set covariance matrices (conservative estimates for MPU6050)
    std::array<double, 9> linear_accel_cov = {0.01, 0, 0, 0, 0.01, 0, 0, 0, 0.01};
    std::array<double, 9> angular_vel_cov = {0.001, 0, 0, 0, 0.001, 0, 0, 0, 0.001};
    
    std::copy(linear_accel_cov.begin(), linear_accel_cov.end(), 
              imu_msg.linear_acceleration_covariance.begin());
    std::copy(angular_vel_cov.begin(), angular_vel_cov.end(),
              imu_msg.angular_velocity_covariance.begin());
    
    return imu_msg;
}

sensor_msgs::msg::NavSatFix ESPBridge::create_gnss_message(const nlohmann::json& gnss_json,
                                                         uint64_t timestamp_us) {
    sensor_msgs::msg::NavSatFix gnss_msg;
    
    // Position data
    gnss_msg.latitude = gnss_json.value("latitude", 0.0);
    gnss_msg.longitude = gnss_json.value("longitude", 0.0);
    gnss_msg.altitude = gnss_json.value("altitude", 0.0);
    
    // Fix status
    bool fix_valid = gnss_json.value("fix_valid", false);
    uint8_t sat_count = gnss_json.value("satellites", 0);
    
    if (fix_valid && sat_count >= 4) {
        gnss_msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
    } else if (sat_count > 0) {
        gnss_msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX;
    } else {
        gnss_msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX;
    }
    
    gnss_msg.status.service = sensor_msgs::msg::NavSatStatus::SERVICE_GPS;
    
    // Position covariance from HDOP
    double hdop = gnss_json.value("hdop", 99.0);
    double pos_variance = hdop * hdop; // Simple HDOP to variance mapping
    
    gnss_msg.position_covariance[0] = pos_variance;  // East
    gnss_msg.position_covariance[4] = pos_variance;  // North  
    gnss_msg.position_covariance[8] = pos_variance * 2; // Up (typically worse)
    gnss_msg.position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;
    
    return gnss_msg;
}

} // namespace squatch_nodes