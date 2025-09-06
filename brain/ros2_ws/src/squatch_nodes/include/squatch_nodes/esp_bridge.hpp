#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <nlohmann/json.hpp>
#include <string>
#include <memory>
#include <thread>
#include <expected>
#include <system_error>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <unistd.h>

namespace squatch_nodes {

/**
 * @brief Error types for expected-based error handling
 */
enum class BridgeError {
    SocketCreate,
    AddressResolve, 
    ConnectionFailed,
    JsonParse,
    MessageProcess,
    TcpRead
};

/**
 * @brief ROS2 node that bridges ESP32 UART JSON output to ROS2 sensor topics
 * 
 * Reads JSON sensor data from ESP32 via TCP socket (QEMU serial redirect) and publishes
 * to standard ROS2 sensor message topics. Supports automatic reconnection
 * and robust error handling.
 */
class ESPBridge : public rclcpp::Node {
public:
    /**
     * @brief Constructor - initializes publishers and starts TCP communication
     * @param host Hostname or IP address of ESP TCP server (default: "localhost")
     * @param port TCP port number (default: 5555)
     */
    ESPBridge(const std::string& host = "localhost", 
              unsigned int port = 5555);
    
    /**
     * @brief Destructor - ensures clean shutdown of TCP communication
     */
    ~ESPBridge();

private:
    /**
     * @brief Main TCP reading loop - runs in separate thread
     */
    void tcp_read_loop();
    
    /**
     * @brief Attempts to open/reopen TCP connection
     * @return expected with success or BridgeError
     */
    std::expected<void, BridgeError> connect_tcp();
    
    /**
     * @brief Processes a single JSON line from ESP
     * @param json_line Raw JSON string from TCP socket
     * @return expected with success or BridgeError
     */
    std::expected<void, BridgeError> process_json_message(const std::string& json_line);
    
    /**
     * @brief Converts ESP IMU JSON to ROS2 sensor_msgs::Imu
     * @param imu_json JSON object containing IMU data
     * @param timestamp_us ESP timestamp in microseconds
     * @return Populated IMU message
     */
    sensor_msgs::msg::Imu create_imu_message(const nlohmann::json& imu_json, 
                                            uint64_t timestamp_us);
    
    /**
     * @brief Converts ESP GNSS JSON to ROS2 sensor_msgs::NavSatFix
     * @param gnss_json JSON object containing GNSS data
     * @param timestamp_us ESP timestamp in microseconds  
     * @return Populated NavSatFix message
     */
    sensor_msgs::msg::NavSatFix create_gnss_message(const nlohmann::json& gnss_json,
                                                   uint64_t timestamp_us);

    // TCP communication
    int socket_fd_;
    std::string host_;
    unsigned int port_;
    
    // Threading
    std::thread tcp_thread_;
    std::atomic<bool> should_stop_{false};
    
    // ROS2 publishers
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr gnss_publisher_;
    
    // Configuration
    static constexpr size_t TCP_BUFFER_SIZE = 1024;
    static constexpr auto RECONNECT_DELAY = std::chrono::seconds(5);
};

} // namespace squatch_nodes