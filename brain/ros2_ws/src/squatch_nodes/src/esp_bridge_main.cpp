#include "squatch_nodes/esp_bridge.hpp"
#include <rclcpp/rclcpp.hpp>
#include <memory>

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    
    // Parse command line arguments
    std::string host = "localhost";
    unsigned int port = 5555;
    
    if (argc > 1) {
        host = argv[1];
    }
    if (argc > 2) {
        port = std::stoul(argv[2]);
    }
    
    auto node = std::make_shared<squatch_nodes::ESPBridge>(host, port);
    
    RCLCPP_INFO(node->get_logger(), "ESP Bridge node started");
    
    rclcpp::spin(node);
    
    rclcpp::shutdown();
    return 0;
}