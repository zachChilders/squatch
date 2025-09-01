#pragma once

#include <expected>
#include <memory>
#include <cstdint>
#include "esp_err.h"
#include "driver/uart.h"

/**
 * @brief NEO-M8N GNSS Module Interface
 * 
 * RAII-based GNSS module interface for NEO-M8N GPS module
 * using modern C++23 patterns with std::expected error handling.
 */
namespace gnss {

/**
 * @brief GNSS coordinate data
 */
struct GNSSData {
    double latitude{0.0};        ///< Latitude in decimal degrees
    double longitude{0.0};       ///< Longitude in decimal degrees  
    float altitude{0.0f};        ///< Altitude in meters above sea level
    uint8_t satellites{0};       ///< Number of satellites in use
    float hdop{99.99f};          ///< Horizontal dilution of precision
    bool fix_valid{false};       ///< True if GPS fix is valid
    uint32_t timestamp{0};       ///< UTC timestamp (HHMMSS format)
    uint32_t date{0};            ///< UTC date (DDMMYY format)
};

/**
 * @brief GNSS configuration parameters
 */
struct GNSSConfig {
    uart_port_t uart_port{UART_NUM_1};     ///< UART port number
    int tx_pin{17};                         ///< TX pin (ESP32-S2 to NEO-M8N RX)
    int rx_pin{18};                         ///< RX pin (ESP32-S2 from NEO-M8N TX)
    uint32_t baud_rate{9600};              ///< Baud rate (default for NEO-M8N)
    size_t rx_buffer_size{1024};           ///< UART RX buffer size
    size_t tx_buffer_size{256};            ///< UART TX buffer size
};

// Forward declarations
class UARTPort;
class NMEAParser;

/**
 * @brief RAII GNSS Module class
 * 
 * Manages NEO-M8N GNSS module communication with automatic
 * resource management and lifecycle control.
 */
class GNSSModule {
public:
    /**
     * @brief Construct GNSS module with configuration
     * @param config GNSS configuration parameters
     */
    explicit GNSSModule(const GNSSConfig& config = {});
    
    /**
     * @brief Destructor - ensures proper cleanup
     */
    ~GNSSModule();
    
    // Non-copyable, moveable
    GNSSModule(const GNSSModule&) = delete;
    GNSSModule& operator=(const GNSSModule&) = delete;
    GNSSModule(GNSSModule&&) noexcept;
    GNSSModule& operator=(GNSSModule&&) noexcept;
    
    /**
     * @brief Initialize GNSS module
     * @return std::expected<void, esp_err_t> Success or error code
     */
    std::expected<void, esp_err_t> init();
    
    /**
     * @brief Read GNSS data
     * @return std::expected<GNSSData, esp_err_t> GNSS data or error code
     */
    std::expected<GNSSData, esp_err_t> read();
    
    /**
     * @brief Check if GNSS module is available/responding
     * @return std::expected<bool, esp_err_t> True if available or error code
     */
    std::expected<bool, esp_err_t> is_available() const;
    
    /**
     * @brief Get current configuration
     * @return const GNSSConfig& Current configuration
     */
    const GNSSConfig& config() const { return config_; }
    
    /**
     * @brief Check if module is initialized
     * @return bool True if initialized
     */
    bool is_initialized() const { return initialized_; }

private:
    GNSSConfig config_;
    bool initialized_{false};
    std::unique_ptr<UARTPort> uart_;
    std::unique_ptr<NMEAParser> parser_;
};

} // namespace gnss