#pragma once

#include <expected>
#include <string_view>
#include <cstdint>
#include "esp_err.h"
#include "driver/uart.h"

/**
 * @brief UART communication wrapper for GNSS module
 * 
 * RAII wrapper for ESP-IDF UART operations with automatic
 * resource management and cleanup.
 */
namespace gnss {

/**
 * @brief RAII UART Port wrapper
 * 
 * Manages UART port lifecycle with automatic configuration
 * and cleanup following RAII principles.
 */
class UARTPort {
public:
    /**
     * @brief Construct UART port with configuration
     * @param port UART port number
     * @param tx_pin GPIO pin for TX
     * @param rx_pin GPIO pin for RX  
     * @param baud_rate Communication baud rate
     * @param rx_buffer_size RX buffer size
     * @param tx_buffer_size TX buffer size
     */
    UARTPort(
        uart_port_t port,
        int tx_pin,
        int rx_pin,
        uint32_t baud_rate,
        size_t rx_buffer_size,
        size_t tx_buffer_size
    );
    
    /**
     * @brief Destructor - ensures proper UART cleanup
     */
    ~UARTPort();
    
    // Non-copyable, moveable
    UARTPort(const UARTPort&) = delete;
    UARTPort& operator=(const UARTPort&) = delete;
    UARTPort(UARTPort&&) noexcept;
    UARTPort& operator=(UARTPort&&) noexcept;
    
    /**
     * @brief Initialize UART port
     * @return std::expected<void, esp_err_t> Success or error code
     */
    std::expected<void, esp_err_t> init();
    
    /**
     * @brief Read data from UART with timeout
     * @param buffer Buffer to store received data
     * @param length Maximum bytes to read
     * @param timeout_ms Timeout in milliseconds
     * @return std::expected<size_t, esp_err_t> Bytes read or error code
     */
    std::expected<size_t, esp_err_t> read(
        uint8_t* buffer,
        size_t length,
        uint32_t timeout_ms = 1000
    );
    
    /**
     * @brief Write data to UART
     * @param data Data to send
     * @param length Number of bytes to send
     * @return std::expected<size_t, esp_err_t> Bytes written or error code
     */
    std::expected<size_t, esp_err_t> write(
        const uint8_t* data,
        size_t length
    );
    
    /**
     * @brief Read a complete NMEA sentence
     * @param buffer Buffer to store the sentence
     * @param buffer_size Size of the buffer
     * @param timeout_ms Timeout in milliseconds
     * @return std::expected<size_t, esp_err_t> Sentence length or error code
     */
    std::expected<size_t, esp_err_t> read_sentence(
        char* buffer,
        size_t buffer_size,
        uint32_t timeout_ms = 5000
    );
    
    /**
     * @brief Check if data is available to read
     * @return std::expected<size_t, esp_err_t> Available bytes or error code
     */
    std::expected<size_t, esp_err_t> available() const;
    
    /**
     * @brief Check if UART is initialized
     * @return bool True if initialized
     */
    bool is_initialized() const { return initialized_; }
    
    /**
     * @brief Get UART port number
     * @return uart_port_t Port number
     */
    uart_port_t port() const { return port_; }

private:
    uart_port_t port_;
    int tx_pin_;
    int rx_pin_;
    uint32_t baud_rate_;
    size_t rx_buffer_size_;
    size_t tx_buffer_size_;
    bool initialized_{false};
};

} // namespace gnss