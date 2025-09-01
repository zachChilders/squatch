#include "gnss.h"
#include "uart.h"
#include "nmea.h"
#include <memory>
#include "esp_log.h"

static const char* TAG = "GNSS_MODULE";

namespace gnss {

GNSSModule::GNSSModule(const GNSSConfig& config) : config_(config) {
}

GNSSModule::~GNSSModule() {
    // RAII cleanup - unique_ptrs handle deletion automatically
    if (initialized_) {
        ESP_LOGI(TAG, "GNSS module destroyed");
    }
}

GNSSModule::GNSSModule(GNSSModule&& other) noexcept 
    : config_(other.config_),
      initialized_(other.initialized_),
      uart_(std::move(other.uart_)),
      parser_(std::move(other.parser_)) {
    other.initialized_ = false;
}

GNSSModule& GNSSModule::operator=(GNSSModule&& other) noexcept {
    if (this != &other) {
        config_ = other.config_;
        initialized_ = other.initialized_;
        uart_ = std::move(other.uart_);
        parser_ = std::move(other.parser_);
        
        other.initialized_ = false;
    }
    return *this;
}

std::expected<void, esp_err_t> GNSSModule::init() {
    if (initialized_) {
        return std::unexpected(ESP_ERR_INVALID_STATE);
    }
    
    // Create UART port
    uart_ = std::make_unique<UARTPort>(
        config_.uart_port,
        config_.tx_pin,
        config_.rx_pin,
        config_.baud_rate,
        config_.rx_buffer_size,
        config_.tx_buffer_size
    );
    
    // Initialize UART
    auto uart_result = uart_->init();
    if (!uart_result) {
        ESP_LOGE(TAG, "Failed to initialize UART: %s", esp_err_to_name(uart_result.error()));
        uart_.reset();
        return std::unexpected(uart_result.error());
    }
    
    // Create NMEA parser
    parser_ = std::make_unique<NMEAParser>();
    
    initialized_ = true;
    ESP_LOGI(TAG, "GNSS module initialized successfully");
    
    return {};
}

std::expected<GNSSData, esp_err_t> GNSSModule::read() {
    if (!initialized_) {
        return std::unexpected(ESP_ERR_INVALID_STATE);
    }
    
    GNSSData data{};
    
    // Check if data is available
    auto available_result = uart_->available();
    if (!available_result) {
        return std::unexpected(available_result.error());
    }
    
    if (available_result.value() == 0) {
        // No data available, return empty data
        return data;
    }
    
    // Read raw data from UART
    constexpr size_t READ_BUFFER_SIZE = 256;
    uint8_t buffer[READ_BUFFER_SIZE];
    
    auto read_result = uart_->read(buffer, READ_BUFFER_SIZE, 100);
    if (!read_result) {
        if (read_result.error() == ESP_ERR_TIMEOUT) {
            // Timeout is normal, return current data
            return data;
        }
        return std::unexpected(read_result.error());
    }
    
    if (read_result.value() == 0) {
        // No data read, return current data
        return data;
    }
    
    // Parse data through NMEA parser
    auto parse_result = parser_->add_data(buffer, read_result.value(), data);
    if (!parse_result) {
        ESP_LOGW(TAG, "Failed to parse NMEA data: %s", esp_err_to_name(parse_result.error()));
        // Don't fail completely, just log warning and return data
    }
    
    return data;
}

std::expected<bool, esp_err_t> GNSSModule::is_available() const {
    if (!initialized_) {
        return std::unexpected(ESP_ERR_INVALID_STATE);
    }
    
    // Check if UART has data available
    auto available_result = uart_->available();
    if (!available_result) {
        return std::unexpected(available_result.error());
    }
    
    return available_result.value() > 0;
}

} // namespace gnss