#include "uart.h"
#include <cstring>
#include "esp_log.h"
#include "driver/gpio.h"

#ifdef CONFIG_GNSS_MOCK
#include "uart_mock.h"
#endif

static const char* TAG = "GNSS_UART";

namespace gnss {

UARTPort::UARTPort(
    uart_port_t port,
    int tx_pin,
    int rx_pin,
    uint32_t baud_rate,
    size_t rx_buffer_size,
    size_t tx_buffer_size
) : port_(port),
    tx_pin_(tx_pin),
    rx_pin_(rx_pin),
    baud_rate_(baud_rate),
    rx_buffer_size_(rx_buffer_size),
    tx_buffer_size_(tx_buffer_size) {
}

UARTPort::~UARTPort() {
    if (initialized_) {
        uart_driver_delete(port_);
        ESP_LOGI(TAG, "UART port %d cleaned up", port_);
    }
}

UARTPort::UARTPort(UARTPort&& other) noexcept 
    : port_(other.port_),
      tx_pin_(other.tx_pin_),
      rx_pin_(other.rx_pin_),
      baud_rate_(other.baud_rate_),
      rx_buffer_size_(other.rx_buffer_size_),
      tx_buffer_size_(other.tx_buffer_size_),
      initialized_(other.initialized_) {
    other.initialized_ = false;
}

UARTPort& UARTPort::operator=(UARTPort&& other) noexcept {
    if (this != &other) {
        if (initialized_) {
            uart_driver_delete(port_);
        }
        
        port_ = other.port_;
        tx_pin_ = other.tx_pin_;
        rx_pin_ = other.rx_pin_;
        baud_rate_ = other.baud_rate_;
        rx_buffer_size_ = other.rx_buffer_size_;
        tx_buffer_size_ = other.tx_buffer_size_;
        initialized_ = other.initialized_;
        
        other.initialized_ = false;
    }
    return *this;
}

std::expected<void, esp_err_t> UARTPort::init() {
    if (initialized_) {
        return std::unexpected(ESP_ERR_INVALID_STATE);
    }
    
#ifdef CONFIG_GNSS_MOCK
    // Mock initialization - no real UART setup
    initialized_ = true;
    ESP_LOGI(TAG, "UART port %d initialized (MOCK): TX=%d, RX=%d, baud=%lu", 
             port_, tx_pin_, rx_pin_, baud_rate_);
    return {};
#else
    uart_config_t uart_config = {
        .baud_rate = static_cast<int>(baud_rate_),
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 122,
        .source_clk = UART_SCLK_APB,
    };
    
    esp_err_t ret = uart_param_config(port_, &uart_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure UART parameters: %s", esp_err_to_name(ret));
        return std::unexpected(ret);
    }
    
    ret = uart_set_pin(port_, tx_pin_, rx_pin_, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set UART pins: %s", esp_err_to_name(ret));
        return std::unexpected(ret);
    }
    
    ret = uart_driver_install(port_, rx_buffer_size_, tx_buffer_size_, 0, NULL, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to install UART driver: %s", esp_err_to_name(ret));
        return std::unexpected(ret);
    }
    
    initialized_ = true;
#endif
    ESP_LOGI(TAG, "UART port %d initialized: TX=%d, RX=%d, baud=%lu", 
             port_, tx_pin_, rx_pin_, baud_rate_);
    
    return {};
}

std::expected<size_t, esp_err_t> UARTPort::read(
    uint8_t* buffer,
    size_t length,
    uint32_t timeout_ms
) {
    if (!initialized_) {
        return std::unexpected(ESP_ERR_INVALID_STATE);
    }
    
#ifdef CONFIG_GNSS_MOCK
    // Mock NMEA data - generate realistic GPS sentences
    static std::string current_sentence;
    static size_t sentence_pos = 0;
    static uint32_t last_sentence_time = 0;
    
    uint32_t current_time = esp_timer_get_time() / 1000;
    
    // Generate new NMEA sentence every 1000ms
    if (current_time - last_sentence_time >= 1000 || current_sentence.empty()) {
        // Alternate between GGA and RMC sentences using mock generators
        static bool send_gga = true;
        if (send_gga) {
            current_sentence = generate_gga_sentence();
        } else {
            current_sentence = generate_rmc_sentence();
        }
        send_gga = !send_gga;
        
        sentence_pos = 0;
        last_sentence_time = current_time;
    }
    
    // Copy available data to buffer
    size_t available = current_sentence.length() - sentence_pos;
    size_t to_copy = std::min(length, available);
    
    if (to_copy > 0) {
        memcpy(buffer, current_sentence.data() + sentence_pos, to_copy);
        sentence_pos += to_copy;
        
        // Reset when sentence is complete
        if (sentence_pos >= current_sentence.length()) {
            current_sentence.clear();
            sentence_pos = 0;
        }
    }
    
    return to_copy;
#else
    TickType_t timeout_ticks = timeout_ms / portTICK_PERIOD_MS;
    int bytes_read = uart_read_bytes(port_, buffer, length, timeout_ticks);
    
    if (bytes_read < 0) {
        ESP_LOGE(TAG, "UART read error");
        return std::unexpected(ESP_FAIL);
    }
    
    return static_cast<size_t>(bytes_read);
#endif
}

std::expected<size_t, esp_err_t> UARTPort::write(
    const uint8_t* data,
    size_t length
) {
    if (!initialized_) {
        return std::unexpected(ESP_ERR_INVALID_STATE);
    }
    
    int bytes_written = uart_write_bytes(port_, data, length);
    
    if (bytes_written < 0) {
        ESP_LOGE(TAG, "UART write error");
        return std::unexpected(ESP_FAIL);
    }
    
    return static_cast<size_t>(bytes_written);
}

std::expected<size_t, esp_err_t> UARTPort::read_sentence(
    char* buffer,
    size_t buffer_size,
    uint32_t timeout_ms
) {
    if (!initialized_) {
        return std::unexpected(ESP_ERR_INVALID_STATE);
    }
    
    size_t pos = 0;
    uint32_t start_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
    
    while (pos < buffer_size - 1) {
        uint32_t elapsed = (xTaskGetTickCount() * portTICK_PERIOD_MS) - start_time;
        if (elapsed >= timeout_ms) {
            return std::unexpected(ESP_ERR_TIMEOUT);
        }
        
        uint8_t byte;
        auto result = read(&byte, 1, 100);
        if (!result) {
            if (result.error() == ESP_ERR_TIMEOUT) {
                continue;
            }
            return std::unexpected(result.error());
        }
        
        if (result.value() == 0) {
            continue;
        }
        
        buffer[pos++] = static_cast<char>(byte);
        
        // Check for end of NMEA sentence
        if (pos >= 2 && buffer[pos-2] == '\r' && buffer[pos-1] == '\n') {
            buffer[pos] = '\0';
            return pos;
        }
        
        // Also handle just \n
        if (byte == '\n') {
            buffer[pos] = '\0';
            return pos;
        }
    }
    
    return std::unexpected(ESP_ERR_NO_MEM);
}

std::expected<size_t, esp_err_t> UARTPort::available() const {
    if (!initialized_) {
        return std::unexpected(ESP_ERR_INVALID_STATE);
    }
    
#ifdef CONFIG_GNSS_MOCK
    // Mock data availability - simulate having NMEA data every 1000ms
    static uint32_t last_check = 0;
    uint32_t current_time = esp_timer_get_time() / 1000;
    
    if (current_time - last_check >= 1000) {
        last_check = current_time;
        return 80;  // Typical NMEA sentence length
    }
    return 0;  // No new data available yet
#else
    // Use real UART driver
    size_t available_bytes = 0;
    esp_err_t ret = uart_get_buffered_data_len(port_, &available_bytes);
    
    if (ret != ESP_OK) {
        return std::unexpected(ret);
    }
    
    return available_bytes;
#endif
}

} // namespace gnss