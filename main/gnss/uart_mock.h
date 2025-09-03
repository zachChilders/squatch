#pragma once

#ifdef CONFIG_GNSS_MOCK

extern "C" {
#include "driver/uart.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"
}

#include <string>
#include <cstring>
#include <algorithm>

/**
 * @brief Mock GNSS data for UART simulation
 */
struct MockGNSSData {
    // GPS coordinates
    double latitude = 37.7749;      // San Francisco default
    double longitude = -122.4194;
    float altitude = 10.0f;         // 10 meters above sea level
    
    // Fix information
    uint8_t satellites = 8;         // Good satellite count
    float hdop = 1.2f;             // Good precision
    bool fix_valid = true;         // Valid fix
    uint8_t fix_quality = 1;       // 0=no fix, 1=GPS fix, 2=DGPS fix
    
    // Time information
    uint32_t utc_time = 123000;    // 12:30:00 UTC
    uint32_t utc_date = 020125;    // 02/01/25 (DDMMYY)
    
    // Movement simulation
    float speed_knots = 0.0f;      // Speed in knots
    float course_degrees = 0.0f;   // Course over ground
    
    // Signal simulation
    bool signal_available = true;   // Simulate signal loss
    uint32_t last_update_ms = 0;   // For timing NMEA sentences
};

// Global mock GNSS data
extern MockGNSSData g_mock_gnss;

/**
 * @brief Generate NMEA checksum
 */
inline uint8_t nmea_checksum(const std::string& sentence) {
    uint8_t checksum = 0;
    // Calculate checksum between $ and *
    size_t start = sentence.find('$');
    size_t end = sentence.find('*');
    if (start != std::string::npos && end != std::string::npos) {
        for (size_t i = start + 1; i < end; ++i) {
            checksum ^= sentence[i];
        }
    }
    return checksum;
}

/**
 * @brief Generate GGA sentence (Global Positioning System Fix Data)
 */
inline std::string generate_gga_sentence() {
    if (!g_mock_gnss.signal_available || !g_mock_gnss.fix_valid) {
        return "$GPGGA,123000.00,,,,,0,00,99.99,,M,,M,,*63\r\n";
    }
    
    // Convert latitude to DDMM.MMMM format
    int lat_deg = static_cast<int>(std::abs(g_mock_gnss.latitude));
    double lat_min = (std::abs(g_mock_gnss.latitude) - lat_deg) * 60.0;
    char lat_dir = g_mock_gnss.latitude >= 0 ? 'N' : 'S';
    
    // Convert longitude to DDDMM.MMMM format  
    int lon_deg = static_cast<int>(std::abs(g_mock_gnss.longitude));
    double lon_min = (std::abs(g_mock_gnss.longitude) - lon_deg) * 60.0;
    char lon_dir = g_mock_gnss.longitude >= 0 ? 'E' : 'W';
    
    char sentence[128];
    snprintf(sentence, sizeof(sentence),
        "$GPGGA,%06lu.00,%02d%07.4f,%c,%03d%07.4f,%c,%d,%02d,%.2f,%.1f,M,0.0,M,,*",
        g_mock_gnss.utc_time,
        lat_deg, lat_min, lat_dir,
        lon_deg, lon_min, lon_dir,
        g_mock_gnss.fix_quality,
        g_mock_gnss.satellites,
        g_mock_gnss.hdop,
        g_mock_gnss.altitude
    );
    
    std::string nmea_sentence(sentence);
    uint8_t checksum = nmea_checksum(nmea_sentence);
    
    char final_sentence[144];
    snprintf(final_sentence, sizeof(final_sentence), "%s%02X\r\n", sentence, checksum);
    
    return std::string(final_sentence);
}

/**
 * @brief Generate RMC sentence (Recommended Minimum Course)
 */
inline std::string generate_rmc_sentence() {
    if (!g_mock_gnss.signal_available || !g_mock_gnss.fix_valid) {
        return "$GPRMC,123000.00,V,,,,,,,020125,,,N*43\r\n";
    }
    
    // Convert latitude/longitude same as GGA
    int lat_deg = static_cast<int>(std::abs(g_mock_gnss.latitude));
    double lat_min = (std::abs(g_mock_gnss.latitude) - lat_deg) * 60.0;
    char lat_dir = g_mock_gnss.latitude >= 0 ? 'N' : 'S';
    
    int lon_deg = static_cast<int>(std::abs(g_mock_gnss.longitude));
    double lon_min = (std::abs(g_mock_gnss.longitude) - lon_deg) * 60.0;
    char lon_dir = g_mock_gnss.longitude >= 0 ? 'E' : 'W';
    
    char sentence[144];
    snprintf(sentence, sizeof(sentence),
        "$GPRMC,%06lu.00,A,%02d%07.4f,%c,%03d%07.4f,%c,%.2f,%.2f,%06lu,,,A*",
        g_mock_gnss.utc_time,
        lat_deg, lat_min, lat_dir,
        lon_deg, lon_min, lon_dir,
        g_mock_gnss.speed_knots,
        g_mock_gnss.course_degrees,
        g_mock_gnss.utc_date
    );
    
    std::string nmea_sentence(sentence);
    uint8_t checksum = nmea_checksum(nmea_sentence);
    
    char final_sentence[160];
    snprintf(final_sentence, sizeof(final_sentence), "%s%02X\r\n", sentence, checksum);
    
    return std::string(final_sentence);
}

/**
 * @brief Mock UART read function
 */
inline int uart_read_bytes(uart_port_t uart_num, void* buffer, uint32_t length, TickType_t ticks_to_wait) {
    static std::string current_sentence;
    static size_t sentence_pos = 0;
    static uint32_t last_sentence_time = 0;
    
    uint32_t current_time = esp_timer_get_time() / 1000; // Get milliseconds
    
    // Generate new NMEA sentence every 1000ms
    if (current_time - last_sentence_time >= 1000 || current_sentence.empty()) {
        // Alternate between GGA and RMC sentences
        static bool send_gga = true;
        if (send_gga) {
            current_sentence = generate_gga_sentence();
        } else {
            current_sentence = generate_rmc_sentence();
        }
        send_gga = !send_gga;
        
        sentence_pos = 0;
        last_sentence_time = current_time;
        g_mock_gnss.last_update_ms = current_time;
    }
    
    // Copy available data to buffer
    size_t available = current_sentence.length() - sentence_pos;
    size_t to_copy = std::min(static_cast<size_t>(length), available);
    
    if (to_copy > 0) {
        memcpy(buffer, current_sentence.data() + sentence_pos, to_copy);
        sentence_pos += to_copy;
        
        // Reset sentence position when complete
        if (sentence_pos >= current_sentence.length()) {
            current_sentence.clear();
            sentence_pos = 0;
        }
    }
    
    return static_cast<int>(to_copy);
}

/**
 * @brief Mock UART install driver function
 */
inline esp_err_t uart_driver_install(uart_port_t uart_num, int rx_buffer_size, 
                                     int tx_buffer_size, int queue_size,
                                     QueueHandle_t* uart_queue, int intr_alloc_flags) {
    return ESP_OK;  // Always succeed in mock mode
}

/**
 * @brief Mock UART parameter configuration
 */
inline esp_err_t uart_param_config(uart_port_t uart_num, const uart_config_t* uart_config) {
    return ESP_OK;  // Always succeed in mock mode
}

/**
 * @brief Mock UART pin configuration
 */
inline esp_err_t uart_set_pin(uart_port_t uart_num, int tx_io_num, int rx_io_num, 
                              int rts_io_num, int cts_io_num) {
    return ESP_OK;  // Always succeed in mock mode
}

/**
 * @brief Mock UART driver delete
 */
inline esp_err_t uart_driver_delete(uart_port_t uart_num) {
    return ESP_OK;  // Always succeed in mock mode
}

/**
 * @brief Mock UART get buffered data length
 */
inline esp_err_t uart_get_buffered_data_len(uart_port_t uart_num, size_t* size) {
    // Simulate having NMEA data available periodically
    static uint32_t last_check = 0;
    uint32_t current_time = esp_timer_get_time() / 1000;
    
    if (current_time - last_check >= 1000) {
        *size = 80;  // Typical NMEA sentence length
        last_check = current_time;
    } else {
        *size = 0;   // No new data available yet
    }
    
    return ESP_OK;
}

#endif // CONFIG_GNSS_MOCK