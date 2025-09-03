#pragma once

#include <expected>
#include <string_view>
#include <array>
#include <cstdint>
#include "esp_err.h"
#include "gnss.h"

/**
 * @brief NMEA 0183 sentence parsing utilities
 * 
 * Stateful RAII parser for NMEA sentences from GNSS modules
 * following NMEA 0183 specification.
 */
namespace gnss {

/**
 * @brief NMEA sentence types
 */
enum class SentenceType {
    UNKNOWN,
    GGA,    ///< Global Positioning System Fix Data
    RMC,    ///< Recommended Minimum Course
    GSV,    ///< Satellites in view
    GSA,    ///< GPS DOP and active satellites
    VTG     ///< Track made good and ground speed
};

/**
 * @brief RAII NMEA Parser class
 * 
 * Stateful parser that maintains sentence buffers and parsing
 * state between calls for robust NMEA sentence processing.
 */
class NMEAParser {
public:
    /**
     * @brief Construct NMEA parser
     */
    NMEAParser();
    
    /**
     * @brief Destructor
     */
    ~NMEAParser() = default;
    
    // Non-copyable, moveable
    NMEAParser(const NMEAParser&) = delete;
    NMEAParser& operator=(const NMEAParser&) = delete;
    NMEAParser(NMEAParser&&) noexcept = default;
    NMEAParser& operator=(NMEAParser&&) noexcept = default;
    
    /**
     * @brief Parse NMEA sentence and update GNSS data
     * @param sentence NMEA sentence string
     * @param data Reference to GNSSData to update
     * @return std::expected<bool, esp_err_t> True if sentence was parsed successfully
     */
    std::expected<bool, esp_err_t> parse_sentence(std::string_view sentence, GNSSData& data);
    
    /**
     * @brief Add raw data to internal buffer and attempt parsing
     * @param raw_data Raw bytes from UART
     * @param length Number of bytes
     * @param data Reference to GNSSData to update
     * @return std::expected<size_t, esp_err_t> Number of sentences parsed or error
     */
    std::expected<size_t, esp_err_t> add_data(const uint8_t* raw_data, size_t length, GNSSData& data);
    
    /**
     * @brief Clear internal buffers
     */
    void clear();
    
    /**
     * @brief Get sentence type from NMEA string
     * @param sentence NMEA sentence
     * @return SentenceType The type of NMEA sentence
     */
    static SentenceType get_sentence_type(std::string_view sentence);
    
    /**
     * @brief Validate NMEA sentence checksum
     * @param sentence Complete NMEA sentence with checksum
     * @return std::expected<bool, esp_err_t> True if checksum is valid
     */
    static std::expected<bool, esp_err_t> validate_checksum(std::string_view sentence);

private:
    static constexpr size_t BUFFER_SIZE = 512;
    
    std::array<char, BUFFER_SIZE> buffer_;
    size_t buffer_pos_{0};
    
    /**
     * @brief Parse GGA sentence (Global Positioning System Fix Data)
     * @param sentence GGA sentence string
     * @param data Reference to GNSSData to update
     * @return std::expected<bool, esp_err_t> True if parsed successfully
     */
    std::expected<bool, esp_err_t> parse_gga(std::string_view sentence, GNSSData& data);
    
    /**
     * @brief Parse RMC sentence (Recommended Minimum Course)
     * @param sentence RMC sentence string
     * @param data Reference to GNSSData to update
     * @return std::expected<bool, esp_err_t> True if parsed successfully
     */
    std::expected<bool, esp_err_t> parse_rmc(std::string_view sentence, GNSSData& data);
    
    /**
     * @brief Convert NMEA coordinate format to decimal degrees
     * @param nmea_coord NMEA coordinate (DDMM.MMMM or DDDMM.MMMM format)
     * @param direction Direction character ('N', 'S', 'E', 'W')
     * @return std::expected<double, esp_err_t> Coordinate in decimal degrees
     */
    static std::expected<double, esp_err_t> convert_coordinate(
        std::string_view nmea_coord,
        char direction
    );
    
    /**
     * @brief Convert NMEA time to integer format
     * @param time_str NMEA time string (HHMMSS.SSS format)
     * @return std::expected<uint32_t, esp_err_t> Time as HHMMSS integer
     */
    static std::expected<uint32_t, esp_err_t> convert_time(std::string_view time_str);
    
    /**
     * @brief Convert NMEA date to integer format
     * @param date_str NMEA date string (DDMMYY format)
     * @return std::expected<uint32_t, esp_err_t> Date as DDMMYY integer
     */
    static std::expected<uint32_t, esp_err_t> convert_date(std::string_view date_str);
    
    /**
     * @brief Extract complete sentences from buffer
     * @param data Reference to GNSSData to update
     * @return std::expected<size_t, esp_err_t> Number of sentences processed
     */
    std::expected<size_t, esp_err_t> process_buffer(GNSSData& data);
};

} // namespace gnss