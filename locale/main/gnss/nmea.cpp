#include "nmea.h"
#include <cstdlib>
#include <cstring>
#include <cmath>
#include "esp_log.h"

static const char* TAG = "GNSS_NMEA";

namespace gnss {

NMEAParser::NMEAParser() {
    buffer_.fill(0);
}

std::expected<bool, esp_err_t> NMEAParser::parse_sentence(std::string_view sentence, GNSSData& data) {
    if (sentence.empty() || sentence[0] != '$') {
        return false;
    }
    
    // Validate checksum first
    auto checksum_result = validate_checksum(sentence);
    if (!checksum_result) {
        return std::unexpected(checksum_result.error());
    }
    if (!checksum_result.value()) {
        ESP_LOGW(TAG, "Invalid checksum for sentence: %.20s", sentence.data());
        return false;
    }
    
    SentenceType type = get_sentence_type(sentence);
    
    switch (type) {
        case SentenceType::GGA:
            return parse_gga(sentence, data);
        case SentenceType::RMC:
            return parse_rmc(sentence, data);
        default:
            return false;
    }
}

std::expected<size_t, esp_err_t> NMEAParser::add_data(const uint8_t* raw_data, size_t length, GNSSData& data) {
    size_t sentences_parsed = 0;
    
    for (size_t i = 0; i < length; ++i) {
        if (buffer_pos_ >= BUFFER_SIZE - 1) {
            // Buffer full, shift or clear
            buffer_pos_ = 0;
            ESP_LOGW(TAG, "Buffer overflow, clearing");
        }
        
        buffer_[buffer_pos_++] = static_cast<char>(raw_data[i]);
        
        // Check for complete sentence (ends with \n)
        if (raw_data[i] == '\n') {
            buffer_[buffer_pos_] = '\0';
            
            // Find start of sentence
            size_t sentence_start = 0;
            for (size_t j = 0; j < buffer_pos_; ++j) {
                if (buffer_[j] == '$') {
                    sentence_start = j;
                    break;
                }
            }
            
            if (sentence_start < buffer_pos_) {
                std::string_view sentence(buffer_.data() + sentence_start, buffer_pos_ - sentence_start);
                auto parse_result = parse_sentence(sentence, data);
                if (parse_result && parse_result.value()) {
                    sentences_parsed++;
                }
            }
            
            buffer_pos_ = 0;
        }
    }
    
    return sentences_parsed;
}

void NMEAParser::clear() {
    buffer_.fill(0);
    buffer_pos_ = 0;
}

SentenceType NMEAParser::get_sentence_type(std::string_view sentence) {
    if (sentence.length() < 6) return SentenceType::UNKNOWN;
    
    if (sentence.substr(3, 3) == "GGA") return SentenceType::GGA;
    if (sentence.substr(3, 3) == "RMC") return SentenceType::RMC;
    if (sentence.substr(3, 3) == "GSV") return SentenceType::GSV;
    if (sentence.substr(3, 3) == "GSA") return SentenceType::GSA;
    if (sentence.substr(3, 3) == "VTG") return SentenceType::VTG;
    
    return SentenceType::UNKNOWN;
}

std::expected<bool, esp_err_t> NMEAParser::validate_checksum(std::string_view sentence) {
    if (sentence.length() < 4) return false;
    
    // Find checksum delimiter
    size_t star_pos = sentence.find_last_of('*');
    if (star_pos == std::string_view::npos || star_pos + 3 > sentence.length()) {
        return false;
    }
    
    // Calculate checksum
    uint8_t calculated_checksum = 0;
    for (size_t i = 1; i < star_pos; ++i) {
        calculated_checksum ^= sentence[i];
    }
    
    // Parse provided checksum
    char checksum_str[3] = {sentence[star_pos + 1], sentence[star_pos + 2], '\0'};
    uint8_t provided_checksum = static_cast<uint8_t>(strtol(checksum_str, nullptr, 16));
    
    return calculated_checksum == provided_checksum;
}

std::expected<bool, esp_err_t> NMEAParser::parse_gga(std::string_view sentence, GNSSData& data) {
    // $GPGGA,hhmmss.ss,llll.ll,a,yyyyy.yy,a,x,xx,x.x,x.x,M,x.x,M,x.x,xxxx*hh
    
    size_t pos = 0;
    size_t field = 0;
    std::string_view fields[15];
    
    // Split by commas
    while (pos < sentence.length() && field < 15) {
        size_t comma = sentence.find(',', pos);
        if (comma == std::string_view::npos) {
            comma = sentence.find('*', pos);
            if (comma == std::string_view::npos) break;
        }
        
        fields[field++] = sentence.substr(pos, comma - pos);
        pos = comma + 1;
    }
    
    if (field < 10) return false;
    
    // Parse time (field 1)
    if (!fields[1].empty()) {
        auto time_result = convert_time(fields[1]);
        if (time_result) {
            data.timestamp = time_result.value();
        }
    }
    
    // Parse latitude (fields 2,3)
    if (!fields[2].empty() && !fields[3].empty()) {
        auto lat_result = convert_coordinate(fields[2], fields[3][0]);
        if (lat_result) {
            data.latitude = lat_result.value();
        }
    }
    
    // Parse longitude (fields 4,5)
    if (!fields[4].empty() && !fields[5].empty()) {
        auto lon_result = convert_coordinate(fields[4], fields[5][0]);
        if (lon_result) {
            data.longitude = lon_result.value();
        }
    }
    
    // Parse fix quality (field 6)
    if (!fields[6].empty()) {
        int fix_quality = atoi(fields[6].data());
        data.fix_valid = (fix_quality > 0);
    }
    
    // Parse satellites (field 7)
    if (!fields[7].empty()) {
        data.satellites = static_cast<uint8_t>(atoi(fields[7].data()));
    }
    
    // Parse HDOP (field 8)
    if (!fields[8].empty()) {
        data.hdop = static_cast<float>(atof(fields[8].data()));
    }
    
    // Parse altitude (field 9)
    if (!fields[9].empty()) {
        data.altitude = static_cast<float>(atof(fields[9].data()));
    }
    
    return true;
}

std::expected<bool, esp_err_t> NMEAParser::parse_rmc(std::string_view sentence, GNSSData& data) {
    // $GPRMC,hhmmss.ss,A,llll.ll,a,yyyyy.yy,a,x.x,x.x,ddmmyy,x.x,a*hh
    
    size_t pos = 0;
    size_t field = 0;
    std::string_view fields[13];
    
    // Split by commas
    while (pos < sentence.length() && field < 13) {
        size_t comma = sentence.find(',', pos);
        if (comma == std::string_view::npos) {
            comma = sentence.find('*', pos);
            if (comma == std::string_view::npos) break;
        }
        
        fields[field++] = sentence.substr(pos, comma - pos);
        pos = comma + 1;
    }
    
    if (field < 10) return false;
    
    // Parse status (field 2) - A = active, V = void
    if (!fields[2].empty()) {
        data.fix_valid = (fields[2][0] == 'A');
    }
    
    // Parse time (field 1)
    if (!fields[1].empty()) {
        auto time_result = convert_time(fields[1]);
        if (time_result) {
            data.timestamp = time_result.value();
        }
    }
    
    // Parse latitude (fields 3,4)
    if (!fields[3].empty() && !fields[4].empty()) {
        auto lat_result = convert_coordinate(fields[3], fields[4][0]);
        if (lat_result) {
            data.latitude = lat_result.value();
        }
    }
    
    // Parse longitude (fields 5,6)
    if (!fields[5].empty() && !fields[6].empty()) {
        auto lon_result = convert_coordinate(fields[5], fields[6][0]);
        if (lon_result) {
            data.longitude = lon_result.value();
        }
    }
    
    // Parse date (field 9)
    if (!fields[9].empty()) {
        auto date_result = convert_date(fields[9]);
        if (date_result) {
            data.date = date_result.value();
        }
    }
    
    return true;
}

std::expected<double, esp_err_t> NMEAParser::convert_coordinate(std::string_view nmea_coord, char direction) {
    if (nmea_coord.empty()) {
        return std::unexpected(ESP_ERR_INVALID_ARG);
    }
    
    double coord = atof(nmea_coord.data());
    
    // NMEA format: DDMM.MMMM or DDDMM.MMMM
    int degrees = static_cast<int>(coord / 100);
    double minutes = coord - (degrees * 100);
    
    double decimal_degrees = degrees + (minutes / 60.0);
    
    // Apply hemisphere
    if (direction == 'S' || direction == 'W') {
        decimal_degrees = -decimal_degrees;
    }
    
    return decimal_degrees;
}

std::expected<uint32_t, esp_err_t> NMEAParser::convert_time(std::string_view time_str) {
    if (time_str.length() < 6) {
        return std::unexpected(ESP_ERR_INVALID_ARG);
    }
    
    // Extract HHMMSS from HHMMSS.SSS
    char time_chars[7];
    strncpy(time_chars, time_str.data(), 6);
    time_chars[6] = '\0';
    
    return static_cast<uint32_t>(atol(time_chars));
}

std::expected<uint32_t, esp_err_t> NMEAParser::convert_date(std::string_view date_str) {
    if (date_str.length() < 6) {
        return std::unexpected(ESP_ERR_INVALID_ARG);
    }
    
    char date_chars[7];
    strncpy(date_chars, date_str.data(), 6);
    date_chars[6] = '\0';
    
    return static_cast<uint32_t>(atol(date_chars));
}

std::expected<size_t, esp_err_t> NMEAParser::process_buffer(GNSSData& data) {
    // This is called by add_data, implementation included there
    return 0;
}

} // namespace gnss