#pragma once

#include <cstdint>
#include "../lib/json.hpp"
#include "esp_timer.h"

// Forward declarations
struct IMUData;
namespace gnss { struct GNSSData; }

namespace logging {

/// @brief JSON serializable accelerometer data
struct AccelModel {
    float x{0.0f};
    float y{0.0f};
    float z{0.0f};
    
    AccelModel() = default;
    AccelModel(float x_val, float y_val, float z_val) : x(x_val), y(y_val), z(z_val) {}
};

/// @brief JSON serializable gyroscope data
struct GyroModel {
    float x{0.0f};
    float y{0.0f};
    float z{0.0f};
    
    GyroModel() = default;
    GyroModel(float x_val, float y_val, float z_val) : x(x_val), y(y_val), z(z_val) {}
};

/// @brief JSON serializable IMU data
struct IMUModel {
    bool valid{false};
    AccelModel accel;
    GyroModel gyro;
    float temperature{0.0f};
    
    IMUModel() = default;
    IMUModel(bool is_valid, const AccelModel& accel_data, const GyroModel& gyro_data, float temp)
        : valid(is_valid), accel(accel_data), gyro(gyro_data), temperature(temp) {}
};

/// @brief JSON serializable GNSS data
struct GNSSModel {
    bool valid{false};
    double latitude{0.0};
    double longitude{0.0};
    float altitude{0.0f};
    uint8_t satellites{0};
    float hdop{99.99f};
    bool fix_valid{false};
    uint32_t time{0};
    uint32_t date{0};
    
    GNSSModel() = default;
    GNSSModel(bool is_valid, double lat, double lon, float alt, uint8_t sats, 
             float hdop_val, bool fix, uint32_t time_val, uint32_t date_val)
        : valid(is_valid), latitude(lat), longitude(lon), altitude(alt), 
          satellites(sats), hdop(hdop_val), fix_valid(fix), time(time_val), date(date_val) {}
};

/// @brief Complete sensor data packet for JSON serialization
struct SensorPacket {
    uint64_t timestamp{0};       ///< Microseconds since boot
    IMUModel imu;                ///< IMU data with validity
    GNSSModel gnss;              ///< GNSS data with validity
    
    SensorPacket() : timestamp(esp_timer_get_time()) {}
    SensorPacket(const IMUModel& imu_data, const GNSSModel& gnss_data)
        : timestamp(esp_timer_get_time()), imu(imu_data), gnss(gnss_data) {}
};

// Helper function to create models from actual data (implemented in structured_logger.h)
IMUModel create_imu_model(const IMUData& imu_data, bool is_valid);
GNSSModel create_gnss_model(const gnss::GNSSData& gnss_data, bool is_valid);

} // namespace logging

// JSON serialization support using nlohmann/json
namespace nlohmann {
    template<>
    struct adl_serializer<logging::AccelModel> {
        static void to_json(json& j, const logging::AccelModel& accel) {
            j = json{{"x", accel.x}, {"y", accel.y}, {"z", accel.z}};
        }
    };
    
    template<>
    struct adl_serializer<logging::GyroModel> {
        static void to_json(json& j, const logging::GyroModel& gyro) {
            j = json{{"x", gyro.x}, {"y", gyro.y}, {"z", gyro.z}};
        }
    };
    
    template<>
    struct adl_serializer<logging::IMUModel> {
        static void to_json(json& j, const logging::IMUModel& imu) {
            j = json{
                {"valid", imu.valid},
                {"accel", imu.accel},
                {"gyro", imu.gyro},
                {"temperature", imu.temperature}
            };
        }
    };
    
    template<>
    struct adl_serializer<logging::GNSSModel> {
        static void to_json(json& j, const logging::GNSSModel& gnss) {
            j = json{
                {"valid", gnss.valid},
                {"latitude", gnss.latitude},
                {"longitude", gnss.longitude},
                {"altitude", gnss.altitude},
                {"satellites", gnss.satellites},
                {"hdop", gnss.hdop},
                {"fix_valid", gnss.fix_valid},
                {"time", gnss.time},
                {"date", gnss.date}
            };
        }
    };
    
    template<>
    struct adl_serializer<logging::SensorPacket> {
        static void to_json(json& j, const logging::SensorPacket& packet) {
            j = json{
                {"timestamp", packet.timestamp},
                {"imu", packet.imu},
                {"gnss", packet.gnss}
            };
        }
    };
}