#pragma once

extern "C" {
#include "esp_err.h"
#include "esp_log.h"
}

#include "i2c.h"
#include "registers.h"
#include <array>
#include <expected>
#include <cassert>

struct IMUData {
  struct AccelData {
    float x;
    float y;
    float z;
  } accel;

  struct GyroData {
    float x;
    float y;
    float z;
  } gyro;

  float temperature;
};

/**
 * @brief MPU6050 IMU sensor interface
 */
class IMU {
public:
  /**
   * @brief Constructor - Create MPU6050 instance (call init() to initialize)
   */
  IMU() : initialized(false) {}

  /**
   * @brief Initialize MPU6050 sensor
   * @param TAG Log tag for ESP logging
   * @return std::expected containing void on success, or esp_err_t on failure
   */
  std::expected<void, esp_err_t> init(const char *TAG) {
    // Check WHO_AM_I register
    auto who_am_i_result = register_read<1>(MPU6050_WHO_AM_I);
    if (!who_am_i_result) {
      ESP_LOGE(TAG, "Failed to read WHO_AM_I register");
      return std::unexpected(who_am_i_result.error());
    }

    uint8_t who_am_i = who_am_i_result.value()[0];
    ESP_LOGI(TAG, "WHO_AM_I: 0x%02X", who_am_i);

    if (who_am_i != 0x68) {
      ESP_LOGE(TAG, "MPU6050 not found, WHO_AM_I: 0x%02X", who_am_i);
      return std::unexpected(ESP_ERR_NOT_FOUND);
    }

    // Wake up MPU6050
    esp_err_t ret = register_write_byte(MPU6050_PWR_MGMT_1, 0x00);
    if (ret != ESP_OK) {
      ESP_LOGE(TAG, "Failed to wake up MPU6050");
      return std::unexpected(ret);
    }

    ESP_LOGI(TAG, "MPU6050 initialized successfully");
    initialized = true;
    return {};
  }

  /**
   * @brief Read accelerometer, gyroscope, and temperature data
   * @return std::expected containing IMUData on success, or esp_err_t on failure
   */
  std::expected<IMUData, esp_err_t> read_imu_data() {
    assert(initialized && "MPU6050 must be initialized before reading data");
    
    // Read all sensor data in one go (14 bytes starting from accelerometer)
    auto result = register_read<14>(MPU6050_ACCEL_XOUT_H);
    if (!result) {
      return std::unexpected(result.error());
    }

    const auto &data = result.value();
    IMUData imu_data{};

    // Parse accelerometer data (2g range, 16384 LSB/g)
    imu_data.accel.x = static_cast<int16_t>((data[0] << 8) | data[1]) / 16384.0f;
    imu_data.accel.y = static_cast<int16_t>((data[2] << 8) | data[3]) / 16384.0f;
    imu_data.accel.z = static_cast<int16_t>((data[4] << 8) | data[5]) / 16384.0f;

    // Parse temperature data (convert to Celsius: temp = (raw / 340.0) + 36.53)
    int16_t temp_raw = static_cast<int16_t>((data[6] << 8) | data[7]);
    imu_data.temperature = (temp_raw / 340.0f) + 36.53f;

    // Parse gyroscope data (250°/s range, 131 LSB/°/s)
    imu_data.gyro.x = static_cast<int16_t>((data[8] << 8) | data[9]) / 131.0f;
    imu_data.gyro.y = static_cast<int16_t>((data[10] << 8) | data[11]) / 131.0f;
    imu_data.gyro.z = static_cast<int16_t>((data[12] << 8) | data[13]) / 131.0f;

    return imu_data;
  }

private:
  /**
   * @brief Read from MPU6050 register (raw implementation)
   * @param reg_addr Register address to read from
   * @param data Pointer to buffer to store read data
   * @param len Number of bytes to read
   * @return ESP_OK on success, error code on failure
   */
  esp_err_t register_read_raw(uint8_t reg_addr, uint8_t *data, size_t len) {
    return i2c_master_write_read_device(
        I2C_MASTER_NUM, MPU6050_ADDR, &reg_addr, 1, data, len,
        I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
  }

  /**
   * @brief Read from MPU6050 register and return data wrapped in std::expected
   * @tparam N Number of bytes to read
   * @param reg_addr Register address to read from
   * @return std::expected containing array of data bytes on success, or esp_err_t on failure
   */
  template <size_t N>
  std::expected<std::array<uint8_t, N>, esp_err_t> register_read(uint8_t reg_addr) {
    std::array<uint8_t, N> data{};
    esp_err_t ret = register_read_raw(reg_addr, data.data(), N);

    if (ret != ESP_OK) {
      return std::unexpected(ret);
    }

    return data;
  }

  /**
   * @brief Write to MPU6050 register
   * @param reg_addr Register address to write to
   * @param data Data byte to write
   * @return ESP_OK on success, error code on failure
   */
  esp_err_t register_write_byte(uint8_t reg_addr, uint8_t data) {
    uint8_t write_buf[2] = {reg_addr, data};

    return i2c_master_write_to_device(I2C_MASTER_NUM, MPU6050_ADDR, write_buf,
                                     sizeof(write_buf),
                                     I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
  }

  bool initialized;
};

