extern "C" {
    #include "esp_err.h"
    #include "esp_log.h"
}

#include <array>
#include <expected>
#include "i2c.h"
#include "registers.h"

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

static esp_err_t mpu6050_register_read_raw(uint8_t reg_addr, uint8_t *data, size_t len) {
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
template<size_t N>
static std::expected<std::array<uint8_t, N>, esp_err_t> mpu6050_register_read(uint8_t reg_addr) {
  std::array<uint8_t, N> data{};
  esp_err_t ret = mpu6050_register_read_raw(reg_addr, data.data(), N);
  
  if (ret != ESP_OK) {
    return std::unexpected(ret);
  }
  
  return data;
}

/**
 * @brief Write to MPU6050 register
 */
static esp_err_t mpu6050_register_write_byte(uint8_t reg_addr, uint8_t data) {
  int ret;
  uint8_t write_buf[2] = {reg_addr, data};

  ret = i2c_master_write_to_device(I2C_MASTER_NUM, MPU6050_ADDR, write_buf,
                                   sizeof(write_buf),
                                   I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);

  return ret;
}

/**
 * @brief Read accelerometer and gyroscope data using std::expected
 * @return std::expected containing IMUData on success, or esp_err_t on failure
 */
static std::expected<IMUData, esp_err_t> mpu6050_read_imu_data() {
  // Read all sensor data in one go (14 bytes starting from accelerometer)
  auto result = mpu6050_register_read<14>(MPU6050_ACCEL_XOUT_H);
  if (!result) {
    return std::unexpected(result.error());
  }
  
  const auto& data = result.value();
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

/**
 * @brief Initialize MPU6050
 */
 static esp_err_t mpu6050_init(const char *TAG) {
    esp_err_t ret;
    uint8_t who_am_i;
  
    // Check WHO_AM_I register
    ret = mpu6050_register_read_raw(MPU6050_WHO_AM_I, &who_am_i, 1);
    if (ret != ESP_OK) {
      ESP_LOGE(TAG, "Failed to read WHO_AM_I register");
      return ret;
    }
  
    ESP_LOGI(TAG, "WHO_AM_I: 0x%02X", who_am_i);
  
    if (who_am_i != 0x68) {
      ESP_LOGE(TAG, "MPU6050 not found, WHO_AM_I: 0x%02X", who_am_i);
      return ESP_ERR_NOT_FOUND;
    }
  
    // Wake up MPU6050
    ret = mpu6050_register_write_byte(MPU6050_PWR_MGMT_1, 0x00);
    if (ret != ESP_OK) {
      ESP_LOGE(TAG, "Failed to wake up MPU6050");
      return ret;
    }
  
    ESP_LOGI(TAG, "MPU6050 initialized successfully");
    return ESP_OK;
  }