extern "C" {
    #include "esp_err.h"
    #include "esp_log.h"
}

#include "i2c.h"
#include "registers.h"

struct IMUData {
  struct AccelData {
    int16_t x;
    int16_t y;
    int16_t z;
  };
  struct GyroData {
    int16_t gyro_x;
    int16_t y;
    int16_t z;
  };
  struct TemperatureData {
    int16_t temperature;
  };
};


/**
 * @brief Read from MPU6050 register
 */
static esp_err_t mpu6050_register_read(uint8_t reg_addr, uint8_t *data,
                                       size_t len) {
  return i2c_master_write_read_device(
      I2C_MASTER_NUM, MPU6050_ADDR, &reg_addr, 1, data, len,
      I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
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
 * @brief Read accelerometer and gyroscope data
 */
static esp_err_t mpu6050_read_data(int16_t *accel_x, int16_t *accel_y,
                                   int16_t *accel_z, int16_t *gyro_x,
                                   int16_t *gyro_y, int16_t *gyro_z) {
  uint8_t data[14];
  esp_err_t ret;

  // Read all sensor data in one go
  ret = mpu6050_register_read(MPU6050_ACCEL_XOUT_H, data, 14);
  if (ret != ESP_OK) {
    return ret;
  }

  // Parse accelerometer data (2g range, 16384 LSB/g)
  *accel_x = static_cast<int16_t>((data[0] << 8) | data[1]);
  *accel_y = static_cast<int16_t>((data[2] << 8) | data[3]);
  *accel_z = static_cast<int16_t>((data[4] << 8) | data[5]);

  // Skip temperature data (data[6], data[7])

  // Parse gyroscope data (250°/s range, 131 LSB/°/s)
  *gyro_x = static_cast<int16_t>((data[8] << 8) | data[9]);
  *gyro_y = static_cast<int16_t>((data[10] << 8) | data[11]);
  *gyro_z = static_cast<int16_t>((data[12] << 8) | data[13]);

  return ESP_OK;
}

/**
 * @brief Initialize MPU6050
 */
 static esp_err_t mpu6050_init(const char *TAG) {
    esp_err_t ret;
    uint8_t who_am_i;
  
    // Check WHO_AM_I register
    ret = mpu6050_register_read(MPU6050_WHO_AM_I, &who_am_i, 1);
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