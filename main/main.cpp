extern "C" {
#include "esp_log.h"
}

#include "imu/imu.h"
#include "gnss/gnss.h"

static const char *TAG = "SQUATCH";

extern "C" void app_main(void) {
  ESP_LOGI(TAG, "Starting Sensor Package");

  // Initialize I2C
  ESP_ERROR_CHECK(i2c_master_init());
  ESP_LOGI(TAG, "I2C initialized successfully");

  // Create and initialize GNSS module
  gnss::GNSSModule gnss_module;
  auto gnss_init_result = gnss_module.init();
  if (!gnss_init_result) {
    ESP_LOGE(TAG, "Failed to initialize GNSS: %s",
             esp_err_to_name(gnss_init_result.error()));
    return;
  }
  ESP_LOGI(TAG, "GNSS initialized successfully");

  // Create and initialize MPU6050 instance
  IMU imu;
  auto imu_init_result = imu.init(TAG);
  if (!imu_init_result) {
    ESP_LOGE(TAG, "Failed to initialize MPU6050: %s",
             esp_err_to_name(imu_init_result.error()));
    return;
  }

  while (true) {
    // Read IMU data
    auto imu_result = imu.read_imu_data();
    if (!imu_result) {
      ESP_LOGE(TAG, "Failed to read IMU data: %s",
               esp_err_to_name(imu_result.error()));
    } else {
      auto imu_data = imu_result.value();
      ESP_LOGI(TAG, "Accel: X=%.2fg, Y=%.2fg, Z=%.2fg", imu_data.accel.x,
               imu_data.accel.y, imu_data.accel.z);
      ESP_LOGI(TAG, "Gyro:  X=%.2f°/s, Y=%.2f°/s, Z=%.2f°/s", imu_data.gyro.x,
               imu_data.gyro.y, imu_data.gyro.z);
      ESP_LOGI(TAG, "Temp: %.2f°C", imu_data.temperature);
    }

    // Read GNSS data
    auto gnss_result = gnss_module.read();
    if (!gnss_result) {
      ESP_LOGE(TAG, "Failed to read GNSS data: %s",
               esp_err_to_name(gnss_result.error()));
    } else {
      auto gnss_data = gnss_result.value();
      if (gnss_data.fix_valid) {
        ESP_LOGI(TAG, "GPS: Lat=%.6f°, Lon=%.6f°, Alt=%.1fm, Sats=%d", 
                 gnss_data.latitude, gnss_data.longitude, 
                 gnss_data.altitude, gnss_data.satellites);
        ESP_LOGI(TAG, "GPS: Time=%06lu, Date=%06lu, HDOP=%.2f",
                 gnss_data.timestamp, gnss_data.date, gnss_data.hdop);
      } else {
        ESP_LOGI(TAG, "GPS: No valid fix (Sats=%d)", gnss_data.satellites);
      }
    }

    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}