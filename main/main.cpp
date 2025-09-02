extern "C" {
#include "esp_log.h"
}

#include "imu/imu.h"
#include "gnss/gnss.h"
#include "logging/sensor_logger.h"

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

  // Create sensor logger with dual output mode
  logging::SensorLogger logger(logging::SensorLogger::OutputMode::DUAL_OUTPUT);

  while (true) {
    // Read IMU data
    auto imu_result = imu.read_imu_data();
    if (!imu_result) {
      ESP_LOGE(TAG, "Failed to read IMU data: %s",
               esp_err_to_name(imu_result.error()));
    } else {
      logger.log_imu_data(imu_result.value());
    }

    // Read GNSS data
    auto gnss_result = gnss_module.read();
    if (!gnss_result) {
      ESP_LOGE(TAG, "Failed to read GNSS data: %s",
               esp_err_to_name(gnss_result.error()));
    } else {
      logger.log_gnss_data(gnss_result.value());
    }

    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}