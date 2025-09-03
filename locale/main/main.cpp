extern "C" {
#include "esp_log.h"
}

#include "imu/imu.h"
#include "gnss/gnss.h"

static const char *TAG = "SQUATCH";
#include "logging/structured_logger.h"

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

  // Initialize structured logger
  logging::StructuredLogger logger;
  auto logger_init_result = logger.init();
  if (!logger_init_result) {
    ESP_LOGE(TAG, "Failed to initialize logger: %s",
             esp_err_to_name(logger_init_result.error()));
    return;
  }

  while (true) {
    // Read IMU data
    IMUData imu_data{};
    bool imu_valid = false;
    auto imu_result = imu.read_imu_data();
    if (!imu_result) {
      ESP_LOGE(TAG, "Failed to read IMU data: %s",
               esp_err_to_name(imu_result.error()));
    } else {
      imu_data = imu_result.value();
      imu_valid = true;
    }

    // Read GNSS data
    gnss::GNSSData gnss_data{};
    bool gnss_valid = false;
    auto gnss_result = gnss_module.read();
    if (!gnss_result) {
      ESP_LOGE(TAG, "Failed to read GNSS data: %s",
               esp_err_to_name(gnss_result.error()));
    } else {
      gnss_data = gnss_result.value();
      gnss_valid = true;
    }

    // Log structured sensor data
    logging::IMUModel imu_model = logging::create_imu_model(imu_data, imu_valid);
    logging::GNSSModel gnss_model = logging::create_gnss_model(gnss_data, gnss_valid);
    logging::SensorPacket packet(imu_model, gnss_model);
    auto log_result = logger.log(packet);
    if (!log_result) {
      ESP_LOGE(TAG, "Failed to log sensor data: %s",
               esp_err_to_name(log_result.error()));
    }

    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}