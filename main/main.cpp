extern "C" {
    #include "esp_log.h"
}

#include "mpu6050/imu.h"

static const char *TAG = "SQUATCH";

extern "C" void app_main(void)
{
    ESP_LOGI(TAG, "Starting Sensor Package");

    // Initialize I2C
    ESP_ERROR_CHECK(i2c_master_init());
    ESP_LOGI(TAG, "I2C initialized successfully");

    // Initialize MPU6050
    auto init_result = mpu6050_init(TAG);
    if (!init_result) {
        ESP_LOGE(TAG, "Failed to initialize MPU6050: %s", esp_err_to_name(init_result.error()));
        return;
    }

    while (true) {
        auto result = mpu6050_read_imu_data();
        if (!result) {
            ESP_LOGE(TAG, "Failed to read IMU data: %s", esp_err_to_name(result.error()));
            continue;
        }
        
        auto imu_data = result.value();
        ESP_LOGI(TAG, "Accel: X=%.2fg, Y=%.2fg, Z=%.2fg", imu_data.accel.x, imu_data.accel.y, imu_data.accel.z);
        ESP_LOGI(TAG, "Gyro:  X=%.2f°/s, Y=%.2f°/s, Z=%.2f°/s", imu_data.gyro.x, imu_data.gyro.y, imu_data.gyro.z);
        ESP_LOGI(TAG, "Temp: %.2f°C", imu_data.temperature);

        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}
