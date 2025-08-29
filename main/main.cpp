#include "hal/gpio_types.h"

extern "C" {
    #include "freertos/FreeRTOS.h"
    #include "freertos/task.h"
}

#include "mpu6050/imu.h"

extern "C" void app_main(void)
{
    ESP_LOGI(TAG, "Starting Squatch Sensor Package");

    // Initialize I2C
    ESP_ERROR_CHECK(i2c_master_init());
    ESP_LOGI(TAG, "I2C initialized successfully");

    // Initialize MPU6050
    ESP_ERROR_CHECK(mpu6050_init());

    int16_t accel_x, accel_y, accel_z;
    int16_t gyro_x, gyro_y, gyro_z;

    while (true) {
        esp_err_t ret = mpu6050_read_data(&accel_x, &accel_y, &accel_z, &gyro_x, &gyro_y, &gyro_z);
        
        if (ret == ESP_OK) {
            // Convert raw values to meaningful units
            float accel_x_g = accel_x / 16384.0f;
            float accel_y_g = accel_y / 16384.0f;
            float accel_z_g = accel_z / 16384.0f;
            
            float gyro_x_dps = gyro_x / 131.0f;
            float gyro_y_dps = gyro_y / 131.0f;
            float gyro_z_dps = gyro_z / 131.0f;

            ESP_LOGI(TAG, "Accel: X=%.2fg, Y=%.2fg, Z=%.2fg", accel_x_g, accel_y_g, accel_z_g);
            ESP_LOGI(TAG, "Gyro:  X=%.2f°/s, Y=%.2f°/s, Z=%.2f°/s", gyro_x_dps, gyro_y_dps, gyro_z_dps);
        } else {
            ESP_LOGE(TAG, "Failed to read IMU data: %s", esp_err_to_name(ret));
        }

        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}
