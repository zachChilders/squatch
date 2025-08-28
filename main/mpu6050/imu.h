extern "C" {
    #include "driver/i2c.h"
    #include "esp_err.h"
    #include "esp_log.h"
    #include "freertos/FreeRTOS.h"
    #include "freertos/task.h"
}

static const char *TAG = "SQUATCH";

// MPU6050 I2C Configuration
#define I2C_MASTER_SCL_IO           5        /*!< GPIO number used for I2C master clock */
#define I2C_MASTER_SDA_IO           4        /*!< GPIO number used for I2C master data  */
#define I2C_MASTER_NUM              I2C_NUM_0        /*!< I2C master i2c port number, the number of i2c peripheral interfaces available will depend on the chip */
#define I2C_MASTER_FREQ_HZ          400000   /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE   0        /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE   0        /*!< I2C master doesn't need buffer */
#define I2C_MASTER_TIMEOUT_MS       1000


// MPU6050 Register Addresses
#define MPU6050_ADDR                0x68
#define MPU6050_WHO_AM_I            0x75
#define MPU6050_PWR_MGMT_1          0x6B
#define MPU6050_ACCEL_XOUT_H        0x3B
#define MPU6050_GYRO_XOUT_H         0x43


/**
 * @brief Initialize I2C master
 */
 static esp_err_t i2c_master_init(void)
 {   
     i2c_config_t conf = {
         .mode = I2C_MODE_MASTER,
         .sda_io_num = I2C_MASTER_SDA_IO,
         .scl_io_num = I2C_MASTER_SCL_IO,
         .sda_pullup_en = GPIO_PULLUP_ENABLE,
         .scl_pullup_en = GPIO_PULLUP_ENABLE,
         .master = {
             .clk_speed = I2C_MASTER_FREQ_HZ,
         },
         .clk_flags = 0,
     };
 
     esp_err_t err = i2c_param_config(I2C_MASTER_NUM, &conf);
     if (err != ESP_OK) {
         return err;
     }
 
     return i2c_driver_install(I2C_MASTER_NUM, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
 }
 


/**
 * @brief Read from MPU6050 register
 */
 static esp_err_t mpu6050_register_read(uint8_t reg_addr, uint8_t *data, size_t len)
 {
     return i2c_master_write_read_device(I2C_MASTER_NUM, MPU6050_ADDR, &reg_addr, 1, data, len, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
 }
 
 /**
 * @brief Write to MPU6050 register
 */
static esp_err_t mpu6050_register_write_byte(uint8_t reg_addr, uint8_t data)
{
    int ret;
    uint8_t write_buf[2] = {reg_addr, data};

    ret = i2c_master_write_to_device(I2C_MASTER_NUM, MPU6050_ADDR, write_buf, sizeof(write_buf), I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);

    return ret;
}


/**
 * @brief Initialize MPU6050
 */
 static esp_err_t mpu6050_init(void)
 {
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
 
 /**
  * @brief Read accelerometer and gyroscope data
  */
 static esp_err_t mpu6050_read_data(int16_t *accel_x, int16_t *accel_y, int16_t *accel_z, 
                                    int16_t *gyro_x, int16_t *gyro_y, int16_t *gyro_z)
 {
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