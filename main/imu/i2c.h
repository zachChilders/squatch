extern "C" {
#include "driver/i2c.h"
#include "esp_err.h"
#include "esp_log.h"
}

// I2C Configuration
#define I2C_MASTER_SCL_IO 5 /*!< GPIO number used for I2C master clock */
#define I2C_MASTER_SDA_IO 4 /*!< GPIO number used for I2C master data  */
#define I2C_MASTER_NUM                                                         \
  I2C_NUM_0 /*!< I2C master i2c port number, the number of i2c peripheral      \
               interfaces available will depend on the chip */
#define I2C_MASTER_FREQ_HZ 400000   /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE 0 /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE 0 /*!< I2C master doesn't need buffer */
#define I2C_MASTER_TIMEOUT_MS 1000

/**
 * @brief Initialize I2C master
 */
static esp_err_t i2c_master_init(void) {
  i2c_config_t conf = {
      .mode = I2C_MODE_MASTER,
      .sda_io_num = I2C_MASTER_SDA_IO,
      .scl_io_num = I2C_MASTER_SCL_IO,
      .sda_pullup_en = GPIO_PULLUP_ENABLE,
      .scl_pullup_en = GPIO_PULLUP_ENABLE,
      .master =
          {
              .clk_speed = I2C_MASTER_FREQ_HZ,
          },
      .clk_flags = 0,
  };

  esp_err_t err = i2c_param_config(I2C_MASTER_NUM, &conf);
  if (err != ESP_OK) {
    return err;
  }

  return i2c_driver_install(I2C_MASTER_NUM, conf.mode,
                            I2C_MASTER_RX_BUF_DISABLE,
                            I2C_MASTER_TX_BUF_DISABLE, 0);
}
