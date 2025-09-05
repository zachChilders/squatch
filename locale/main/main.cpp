extern "C" {
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
}

#include "tasks.h"

static const char *TAG = "SQUATCH";

extern "C" void app_main(void) {
  ESP_LOGI(TAG, "Starting Sensor Package with FreeRTOS Task Architecture");

  // Initialize sensor task system
  esp_err_t init_result = tasks::init_sensor_tasks();
  if (init_result != ESP_OK) {
    ESP_LOGE(TAG, "Failed to initialize sensor tasks: %s", 
             esp_err_to_name(init_result));
    return;
  }

  // Start all sensor tasks
  tasks::start_sensor_tasks();
  
  ESP_LOGI(TAG, "All sensor tasks started successfully");
  ESP_LOGI(TAG, "Main task entering idle loop");

  // Main task idle loop - tasks are now running independently
  while (true) {
    vTaskDelay(10000 / portTICK_PERIOD_MS); // 10 second idle delay
  }
}