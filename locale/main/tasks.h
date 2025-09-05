#pragma once

extern "C" {
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
}

#include "imu/imu.h"
#include "gnss/gnss.h"
#include "logging/structured_logger.h"

namespace tasks {

// Task configuration
constexpr size_t STACK_SIZE_KB = 4;
constexpr size_t STACK_SIZE = STACK_SIZE_KB * 1024;
constexpr UBaseType_t TASK_PRIORITY_LOW = 1;
constexpr UBaseType_t TASK_PRIORITY_NORMAL = 2;
constexpr UBaseType_t TASK_PRIORITY_HIGH = 3;

// Queue configuration  
constexpr UBaseType_t QUEUE_LENGTH = 10;
constexpr TickType_t QUEUE_SEND_TIMEOUT_MS = 100 / portTICK_PERIOD_MS;
constexpr TickType_t QUEUE_RECEIVE_TIMEOUT_MS = 1000 / portTICK_PERIOD_MS;

// Task timing configuration
constexpr TickType_t IMU_POLL_INTERVAL_MS = 100 / portTICK_PERIOD_MS;  // 10Hz
constexpr TickType_t GNSS_POLL_INTERVAL_MS = 1000 / portTICK_PERIOD_MS; // 1Hz  
constexpr TickType_t LOGGING_POLL_INTERVAL_MS = 1000 / portTICK_PERIOD_MS; // 1Hz

// Message types for queue communication
enum class MessageType : uint8_t {
    IMU_DATA = 1,
    GNSS_DATA = 2
};

// Queue message structure using std::variant instead of union to handle non-trivial types
struct SensorMessage {
    MessageType type;
    uint64_t timestamp_us;
    IMUData imu_data;
    gnss::GNSSData gnss_data;
    
    // Default constructor
    SensorMessage() : type(MessageType::IMU_DATA), timestamp_us(0) {}
    
    // Constructor for IMU data
    SensorMessage(const IMUData& imu) : type(MessageType::IMU_DATA), imu_data(imu) {
        timestamp_us = esp_timer_get_time();
    }
    
    // Constructor for GNSS data  
    SensorMessage(const gnss::GNSSData& gnss) : type(MessageType::GNSS_DATA), gnss_data(gnss) {
        timestamp_us = esp_timer_get_time();
    }
};

// Global queue handles (initialized in main.cpp)
extern QueueHandle_t sensor_data_queue;

// Task function prototypes
void imu_task(void* parameters);
void gnss_task(void* parameters);
void logging_task(void* parameters);

// Task initialization
esp_err_t init_sensor_tasks();
void start_sensor_tasks();

} // namespace tasks