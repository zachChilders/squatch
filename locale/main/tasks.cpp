#include "tasks.h"

namespace tasks {

static const char *TAG = "TASKS";

// Global queue handle
QueueHandle_t sensor_data_queue = nullptr;

// Global sensor instances (thread-safe as each task owns its instance)

static std::unique_ptr<IMU> imu_instance = std::make_unique<IMU>();
static std::unique_ptr<gnss::GNSSModule> gnss_instance = std::make_unique<gnss::GNSSModule>();
static std::unique_ptr<logging::StructuredLogger> logger_instance = std::make_unique<logging::StructuredLogger>();

/**
 * @brief IMU polling task - reads MPU6050 at configurable interval
 */
void imu_task(void* parameters) {
    ESP_LOGI(TAG, "IMU task started");
    
    IMUData imu_data{};
    
    while (true) {
        // Read IMU data
        auto imu_result = imu_instance->read_imu_data();
        if (!imu_result) {
            ESP_LOGE(TAG, "IMU task: Failed to read IMU data: %s",
                     esp_err_to_name(imu_result.error()));
            // Use default/invalid data but continue
        } else {
            imu_data = imu_result.value();
        }
        
        // Send data to logging task via queue
        SensorMessage message(imu_data);
        BaseType_t queue_result = xQueueSend(sensor_data_queue, &message, QUEUE_SEND_TIMEOUT_MS);
        if (queue_result != pdPASS) {
            ESP_LOGW(TAG, "IMU task: Failed to send data to queue (queue full?)");
        }
        
        vTaskDelay(IMU_POLL_INTERVAL_MS);
    }
}

/**
 * @brief GNSS polling task - reads NEO-M8N at configurable interval  
 */
void gnss_task(void* parameters) {
    ESP_LOGI(TAG, "GNSS task started");
    
    gnss::GNSSData gnss_data{};
    
    while (true) {
        // Read GNSS data
        auto gnss_result = gnss_instance->read();
        if (!gnss_result) {
            ESP_LOGE(TAG, "GNSS task: Failed to read GNSS data: %s", 
                     esp_err_to_name(gnss_result.error()));
            // Use default/invalid data but continue
        } else {
            gnss_data = gnss_result.value();
        }
        
        // Send data to logging task via queue
        SensorMessage message(gnss_data);
        BaseType_t queue_result = xQueueSend(sensor_data_queue, &message, QUEUE_SEND_TIMEOUT_MS);
        if (queue_result != pdPASS) {
            ESP_LOGW(TAG, "GNSS task: Failed to send data to queue (queue full?)");
        }
        
        vTaskDelay(GNSS_POLL_INTERVAL_MS);
    }
}

/**
 * @brief Logging task - receives sensor data from queue and outputs JSON
 */
void logging_task(void* parameters) {
    ESP_LOGI(TAG, "Logging task started");
    
    SensorMessage message;
    IMUData latest_imu_data{};
    gnss::GNSSData latest_gnss_data{};
    
    TickType_t last_log_time = xTaskGetTickCount();
    
    while (true) {
        // Check for incoming sensor data
        BaseType_t queue_result = xQueueReceive(sensor_data_queue, &message, QUEUE_RECEIVE_TIMEOUT_MS);
        
        if (queue_result == pdPASS) {
            // Update latest sensor data based on message type
            switch (message.type) {
                case MessageType::IMU_DATA:
                    latest_imu_data = message.imu_data;
                    break;
                    
                case MessageType::GNSS_DATA:
                    latest_gnss_data = message.gnss_data;
                    break;
                    
                default:
                    ESP_LOGW(TAG, "Logging task: Unknown message type: %d", 
                             static_cast<int>(message.type));
                    break;
            }
        }
        
        // Check if it's time to output JSON (1Hz regardless of queue activity)
        TickType_t current_time = xTaskGetTickCount();
        if ((current_time - last_log_time) >= LOGGING_POLL_INTERVAL_MS) {
            
            // Create structured sensor data models
            logging::IMUModel imu_model = logging::create_imu_model(latest_imu_data);
            logging::GNSSModel gnss_model = logging::create_gnss_model(latest_gnss_data);
            logging::SensorPacket packet(imu_model, gnss_model);
            
            // Log to UART0/stdout
            auto log_result = logger_instance->log(packet);
            if (!log_result) {
                ESP_LOGE(TAG, "Logging task: Failed to log sensor data: %s",
                         esp_err_to_name(log_result.error()));
            }
            
            last_log_time = current_time;
        }
    }
}

/**
 * @brief Initialize sensor task system - creates queue and initializes sensors
 */
esp_err_t init_sensor_tasks() {
    ESP_LOGI(TAG, "Initializing sensor tasks");
    
    // Create sensor data queue
    sensor_data_queue = xQueueCreate(QUEUE_LENGTH, sizeof(SensorMessage));
    if (sensor_data_queue == nullptr) {
        ESP_LOGE(TAG, "Failed to create sensor data queue");
        return ESP_ERR_NO_MEM;
    }
    
    // Initialize I2C for IMU
    esp_err_t i2c_result = i2c_master_init();
    if (i2c_result != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize I2C: %s", esp_err_to_name(i2c_result));
        return i2c_result;
    }
    ESP_LOGI(TAG, "I2C initialized successfully");
    
    // Create and initialize IMU instance
    auto imu_init_result = imu_instance->init(TAG);
    if (!imu_init_result) {
        ESP_LOGE(TAG, "Failed to initialize MPU6050: %s",
                 esp_err_to_name(imu_init_result.error()));
        return imu_init_result.error();
    }
    
    // Create and initialize GNSS instance
    auto gnss_init_result = gnss_instance->init();
    if (!gnss_init_result) {
        ESP_LOGE(TAG, "Failed to initialize GNSS: %s",
                 esp_err_to_name(gnss_init_result.error()));
        return gnss_init_result.error();
    }
    ESP_LOGI(TAG, "GNSS initialized successfully");
    
    // Create and initialize structured logger
    logger_instance = std::make_unique<logging::StructuredLogger>();
    auto logger_init_result = logger_instance->init();
    if (!logger_init_result) {
        ESP_LOGE(TAG, "Failed to initialize logger: %s",
                 esp_err_to_name(logger_init_result.error()));
        return logger_init_result.error();
    }
    
    ESP_LOGI(TAG, "All sensors initialized successfully");
    return ESP_OK;
}

/**
 * @brief Start all sensor tasks
 */
void start_sensor_tasks() {
    ESP_LOGI(TAG, "Starting sensor tasks");
    
    // Create IMU task
    BaseType_t imu_task_result = xTaskCreate(
        imu_task,
        "imu_task", 
        STACK_SIZE,
        nullptr,
        TASK_PRIORITY_NORMAL,
        nullptr
    );
    
    if (imu_task_result != pdPASS) {
        ESP_LOGE(TAG, "Failed to create IMU task");
        return;
    }
    
    // Create GNSS task  
    BaseType_t gnss_task_result = xTaskCreate(
        gnss_task,
        "gnss_task",
        STACK_SIZE, 
        nullptr,
        TASK_PRIORITY_NORMAL,
        nullptr
    );
    
    if (gnss_task_result != pdPASS) {
        ESP_LOGE(TAG, "Failed to create GNSS task");
        return;
    }
    
    // Create logging task
    BaseType_t logging_task_result = xTaskCreate(
        logging_task,
        "logging_task",
        STACK_SIZE,
        nullptr, 
        TASK_PRIORITY_HIGH,  // Higher priority to handle queue processing
        nullptr
    );
    
    if (logging_task_result != pdPASS) {
        ESP_LOGE(TAG, "Failed to create logging task");
        return;
    }
    
    ESP_LOGI(TAG, "All sensor tasks created successfully");
}

} // namespace tasks