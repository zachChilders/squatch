# Code Style and Conventions

## Language Standards
- **C++23** (CMAKE_CXX_STANDARD 23)
- Modern C++ features encouraged (std::expected, templates, structured bindings)

## Error Handling
- **Primary Pattern**: std::expected<T, esp_err_t> for functions that can fail
- Return std::unexpected(error_code) on failure
- Return value directly or empty {} on success
- ESP-IDF error checking with ESP_ERROR_CHECK() for critical failures

## Naming Conventions
- **Functions**: snake_case (e.g., `mpu6050_init`, `app_main`)
- **Variables**: snake_case (e.g., `imu_data`, `temp_raw`)  
- **Constants**: UPPER_CASE (e.g., `TAG`)
- **Structs**: PascalCase (e.g., `IMUData`, `AccelData`, `GyroData`)
- **Template Parameters**: PascalCase (e.g., `<size_t N>`)

## Documentation
- Doxygen-style comments for public functions
- @brief, @param, @return tags
- @tparam for template parameters
- Inline comments for complex calculations

## Code Organization  
- Header-only implementations for templates and small utilities
- Static functions for internal module functions
- Extern "C" wrappers for ESP-IDF C APIs
- Structured data types over primitive arrays

## ESP-IDF Integration
- Use ESP_LOGI, ESP_LOGE, ESP_ERROR_CHECK for logging and error handling
- TAG constants for log identification
- FreeRTOS task delays with vTaskDelay() and portTICK_PERIOD_MS