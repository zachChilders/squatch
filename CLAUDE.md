# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

**Squatch** is an ESP32-S2 based sensor package that reads IMU (MPU6050) data via I2C and continuously logs accelerometer, gyroscope, and temperature readings. The project uses modern C++23 with std::expected for error handling and is built using the ESP-IDF framework.

## Build Commands

```bash
# Build the project
idf.py build

# Flash to device
idf.py flash

# Monitor serial output  
idf.py monitor

# Flash and monitor in one command
idf.py flash monitor

# Clean build
idf.py clean

# Set target (if needed)
idf.py set-target esp32s2
```

Alternative make commands (if IDF_PATH is set):
```bash
make
make flash  
make monitor
```

## Architecture

The codebase follows a modular structure with header-only implementations:

- **main/main.cpp**: Application entry point with app_main() function
- **main/imu/**: MPU6050 sensor interface module
  - **imu.h**: Complete IMU interface with template functions and std::expected error handling
  - **registers.h**: MPU6050 register definitions
  - **i2c.h**: I2C communication primitives

Key architectural patterns:
- Modern C++ error handling with `std::expected<T, esp_err_t>`
- Template functions for type-safe register operations
- Structured data types (IMUData, AccelData, GyroData) over primitive arrays
- Header-only implementation for templates and utilities

## Code Conventions

- **Language**: C++23 standard
- **Error Handling**: std::expected pattern - return `std::unexpected(error_code)` on failure
- **Naming**: snake_case for functions/variables, PascalCase for structs, UPPER_CASE for constants
- **Documentation**: Doxygen-style comments (@brief, @param, @return)
- **ESP-IDF Integration**: Use ESP_LOGI/ESP_LOGE for logging, ESP_ERROR_CHECK for critical errors

## Development Environment

- **Target**: ESP32-S2 microcontroller
- **IDE**: VS Code with ESP-IDF extension
- **Language Server**: clangd configured for ESP32-S2 toolchain
- **Simulation**: Wokwi support via wokwi.toml configuration

## Task Completion

For any changes:
1. Build successfully with `idf.py build`
2. Check for compilation warnings
3. Verify clangd static analysis passes
4. Test with hardware (`idf.py flash monitor`) or Wokwi simulation if possible
5. Ensure consistent std::expected error handling patterns
6. Follow project naming and documentation conventions
## Sessions System Behaviors

@CLAUDE.sessions.md
