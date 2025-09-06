# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

**Squatch** is a dual-component system for sensor fusion and SLAM exploration:

- **Locale**: ESP32 firmware that polls IMU and GNSS sensors, outputting JSON messages via UART/TCP
- **Brain**: Dockerized ROS2 system that processes sensor data from the ESP32 via TCP bridge

The project combines ESP-IDF/FreeRTOS firmware with ROS2 for integrated sensor data processing and uses QEMU for development/testing without physical hardware.

## Build Commands

### Integrated Development Workflow (Recommended)
```bash
# Complete system startup - ESP32 QEMU + ROS2
./scripts/dev.sh start-all

# System status monitoring  
./scripts/dev.sh status

# View ROS2 sensor data
./scripts/dev.sh ros-topics

# Clean shutdown
./scripts/dev.sh stop-all

# Help for all available commands
./scripts/dev.sh help
```

### ESP32 Firmware (locale/)
```bash
cd locale/

# Build firmware
idf.py build

# Flash to hardware
idf.py flash

# Serial monitoring
idf.py monitor

# QEMU simulation
idf.py qemu
```

### ROS2 Brain System
```bash
# Individual ROS2 control
./scripts/dev.sh ros-build    # Build container
./scripts/dev.sh ros-up       # Start container  
./scripts/dev.sh ros-bridge   # Run ESP bridge
./scripts/dev.sh ros-shell    # Access container
```

## Architecture

### System Components
- **ESP32 Firmware** (`locale/`): C++23 firmware with IMU/GNSS sensor polling
- **ROS2 Brain** (`brain/`): Docker container with ESP Bridge Node
- **TCP Integration**: Port 5555 connects ESP32 QEMU to ROS2 bridge
- **Development Scripts**: Unified workflow via `scripts/dev.sh`

### Code Structure  
```
locale/main/           # ESP32 application code
├── main.cpp          # Application entry point  
├── imu/              # MPU6050 IMU interface
├── gnss/             # GNSS module interface
├── lib/              # Shared libraries (UART, I2C)
└── logging/          # Structured JSON logging

brain/ros2_ws/src/squatch_nodes/  # ROS2 integration
├── src/              # ESP bridge implementation
└── include/          # ROS2 node headers
```

## Code Conventions

- **Language**: C++23 standard with ESP-IDF framework
- **Error Handling**: `std::expected<T, esp_err_t>` pattern throughout
- **Naming**: snake_case functions/variables, PascalCase structs, UPPER_CASE constants  
- **Documentation**: Doxygen-style comments (@brief, @param, @return)
- **Logging**: ESP_LOGI/ESP_LOGE for ESP32, structured JSON output to TCP
- **Architecture**: Header-only implementations for templates, modular sensor interfaces

## Development Environment

- **ESP32 Target**: Configurable (ESP32-S2 default)
- **Simulation**: QEMU via ESP-IDF with TCP serial output
- **ROS2**: Dockerized environment with sensor topic processing
- **Integration**: TCP port 5555 for ESP32 ↔ ROS2 data flow
- **IDE**: VS Code with ESP-IDF extension, clangd language server

## Task Completion

For any changes:
1. Use `./scripts/dev.sh start-all` for integrated testing
2. Verify ESP32 firmware builds: `cd locale && idf.py build`
3. Check ROS2 integration: `./scripts/dev.sh ros-topics`
4. Monitor system status: `./scripts/dev.sh status`  
5. Ensure proper JSON message format for ESP→ROS2 communication
6. Follow std::expected error handling patterns in ESP32 code
7. Test with QEMU simulation before hardware deployment