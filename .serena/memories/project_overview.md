# Project Overview

**Project Name:** Squatch  
**Purpose:** ESP32-S2 based sensor package that reads IMU (MPU6050) data and logs accelerometer, gyroscope, and temperature readings

**Tech Stack:**
- C++23 (set in CMakeLists.txt)
- ESP-IDF framework for ESP32-S2 microcontroller
- Wokwi simulator support
- CMake build system
- FreeRTOS (implicit with ESP-IDF)

**Hardware Target:**
- ESP32-S2 microcontroller
- MPU6050 IMU sensor connected via I2C
- Uses I2C communication for sensor data

**Key Features:**
- Modern C++ with std::expected for error handling
- Template-based register reading functions
- Structured sensor data types (IMUData, AccelData, GyroData)
- Continuous sensor data logging with 1-second intervals