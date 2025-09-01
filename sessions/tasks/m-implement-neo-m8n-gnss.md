---
task: m-implement-neo-m8n-gnss
branch: feature/implement-neo-m8n-gnss
status: in-progress
created: 2025-08-31
modules: [main, gnss, wokwi-simulation]
---

# Implement NEO-M8N GNSS Module Support

## Problem/Goal
Add NEO-M8N GNSS module integration to the Squatch sensor package for GPS/location tracking capabilities. Since Wokwi doesn't have an off-the-shelf NEO-M8N part, we need to create a custom Wokwi part definition for simulation support alongside the hardware implementation.

## Success Criteria
- [ ] NEO-M8N hardware interface implemented (UART communication)
- [ ] GNSS data parsing (NMEA sentences - GGA, RMC, etc.)
- [ ] Integration with existing sensor data logging
- [ ] Custom Wokwi part definition created for NEO-M8N simulation
- [ ] Wokwi simulation validates GNSS functionality
- [ ] Hardware testing confirms GPS data acquisition
- [ ] Documentation updated with GNSS configuration

## Context Manifest

### How the Current Sensor System Works: IMU Integration Architecture

The Squatch project implements a highly structured, modern C++ sensor interface pattern using ESP-IDF on ESP32-S2 hardware. When the system boots, app_main() in `/Users/zach/src/squatch/main/main.cpp` initializes the I2C master interface using a static configuration pattern. The I2C initialization creates a master bus on GPIO pins 4 (SDA) and 5 (SCL) running at 400kHz with pull-up resistors enabled.

The IMU sensor integration follows a class-based architecture with the `IMU` class defined in `/Users/zach/src/squatch/main/imu/imu.h`. This class encapsulates all MPU6050 communication and data processing logic. The initialization process first validates the sensor by reading the WHO_AM_I register (0x75) and expecting a response of 0x68. If validation succeeds, it wakes up the MPU6050 by writing 0x00 to the power management register (0x6B). This two-phase initialization ensures sensor presence and proper power state before attempting data operations.

Data acquisition uses a bulk read strategy - the system reads 14 consecutive bytes starting from the accelerometer high byte register (0x3B). This single I2C transaction captures accelerometer X/Y/Z (6 bytes), temperature (2 bytes), and gyroscope X/Y/Z (6 bytes) data. The raw 16-bit values are then converted to meaningful units: accelerometer data is scaled by 16384 LSB/g (2g range), gyroscope by 131 LSB/°/s (250°/s range), and temperature using the formula (raw/340.0) + 36.53 for Celsius.

Critical to the architecture is the consistent use of `std::expected<T, esp_err_t>` for error handling throughout the sensor interface. Every operation that can fail returns either the expected result or an ESP-IDF error code. This eliminates exceptions and provides deterministic error propagation. The main loop demonstrates this pattern by checking each result and logging failures while continuing operation.

The hardware interface is abstracted through template functions like `register_read<N>()` which provide type-safe, compile-time sized operations. The `register_read_raw()` function handles the actual I2C communication using ESP-IDF's `i2c_master_write_read_device()` API with a 1-second timeout. This layered approach separates low-level I2C operations from high-level sensor logic.

### For GNSS Implementation: Integration Points and Required Modifications

Adding NEO-M8N GNSS support will require creating a parallel sensor module structure following the established patterns, but using UART communication instead of I2C. The main application loop currently runs a simple read-log cycle for IMU data at 1Hz. The GNSS integration will need to either interleave with this pattern or run on a separate task to handle the different data rates and communication protocols.

The NEO-M8N communicates via UART at 9600 baud (default) using NMEA 0183 sentences. Unlike the IMU's register-based interface, GNSS data arrives as ASCII sentences terminated with CRLF. Key sentences include GGA (position/fix data), RMC (recommended minimum), and others. The parsing logic will need to handle variable-length text parsing rather than fixed binary structures.

Communication setup will require configuring a UART instance (likely UART1 since UART0 is used for console). ESP32-S2 has 2 UART controllers, and we need to avoid conflicting with the console UART. The current system uses console on UART0 at 115200 baud. GPIO pin assignment will need coordination - typical NEO-M8N connections use TX/RX pins that don't conflict with existing I2C pins (4,5) or console pins.

Power management considerations include the fact that GNSS modules can consume significant power during acquisition. The NEO-M8N supports various power modes and can be configured for periodic GPS fixes to conserve battery. The implementation should consider startup time (cold start can take 30+ seconds) and provide configurable update rates.

Data integration will require extending the current logging pattern to include GNSS coordinates alongside IMU data. The structured data approach using dedicated structs (similar to `IMUData`, `AccelData`, `GyroData`) should be followed. A `GNSSData` structure should contain latitude, longitude, altitude, fix quality, satellite count, and timestamp information parsed from NMEA sentences.

### Wokwi Simulation Requirements: Custom Part Creation

Since Wokwi doesn't provide a NEO-M8N component, a custom part definition is required. The current simulation uses `diagram.json` with an ESP32-S2 DevKit and MPU6050 connected via I2C. The custom GNSS part will need to simulate UART communication and provide realistic NMEA sentence output.

The custom part definition requires creating a JSON specification that defines the part's visual appearance, pin connections, and behavioral simulation. Looking at the existing setup, the MPU6050 part connects to 3V3, GND, and the I2C pins. The NEO-M8N part will need similar power connections plus UART TX/RX pins. The simulation behavior should generate realistic NMEA sentences with configurable fix scenarios (no fix, 2D fix, 3D fix) and coordinate data.

The current Wokwi configuration in `wokwi.toml` references `build/squatch.bin` and `build/squathc.elf` (note: there's a typo in the ELF filename). The simulation will need to support UART communication patterns and provide meaningful GPS data for testing the parsing logic without requiring actual satellite signals.

### Technical Reference Details

#### Current Architecture Patterns to Follow

**Error Handling Pattern:**
```cpp
std::expected<DataType, esp_err_t> operation() {
    // Perform operation
    if (error_condition) {
        return std::unexpected(ESP_ERR_CODE);
    }
    return data_result;
}
```

**Structured Data Types:**
```cpp
struct GNSSData {
    struct Position {
        double latitude;
        double longitude;
        float altitude;
    } position;
    
    struct FixInfo {
        uint8_t quality;      // 0=no fix, 1=GPS fix, 2=DGPS fix
        uint8_t satellites;
        float hdop;           // Horizontal dilution of precision
    } fix;
    
    uint32_t timestamp;       // UTC timestamp from RMC
};
```

#### ESP-IDF UART Configuration Pattern

Based on the I2C initialization pattern, UART setup should follow similar conventions:

```cpp
#define GNSS_UART_NUM        UART_NUM_1
#define GNSS_UART_TXD_PIN    GPIO_NUM_17  // Avoid conflicts with I2C pins 4,5
#define GNSS_UART_RXD_PIN    GPIO_NUM_18
#define GNSS_UART_BAUD_RATE  9600
#define GNSS_UART_BUF_SIZE   1024
#define GNSS_UART_TIMEOUT_MS 1000
```

#### File Structure to Implement

Following the IMU module pattern:
- `/Users/zach/src/squatch/main/gnss/gnss.h` - Main GNSS interface class
- `/Users/zach/src/squatch/main/gnss/uart.h` - UART communication primitives  
- `/Users/zach/src/squatch/main/gnss/nmea.h` - NMEA sentence parsing utilities

#### Build System Integration

The `main/CMakeLists.txt` currently requires only the `driver` component. GNSS implementation will use the same driver component (for UART) but may need additional components if advanced features like threading are used. The C++23 standard is already configured at the project level.

#### Wokwi Custom Part Structure

Custom part definition will need:
- JSON part specification with pins, appearance
- Behavior simulation logic for NMEA generation
- Integration with existing diagram.json structure
- Connection patterns similar to existing MPU6050 setup

The simulation should provide test scenarios: stationary fix, moving coordinates, satellite acquisition sequences, and error conditions (no fix, poor signal) for comprehensive testing of the parsing and error handling logic.

## User Notes
- NEO-M8N communicates via UART (typically 9600 baud default)
- Need to handle NMEA 0183 sentence parsing
- Consider power management for GNSS module
- Wokwi custom part will require JSON definition and behavior simulation
- Should follow existing project patterns (std::expected error handling, structured data types)

## Work Log
<!-- Updated as work progresses -->
- [2025-08-31] Task created, need to research NEO-M8N interface and Wokwi custom parts