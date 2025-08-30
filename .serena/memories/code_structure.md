# Code Structure

## Directory Layout
```
/
├── main/                    # Main application code
│   ├── main.cpp            # Application entry point (app_main)
│   ├── imu/                # IMU sensor module
│   │   ├── imu.h          # MPU6050 interface and data structures
│   │   ├── registers.h    # MPU6050 register definitions
│   │   └── i2c.h          # I2C communication interface
│   ├── i2c.h              # Top-level I2C header
│   └── CMakeLists.txt     # Component build configuration
├── CMakeLists.txt         # Main build configuration
├── Makefile               # ESP-IDF project makefile
├── wokwi.toml            # Wokwi simulator configuration
└── build/                # Build output directory (gitignored)
```

## Key Components
- **main.cpp**: Contains app_main() function that initializes I2C, initializes MPU6050, and runs continuous sensor reading loop
- **imu/imu.h**: Header-only implementation with template functions for MPU6050 communication, uses std::expected for error handling
- **I2C Interface**: Abstracts ESP-IDF I2C master operations

## Architecture Patterns
- Header-only implementation for IMU functionality
- Modern C++ error handling with std::expected instead of traditional error codes
- Template functions for type-safe register reading
- Structured data types for sensor readings