# IMU Mock Integration

Software-based MPU6050 mock for testing and simulation without hardware.

## Usage

### Enable Mock Mode
Add to your ESP-IDF project configuration:

```cmake
# In CMakeLists.txt or via menuconfig
target_compile_definitions(${COMPONENT_LIB} PRIVATE CONFIG_IMU_MOCK)
```

Or set via environment/build flag:
```bash
export CPPFLAGS="$CPPFLAGS -DCONFIG_IMU_MOCK"
idf.py build
```

### Basic Usage
Your existing IMU code works unchanged:

```cpp
#include "imu/imu.h"

IMU imu;
auto result = imu.init("IMU");  // Works with mock data
auto data = imu.read_imu_data(); // Returns mock sensor values
```

### Control Mock Data

```cpp
#ifdef CONFIG_IMU_MOCK
#include "imu/mock_scenarios.h"

// Use predefined scenarios
MockScenarios::stationary();  // 1g on Z, no rotation
MockScenarios::tilted_45deg(); // 45° tilt
MockScenarios::rotating();     // 10°/s rotation
MockScenarios::hot();          // 85°C temperature

// Or set custom values
mock_set_accel_g(0.5f, 0.0f, 0.866f);    // Custom acceleration
mock_set_gyro_dps(1.0f, 2.0f, 0.0f);     // Custom rotation
mock_set_temperature_c(30.5f);           // Custom temperature
#endif
```

## Integration Points

- **Real Hardware**: Include `imu/i2c.h`, uses ESP-IDF I2C driver
- **Mock Mode**: Include `imu/mock.h`, provides simulated responses
- **Scenarios**: Include `imu/mock_scenarios.h` for predefined test cases

## File Structure

```
main/imu/
├── imu.h              # Main IMU class (unchanged)
├── i2c.h              # Real I2C implementation
├── mock.h             # Mock I2C implementation  
├── mock.cpp           # Mock data instance
├── mock_scenarios.h   # Predefined test scenarios
└── registers.h        # MPU6050 register definitions
```

## Testing Different Scenarios

The mock allows testing various conditions:

- **Stationary**: Normal 1g gravity, useful for calibration testing
- **Tilted**: Test orientation algorithms  
- **Rotating**: Test gyroscope integration
- **Hot/Cold**: Test temperature compensation
- **Zero-G**: Test edge case handling
- **Vibrating**: Test noise filtering

## Compilation

- **With Mock**: `#define CONFIG_IMU_MOCK` - No I2C hardware needed
- **Without Mock**: Normal build - Requires MPU6050 connected to I2C pins 4/5