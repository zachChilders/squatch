# Task Completion Checklist

Since this is an ESP32 embedded project, traditional linting/formatting tools may not be available or configured. However, here are the completion steps:

## Code Quality
1. **Build successfully** with `idf.py build` 
2. **No compilation warnings** - ESP-IDF toolchain will show warnings
3. **clangd static analysis** - VS Code integration should show issues
4. **Code follows project conventions** (see code_style_conventions.md)

## Testing
1. **Build test** - Ensure project compiles without errors
2. **Flash and monitor test** - If hardware is available, test with `idf.py flash monitor`
3. **Wokwi simulation test** - Use the configured wokwi.toml for simulation testing

## ESP32 Specific Checks
1. **Memory usage** - Check build output for memory allocation warnings
2. **Task stack sizes** - Ensure FreeRTOS tasks have adequate stack space
3. **I2C communication** - Verify I2C timing and error handling
4. **Power management** - Check if sleep modes are properly handled

## Code Review Points
1. **Error handling** - Verify std::expected usage is consistent
2. **Resource management** - Check for proper initialization/cleanup
3. **Hardware registers** - Verify register addresses and bit manipulations
4. **Sensor calibration** - Check conversion factors for accelerometer/gyroscope readings