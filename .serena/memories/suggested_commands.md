# Suggested Commands

## ESP-IDF Commands (requires ESP-IDF environment setup)
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

# Full clean (including sdkconfig)
idf.py fullclean

# Build for specific target (ESP32-S2)
idf.py set-target esp32s2
idf.py build

# Configuration menu
idf.py menuconfig
```

## Alternative Make Commands
```bash
# Build using traditional makefile (if IDF_PATH is set)
make

# Flash using make
make flash

# Monitor using make  
make monitor
```

## Wokwi Simulation
```bash
# The project includes wokwi.toml for simulation
# Build first, then use Wokwi CLI or web interface
# Firmware file: build/squatch.bin
```

## Development Environment
- VS Code with ESP-IDF extension recommended
- clangd configured for ESP32-S2 toolchain
- Target device: ESP32-S2
- Flash via UART (configured in .vscode/settings.json)