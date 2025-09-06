# Squatch

An exploration into simulation, sensor fusion, and SLAM.

![Squatch Logo](static/logo.png)

## Locale
An esp32 firmware designed to poll data from an IMU and GNSS module.  It logs json messages to uart (for now).  esp-idf and freertos based.

## Brain
A dockerized ros2 setup.  Includes a node that reads uart messages, parses them, then inserts them into topics for processing.

## Setup
- esp-idf tooling, install via vscode
- [esp-idf qemu](https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-guides/tools/qemu.html)
- docker & docker compose

## Development Workflow

### Quick Start
```bash
# Start complete ESP-QEMU-ROS2 integration
./scripts/dev.sh start-all

# Monitor system status
./scripts/dev.sh status

# View ROS2 sensor topics
./scripts/dev.sh ros-topics

# Access ROS2 container for debugging  
./scripts/dev.sh ros-shell

# Clean shutdown of all systems
./scripts/dev.sh stop-all
```

### Development Commands
```bash
# Integrated commands (recommended)
./scripts/dev.sh start-all    # Complete system startup
./scripts/dev.sh stop-all     # Clean shutdown  
./scripts/dev.sh status       # System health check

# Individual ESP32 control
./scripts/dev.sh esp-start    # Start QEMU only
./scripts/dev.sh esp-stop     # Stop QEMU only
./scripts/dev.sh esp-status   # Check QEMU status

# Individual ROS2 control  
./scripts/dev.sh ros-build    # Build container
./scripts/dev.sh ros-up       # Start container
./scripts/dev.sh ros-build-ws # Build workspace
./scripts/dev.sh ros-bridge   # Run ESP bridge
./scripts/dev.sh ros-topics   # List topics
./scripts/dev.sh ros-shell    # Container shell
./scripts/dev.sh ros-down     # Stop container
```

Run `./scripts/dev.sh help` for complete command reference.

## Legacy Integration Testing (Manual Process)

For granular control, the original scripts remain functional:

```bash
# Terminal 1: ESP32 side
./scripts/qemu.sh start

# Terminal 2: ROS2 side  
cd brain && ./scripts/dev.sh up
./scripts/dev.sh build-ws
./scripts/dev.sh esp-bridge

# Monitor data
./scripts/dev.sh shell
ros2 topic echo /squatch/gnss
```

## Architecture Overview

**ESP32 QEMU**: Runs sensor firmware with JSON output via TCP port 5555
**ROS2 Brain**: Docker container with ESP Bridge Node consuming TCP data  
**Integration**: TCP port 5555 coordinates data flow from ESP32 to ROS2

## TODO:

- General cleanup getting code up to standards
- ros2 services to consume data
- Telemetry module
- Switch to canbus
