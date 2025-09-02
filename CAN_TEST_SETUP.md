# CAN Bus Testing Setup

## Virtual CAN Interface Setup (Linux)

### 1. Load virtual CAN module and create interface
```bash
sudo modprobe vcan
sudo ip link add dev vcan0 type vcan
sudo ip link set up vcan0
```

### 2. Install CAN utilities
```bash
sudo apt install can-utils
```

## Testing CAN Messages

### 3. Monitor all CAN traffic
```bash
# In one terminal - monitor all messages
candump vcan0
```

### 4. Simulate ESP32 sensor messages
```bash
# IMU Accelerometer (ID 0x100) - X=1.2g, Y=0.8g
cansend vcan0 100#3F99999A3F4CCCCD

# IMU Gyroscope (ID 0x101) - X=45°/s, Y=-30°/s  
cansend vcan0 101#42340000C1F00000

# IMU Temperature (ID 0x102) - 25.5°C
cansend vcan0 102#41CC0000

# GNSS Position (ID 0x200) - Lat=37.7749, Lon=-122.4194
cansend vcan0 200#4217020EC2F46B8B

# GNSS Status (ID 0x201) - Alt=100m, 8 sats, valid fix, HDOP=1.2
cansend vcan0 201#42C8000008014C00

# Heartbeat (ID 0x7FF)
cansend vcan0 7FF#5351554154434801
```

## ROS2 CAN Bridge Setup

### 5. Install ROS2 CAN packages
```bash
sudo apt install ros-humble-socketcan-bridge ros-humble-can-msgs
```

### 6. Start CAN to ROS2 bridge
```bash
# Bridge vcan0 to ROS2 topics
ros2 run socketcan_bridge socketcan_bridge_node --ros-args -p interface:=vcan0
```

### 7. Monitor ROS2 topics
```bash
# List available topics
ros2 topic list

# Monitor CAN messages as ROS2 topics
ros2 topic echo /from_can_bus

# Send ROS2 messages to CAN
ros2 topic pub /to_can_bus can_msgs/Frame "{id: 0x100, data: [0x3F,0x99,0x99,0x9A,0x3F,0x4C,0xCC,0xCD]}"
```

## CAN Message Format

| CAN ID | Data Format | Description |
|--------|-------------|-------------|
| 0x100  | float x, float y | IMU Accel X,Y (g) |
| 0x101  | float x, float y | IMU Gyro X,Y (°/s) |
| 0x102  | float temp   | IMU Temperature (°C) |
| 0x200  | double lat, double lon | GNSS Position |
| 0x201  | float alt, uint8 sats, uint8 valid, uint16 hdop | GNSS Status |
| 0x7FF  | "SQUATCH\x01" | Heartbeat |

## Safety Monitoring

### 8. Watchdog monitoring example
```bash
# Monitor heartbeat messages (should see every ~1 second)
candump vcan0 | grep "7FF"

# Detect missing messages (timeout detection)
timeout 5s candump vcan0 | grep "100" || echo "IMU TIMEOUT!"
```

## Testing Workflow

1. **Start virtual CAN**: Create vcan0 interface
2. **Monitor traffic**: Run candump in background  
3. **Simulate ESP32**: Send test messages with cansend
4. **Bridge to ROS2**: Start socketcan_bridge_node
5. **Verify ROS2**: Check topics with ros2 topic echo
6. **Test safety**: Simulate timeouts and failures