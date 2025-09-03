# Locale Sensor

This is meant to be flashed onto an ESP board with the following components:

- ESP32s2
- Neo-M8N Gnss Module (UART)
- MPU6050 (I2C)

The sensor units have mocks so that the firmware can be tested inside Qemu.  The mocks simply are compiled in and return realistic looking data to see how the firmware will react.