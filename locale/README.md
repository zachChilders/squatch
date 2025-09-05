# Locale Sensor

This is meant to be flashed onto an ESP board with the following components:

- ESP32s2
- Neo-M8N Gnss Module (UART)
- MPU6050 (I2C)

The sensor units have mocks so that the firmware can be tested inside Qemu.  The mocks simply are compiled in and return realistic looking data to see how the firmware will react.

## Setup
- ! You *MUST* have esp-idf extension installed
- ! You *MUST* source the esp environment with `. ~/<wherever esp is cloned>/export.sh`
- `idf.py build` will run the full build
- `idf.py qemu monitor` will run the esp32 in qemu and output to your terminal instead of uart