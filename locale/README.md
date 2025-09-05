# Locale Sensor

This is meant to be flashed onto an ESP board with the following components:

- ESP32s2
- Neo-M8N Gnss Module (UART)
- MPU6050 (I2C)

The sensor units have mocks so that the firmware can be tested inside Qemu.  The mocks simply are compiled in and return realistic looking data to see how the firmware will react.

## Setup
<details>
<summary>esp-idf install</summary>

0) Prereqs
### macOS (Homebrew):
`brew install cmake ninja dfu-util ccache git wget python3`

### Ubuntu/Debian:
`sudo apt-get update`

`sudo apt-get install -y git wget flex bison gperf python3 python3-pip python3-venv \
  cmake ninja-build ccache libffi-dev libssl-dev dfu-util libusb-1.0-0`

1) Get ESP-IDF (stable branch)

`mkdir -p ~/esp && cd ~/esp`

`git clone -b v5.5.1 --recursive https://github.com/espressif/esp-idf.git`

2) Install toolchains (choose your chips; here: esp32s2 only)

`cd ~/esp/esp-idf`

`./install.sh esp32s2`

3) Activate the ESP-IDF environment (run in every new shell)

`. $HOME/esp/esp-idf/export.sh`

(Optional) Add a convenience alias so you can type `get_idf` anytime:
`echo "alias get_idf='. \$HOME/esp/esp-idf/export.sh'" >> ~/.bashrc`   # or ~/.zshrc

4) Quick sanity check

`idf.py --version`

</details>

## Build

- ! You *MUST* have esp-idf extension installed
- ! You *MUST* source the esp environment with `. ~/<wherever esp is cloned>/export.sh`
- `idf.py build` will run the full build
- `idf.py qemu monitor` will run the esp32 in qemu and output to your terminal instead of uart