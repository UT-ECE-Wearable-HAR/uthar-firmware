# uthar-firmware

![CI](https://github.com/UT-ECE-Wearable-HAR/uthar-firmware/workflows/CI/badge.svg)

## Introduction

This firmware is meant to be used with the M5-Camera B board or equivalent.
The starter code and documentation for the M5-Camera can be found [here](https://github.com/m5stack/M5Stack-Camera)

## Builds

To build from source install [ESP-IDF](https://github.com/espressif/esp-idf) 3.3, and follow the relevant instructions to build this firmware.
This firmware does not support ESP-IDF versions after 3.3.

You can find the latest CI builds on the releases tab,
download and unzip uthar.zip then proceed to the flash instructions

## Flash Instructions

Install [esptool](https://github.com/espressif/esptool) and pyserial
This can be installed through pip or through some package managers

Example flash command: change port to ex. COM1 on Windows

### Using esp.py wrapper

Using the esp.py wrapper you can flash, run and monitor \
usage: esp.py [-h] [--flash] [--run] [--monitor] port

```bash
# firmware files should be in working directory
# flashing
python esp.py /dev/ttyUSB0 --flash --monitor

# running
python esp.py /dev/ttyUSB0 --run --monitor
```

### Using esptool

```bash
esptool.py --chip esp32 --port /dev/ttyUSB0 --baud 115200 --before default_reset --after hard_reset write_flash -z --flash_mode dio --flash_freq 40m --flash_size detect 0x1000 bootloader.bin 0x10000 uthar.bin 0x8000 partitions_singleapp.bin
```

Run command: `esptool.py --port /dev/ttyUSB0 run`

## Bluetooth Server

### Generate dmp Python module

To build from source install the `numpy`, then:

```bash
cd bindings
python setup.py build
```

The module can be used by a script when it's in the working directory
Available functions and types can be found using autocomplete
```python
import dmp
q = dmp.quaternion(packet)
```

### Running the Server

- Install the `PyBluez` module
- Bluetooth server automatically connects to first device advertising SPP (UUID: 1101)
- On some systems the script may prompt you automatically to pair the first time to try to connect to the device.
If this does not occur please pair to the ESP manually. This usually only needs to be done once.
- Then run `python bluetooth_server.py`
