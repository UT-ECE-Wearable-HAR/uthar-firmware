# uthar-firmware

![CI](https://github.com/UT-ECE-Wearable-HAR/uthar-firmware/workflows/CI/badge.svg)

## Builds

You can find the latest builds on the releases tab,
download and unzip uthar.zip then proceed to the flash instructions

## Flash Instructions

Install [esptool](https://github.com/espressif/esptool) and pyserial
This can be installed through pip or through some package managers

Example flash command: change port to ex. COM1 on Windows

### Using esp.py wrapper

Using the esp.py wrapper you can flash, run and monitor \
usage: esp.py [-h] [--flash] [--run] port

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

- Install the `PyBluez` module
- Bluetooth server automatically connects to first device advertising SPP (UUID: 1101)
- On some systems the script may prompt you automatically to pair the first time to try to connect to the device.
If this does not occur please pair to the ESP manually. This usually only needs to be done once.
- Then run `python bluetooth_server.py`
