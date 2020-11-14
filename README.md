# uthar-firmware
![CI](https://github.com/UT-ECE-Wearable-HAR/uthar-firmware/workflows/CI/badge.svg)

## Builds

You can find the latest builds on the releases tab,
download and unzip uthar.zip then proceed to the flash instructions

## Flash Instructions

Install [esptool](https://github.com/espressif/esptool)
This can be installed through pip or through some package managers

Example flash command: change port to ex. COM1 on Windows

```
esptool.py --port /dev/ttyUSB0 write_flash 0x8000 partition-table.bin 0x1000 bootloader.bin 0x10000 uthar.bin
```

## Bluetooth Server

- Install the `PyBluez` module
- set the `ESP_MAC` environment variable to the bluetooth mac address of your esp device
  - You can get the MAC address by running `esptool.py read_mac`
- make sure you have paired your system with the esp device, you should only need to do this once
- Then run `python bluetooth_server.py`
