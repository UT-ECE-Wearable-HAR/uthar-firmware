# uthar-firmware

UT HAR Wearable firmware

## Flash Instructions

Install [esptool](https://github.com/espressif/esptool)
This can be installed through pip or through some package managers
such as pacman on Arch

Example flash command: change port to ex. COM1 on Windows

```
esptool.py --port /dev/ttyUSB0 write_flash 0x8000 build/partition_table/partition-table.bin 0x1000 build/bootloader/bootloader.bin 0x10000 build/uthar.bin
```

## Bluetooth Server

- Install the `PyBluez` module
- set the `ESP_MAC` environment variable to the bluetooth mac address of your esp device
- Then run `python bluetooth_server.py`
