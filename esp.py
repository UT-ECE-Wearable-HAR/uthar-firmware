#!/usr/bin/env python3

import sys
import esptool
from serial.tools import miniterm
import serial
import argparse

"""
wrapper for esptool, also lets you see serial output
usage: esp.py [-h] [--flash] [--run] [--monitor] port
dependecies: pyserial, esptool, python 3
"""

parser = argparse.ArgumentParser(description="esptool wrapper")
parser.add_argument("port", type=str, help="/dev/ttyUSB0")
parser.add_argument("--flash", action="store_true", help="flash the firmware")
parser.add_argument("--run", action="store_true", help="run the flashed program")
parser.add_argument("--monitor", action="store_true", help="view serial output")
args = parser.parse_args()

command = [
    "--chip",
    "esp32",
    "--port",
    args.port,
    "--baud",
    "115200",
    "--before",
    "default_reset",
    "--after",
    "hard_reset",
    "write_flash",
    "-z",
    "--flash_mode",
    "dio",
    "--flash_freq",
    "40m",
    "--flash_size",
    "detect",
    "0x1000",
    "bootloader.bin",
    "0x10000",
    "uthar.bin",
    "0x8000",
    "partitions_singleapp.bin",
]

if args.flash:
    print("Using command %s" % " ".join(command))
    esptool.main(command)

if args.run and not args.flash:
    command = ["--chip", "esp32", "--port", args.port, "--baud", "115200", "run"]
    print("Using command %s" % " ".join(command))
    esptool.main(command)

if args.monitor:
    serial_instance = serial.serial_for_url(
        args.port,
        115200,
        parity="N",
        rtscts=False,
        xonxoff=False,
        do_not_open=True,
    )
    serial_instance.open()
    minit = miniterm.Miniterm(serial_instance, echo=False, eol="crlf", filters=[])
    minit.exit_character = chr(0x1D)
    minit.menu_character = chr(0x14)
    minit.raw = True
    minit.set_rx_encoding("UTF-8")
    minit.set_tx_encoding("UTF-8")

    sys.stderr.write(
        "--- minit on {p.name}  {p.baudrate},{p.bytesize},{p.parity},{p.stopbits} ---\n".format(
            p=minit.serial
        )
    )
    sys.stderr.write(
        "--- Quit: {} | Menu: {} | Help: {} followed by {} ---\n".format(
            miniterm.key_description(minit.exit_character),
            miniterm.key_description(minit.menu_character),
            miniterm.key_description(minit.menu_character),
            miniterm.key_description("\x08"),
        )
    )

    minit.start()
    try:
        minit.join(True)
    except KeyboardInterrupt:
        pass
    sys.stderr.write("\n--- exit ---\n")
    minit.join()
    minit.close()
