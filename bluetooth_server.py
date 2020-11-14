#!/usr/bin/env python3

from bluetooth import *
import os
import re


def bluetooth_receive():
    data = BTsocket.recv(1024)
    while (len(data) > 64):
        data = BTsocket.recv(1024)
    result = re.match(cLength, data.decode('utf-8'))
    if result:
        img = bytes()
        img_size = int(result.groups('length')[0])
        # print(f"Receiving JPEG of size {img_size} bytes")
        while len(img) < img_size:
            img += BTsocket.recv(1024)
            # print(f"Received {len(img)} bytes...")

        # print("Receive complete")
    return img, img_size


def main2():
    global img, BTsocket, cLength
    if not os.path.exists('out'):
        os.makedirs('out')
    img = bytes()
    cLength = re.compile(r"^Content-Length: (?P<length>\d+)\r\n\r\n$")
    BTsocket = BluetoothSocket(RFCOMM)
    BTsocket.connect((os.environ['ESP_MAC'], 1))
    try:
        for i in range(1000):
            img, size = bluetooth_receive()
            if not img:
                continue
            with open(f"out/test{i}.jpeg", "wb") as file:
                file.write(img)
    except KeyboardInterrupt:
        BTsocket.close()


if __name__ == '__main__':
    # main()
    main2()
