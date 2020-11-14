#!/usr/bin/env python3

from bluetooth import *
import os
import re


# receive a jpeg image over bluetooth
# the image is split into multiple packets,
# prepended by a Content-Length header packet
def bluetooth_receive():
    # try to receive a length packet
    data = BTsocket.recv(1024)
    # if > 64, we know it's not a length packet
    while (len(data) > 64):
        data = BTsocket.recv(1024)
    # convert length packet to string, and regex out the length from
    # the Content-Length header
    result = re.match(cLength, data.decode('utf-8'))
    if result:
        img = bytes()
        # convert regex result to integer
        img_size = int(result.groups('length')[0])
        print(f"Receiving JPEG of size {img_size} bytes")
        # continue receiving packets until we have received the full image
        while len(img) < img_size:
            img += BTsocket.recv(1024)
            print(f"Received {len(img)} bytes...")

        print("Receive complete")
    return img, img_size


def main():
  """
  This program attempts to receive a stream of a 1000 jpeg images,
  saving them in the out folder
  """
  global img, BTsocket, cLength
   if not os.path.exists('out'):
        os.makedirs('out')
    img = bytes()
    # regex to extract length from the header
    cLength = re.compile(r"^Content-Length: (?P<length>\d+)\r\n\r\n$")
    BTsocket = BluetoothSocket(RFCOMM)
    # grab the mac address from ESP_MAC env variable
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
    main()
