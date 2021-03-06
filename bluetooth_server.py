#!/usr/bin/env python3

from bluetooth import BluetoothSocket, RFCOMM, find_service
import os
import re
import time
import math
import dmp


def display_data(f, packet, name):
    data = f(packet)
    print(f"{name}: {data}")


def mpu_extract(packet):
    display_data(dmp.quaternion, packet, "quaternion")
    display_data(dmp.gravity, packet, "gravity")
    ypr = dmp.yawPitchRoll(packet)
    print("YAW: %3.1f" % (ypr[0] * 180 / math.pi))
    print("PITCH: %3.1f" % (ypr[1] * 180 / math.pi))
    print("ROLL: %3.1f" % (ypr[2] * 180 / math.pi))
    display_data(dmp.gyro, packet, "gyro")
    display_data(dmp.accel, packet, "accel")
    display_data(dmp.linAccel, packet, "linAccel")
    display_data(dmp.linAccelWorld, packet, "linAccelWorld")
    display_data(dmp.euler, packet, "euler")


# receive a jpeg image over bluetooth
# the image is split into multiple packets,
# prepended by a Content-Length header packet
def bluetooth_receive():
    global BTsocket, img, cLength
    # Awknowledge readiness to receive by sending "RCV_READY" (null-terminated)
    BTsocket.send("RCV_READY".encode("ASCII") + b"\x00")
    # try to receive a length packet
    data = BTsocket.recv(1024)
    # if > 64, we know it's not a length packet
    # while len(data) > 128:
    # data = BTsocket.recv(1024)
    # 10 mpu packets
    for _ in range(10):
        # remove the 42 byte mpu packet
        mpu_data = data[:42]
        # print(mpu_data.hex())
        mpu_extract(mpu_data)
        data = data[42:]
    # print(data)
    # convert length packet to string, and regex out the length from
    # the Content-Length header
    result = re.match(cLength, data.decode("utf-8"))
    if result:
        img = bytes()
        # convert regex result to integer
        img_size = int(result.groups("length")[0])
        print(f"Receiving JPEG of size {img_size} bytes")
        # continue receiving packets until we have received the full image
        while len(img) < img_size:
            img += BTsocket.recv(1024)
            # print(f"Received {len(img)} bytes...")

        print("Receive complete")
    return img, img_size


def main():
    """
    This program attempts to receive a stream of a 1000 jpeg images,
    saving them in the out folder
    """
    global img, BTsocket, cLength
    if not os.path.exists("out"):
        os.makedirs("out")
    img = bytes()
    # regex to extract length from the header
    cLength = re.compile(r"^Content-Length: (?P<length>\d+)\r\n\r\n$")
    BTsocket = BluetoothSocket(RFCOMM)
    # search for our device
    found = False
    spp_service_devices = []
    while not found:
        print("Searching for device...")
        spp_service_devices = find_service(name="UTHAR_SERVER", uuid="1101")
        if len(spp_service_devices):
            found = True
    # connect to the first device which advertizes the SPP service
    print("Device found. Connecting...")
    BTsocket.connect((spp_service_devices[0]["host"], spp_service_devices[0]["port"]))
    start = time.time()
    images = 1000
    img_count = 0
    try:
        for i in range(images):
            print(f"\nImage {i}:")
            img, size = bluetooth_receive()
            if not img:
                continue
            with open(f"out/test{i}.jpeg", "wb") as file:
                file.write(img)
            img_count += 1
    except KeyboardInterrupt:
        BTsocket.close()
    end = time.time()
    print(
        f"Received {img_count} images in {end - start:.2f} seconds: {img_count/(end - start):.2f} FPS"
    )


if __name__ == "__main__":
    main()
