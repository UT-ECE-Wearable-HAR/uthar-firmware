#!/usr/bin/env python3

from bluetooth import BluetoothSocket, RFCOMM, find_service
import os
import re
import time
import math
import bindings.dmp as dmp


def vectorfloat_extract(f, packet, name):
    vec = dmp.VectorFloat()
    f(vec, packet)
    print(f"{name}: x: {vec.x}, y: {vec.y}, z: {vec.z}")
    return vec


def vectorint_extract(f, packet, name):
    vec = dmp.VectorInt16()
    f(vec, packet)
    print(f"{name}: x: {vec.x}, y: {vec.y}, z: {vec.z}")
    return vec


def vectorint_extract_accel(f, other, name, accel):
    vec = dmp.VectorInt16()
    f(vec, accel, other)
    print(f"{name}: x: {vec.x}, y: {vec.y}, z: {vec.z}")
    return vec


def mpu_extract(packet):
    q = dmp.Quaternion()
    dmp.dmpGetQuaternion(q, packet)
    print(f"quaternion: w: {q.w}, x: {q.x}, y: {q.y}, z: {q.z}")

    gravity = dmp.VectorFloat()
    dmp.dmpGetGravity(gravity, q)
    print(f"gravity: x: {gravity.x}, y: {gravity.y}, z: {gravity.z}")

    ypr = dmp.VectorFloat()
    dmp.dmpGetYawPitchRoll(ypr, q, gravity)
    # ypr is radians, converted here to degrees for display
    print("YAW: %3.1f" % (ypr.z * 180 / math.pi))
    print("PITCH: %3.1f" % (ypr.y * 180 / math.pi))
    print("ROLL: %3.1f" % (ypr.x * 180 / math.pi))

    accel = vectorint_extract(dmp.dmpGetAccel, packet, "Accel")
    vectorint_extract(dmp.dmpGetGyro, packet, "Gyro")

    # remmoves gravity component from accel
    accelReal = vectorint_extract_accel(
        dmp.dmpGetLinearAccel, gravity, "LinearAccel", accel
    )

    # rotate measured 3D acceleration vector into original state
    # frame of reference based on orientation quaternion
    vectorint_extract_accel(
        dmp.dmpGetLinearAccelInWorld, q, "LinearAccelWorld", accelReal
    )

    euler = dmp.VectorFloat()
    dmp.dmpGetEuler(euler, q)
    # euler angles are in radians
    print(f"euler: psi: {euler.x}, theta: {euler.y}, phi: {euler.z}")


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
    while len(data) > 128:
        data = BTsocket.recv(1024)
    # remove the 42 byte mpu packet
    mpu_data = data[:42]
    mpu_extract(mpu_data)
    data = data[42:]
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
    try:
        for i in range(images):
            print(f"\nImage {i}:")
            img, size = bluetooth_receive()
            if not img:
                continue
            with open(f"out/test{i}.jpeg", "wb") as file:
                file.write(img)
    except KeyboardInterrupt:
        BTsocket.close()
    end = time.time()
    print(
        f"Received {images} images in {end - start:.2f} seconds: {images/(end - start):.2f} FPS"
    )


if __name__ == "__main__":
    main()
