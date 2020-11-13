#!/usr/bin/env python3

from bluetooth import *
import os
from PIL import Image
import threading
from http.server import BaseHTTPRequestHandler, HTTPServer
from socketserver import ThreadingMixIn
import re
from io import BytesIO


class CamHandler(BaseHTTPRequestHandler):
    def do_GET(self):
        """
        if self.path.endswith('.mjpg'):
            self.send_response(200)
            self.send_header(
                'Content-type',
                'multipart/x-mixed-replace; boundary=--jpgboundary')
            self.end_headers()
            while True:
                try:
                    img, size = bluetooth_receive()
                    if not img:
                        return
                    img_file = BytesIO(img)
                    jpg = Image.open(img_file)
                    tmpFile = StringIO.StringIO()
                    jpg.save(tmpFile, 'JPEG')
                    self.wfile.write("--jpgboundary")
                    self.send_header('Content-type', 'image/jpeg')
                    self.send_header('Content-length', str(tmpFile.len))
                    self.end_headers()
                    jpg.save(self.wfile, 'JPEG')
                    time.sleep(0.05)
                except KeyboardInterrupt:
                    break
            return
        if self.path.endswith('.html'):
            self.send_response(200)
            self.send_header(b'Content-type', 'text/html')
            self.end_headers()
            self.wfile.write(b'<html><head></head><body>')
            self.wfile.write(b'<img src="http://127.0.0.1:8080/cam.mjpg"/>')
            self.wfile.write(b'</body></html>')
            return
        """
        # Send headers
        self.send_response(200)
        self.send_header("Cache-Control", "no-cache")
        self.send_header("Pragma", "no-cache")
        self.send_header("Connection", "close")
        self.send_header("Content-Type",
                         "multipart/x-mixed-replace; boundary=--myboundary")
        self.end_headers()

        o = self.wfile

        # Send image files in a loop
        lastFrameTime = 0
        while True:
            img, size = bluetooth_receive()
            if not img:
                return

            # Wait if required so we stay under the max FPS
            try:
                o.write(b"--myboundary\r\n")
                o.write(b"Content-Type: image/jpeg\r\n")
                o.write(b"Content-Length: %s\r\n" % len(img))
                o.write(b"\r\n")
                o.write(img)
                o.write(b"\r\n")
            except KeyboardInterrupt:
                return


class ThreadedHTTPServer(ThreadingMixIn, HTTPServer):
    """Handle requests in a separate thread."""


def main():
    global BTsocket, cLength

    cLength = re.compile(r"^Content-Length: (?P<length>\d+)\r\n\r\n$")
    BTsocket = BluetoothSocket(RFCOMM)
    BTsocket.connect((os.environ['ESP_MAC'], 1))
    try:
        server = ThreadedHTTPServer(('localhost', 8080), CamHandler)
        print("server started")
        server.serve_forever()
    except KeyboardInterrupt:
        server.socket.close()
        BTsocket.close()


def bluetooth_receive():
    data = BTsocket.recv(1024)
    while (len(data) > 64):
        data = BTsocket.recv(1024)
    result = re.match(cLength, data.decode('ascii'))
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
        for i in range(10):
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
