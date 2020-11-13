from bluetooth import *
import os
import threading
from http.server import BaseHTTPRequestHandler, HTTPServer
from socketserver import ThreadingMixIn
import re


class CamHandler(BaseHTTPRequestHandler):
    def do_GET(self):
        print("get")
        data = BTsocket.recv(1024)

        result = re.match(cLength, data.decode('utf-8'))
        if result:
            img = bytes()
            img_size = int(result.groups('length')[0])
            print(f"Receiving JPEG of size {img_size} bytes")

            while len(img) < img_size:
                img += BTsocket.recv(1024)
                print(f"Received {len(img)} bytes...")

            print("Receive complete")
        return


class ThreadedHTTPServer(ThreadingMixIn, HTTPServer):
    """Handle requests in a separate thread."""


def main():
    global img, BTsocket, cLength

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


def main2():
    cLength = re.compile(r"^Content-Length: (?P<length>\d+)\r\n\r\n$")
    BTsocket = BluetoothSocket(RFCOMM)
    BTsocket.connect((os.environ['ESP_MAC'], 1))
    while True:
        data = BTsocket.recv(1024)
        print(data)
        result = re.match(cLength, data.decode('utf-8'))
        if result:
            print("match")
            img = bytes()
            print(result.groups('length')[0])
            img_size = int(result.groups('length')[0])
            print(f"Receiving JPEG of size {img_size} bytes")
            BTsocket.close()


if __name__ == '__main__':
    # main()
    main2()
