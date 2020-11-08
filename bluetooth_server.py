from bluetooth import *
 
BTsocket=BluetoothSocket( RFCOMM )
 
BTsocket.connect(("CC:50:E3:A9:EF:8A", 1))

data = BTsocket.recv(1024)

img = bytes()

img_size = int.from_bytes(data, "little")
print(f"Receiving JPEG of size {img_size} bytes")

while len(img) < img_size:
    img += BTsocket.recv(1024)
    print(f"Recieved {len(img)} bytes...")

print("Recieve complete")

with open("test.jpeg", "wb") as file:
    file.write(img)

print("File write complete") 
BTsocket.close()