import os
import socket
from threading import Thread

HOST = '127.0.0.1'  # The server's hostname or IP address
PORT = 50007        # The port used by the server

soc = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
server_address = (HOST, PORT)
soc.sendto(b"Hello from lbr client", server_address)

def receive():
    while True:
        data, server = soc.recvfrom(1024)
        if len(data) == 0:
            os._exit(1)
        print('Received', repr(data))

def send():
    while True:
        msg = input("Enter message: ")
        soc.sendto(msg.encode("UTF-8"), server_address)


if __name__ == '__main__':
    Thread(target = receive).start()
    Thread(target = send).start()