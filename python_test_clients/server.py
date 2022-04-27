import socket
import sys

# Create a TCP/IP socket
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

# Bind the socket to the port
server_address = ('localhost', 6664)
print('starting up on %s port %s' % server_address)
sock.bind(server_address)

# Listen for incoming connections
sock.listen(1)

# Wait for a connection
print('waiting for a connection')
connection, client_address = sock.accept()

try:
    print('connection from', client_address)

    while True:
        msg = input("Send a message: ")

        if msg == "shutdown":
            break
        
        connection.send(str(msg+"\n").encode('UTF-8'))
        
finally:
    # Clean up the connection
    sock.close()
    connection.close()
    