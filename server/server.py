import socket
import threading

# create a socket object
server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

# get local machine name
host = socket.gethostname()

# bind the socket to a public host and a port number
server_socket.bind(("10.209.234.177", 8081))

# set the server to listen for incoming connections
server_socket.listen(1)

print('Server is running on', host, 'port', 8081)

while True:
    # wait for a client to connect
    print('Waiting for a client to connect...')
    client_socket, addr = server_socket.accept()
    print("en klient er kommet ind")
    client_socket.send(bytes("koer frem 5", "utf-8"))
    client_socket.close()
