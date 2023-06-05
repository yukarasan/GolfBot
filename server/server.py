import socket
from flask import Flask


app=Flask(__name__)

@app.route("/")
def test():
    return "hej"

app.run(port=8081)


def setUpSocketAndReturnIt():
    # create a socket object
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    # bind the socket to a public host and a port number
    # ip = "192.168.0.101"
    #ip = "172.20.10.4"
    ip = "localhost"
    server_socket.bind((ip, 8081))
    # set the server to listen for incoming connections
    server_socket.listen(1)
    print('Server is running on', ip, 'port', 8081)
    return server_socket


def waitForClientAndReturnClientSocket(server_socket):
    global client_socket
    while True:
        print('Waiting for a client to connect...')
        client_socket, addr = server_socket.accept()
        break
    return client_socket


def sendMessageINS(message, client_socket):
    print("sending", message)
    client_socket.send(bytes(message, "utf-8"))
    client_socket.close()


def sendMessageInstruction(messageInstruction, server_socket):
    while True:
        # wait for a client to connect
        print('Waiting for a client to connect...')
        client_socket, addr = server_socket.accept()
        print("sender", messageInstruction)
        client_socket.send(bytes(messageInstruction, "utf-8"))
        client_socket.close()


def testOpenCVandSocket():
    server_socket = setUpSocketAndReturnIt()
    client_socket = waitForClientAndReturnClientSocket(server_socket)
    sendMessageINS("din far", client_socket)

    server_socket.close()


#if __name__ == "__main__":
 #   testOpenCVandSocket()
