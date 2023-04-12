import socket

def main():
    try:
        # create a socket object
        client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

        # connect to the server at the specified IP address and port
        client_socket.connect(("10.209.234.177", 8081))

        # create a file-like wrapper for receiving data from the server
        in_file = client_socket.makefile('r')

        # read the server's response
        response = in_file.readline()
        print("Received:", response.strip())

        # close the input stream and socket
        in_file.close()
        client_socket.close()

    except Exception as e:
        print(e)

if __name__ == "__main__":
    main()