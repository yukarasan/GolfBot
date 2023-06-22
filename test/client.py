import socket
import json

def main():
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.connect(("10.209.234.177", 8081))

    try:
        request = "GET / HTTP/1.1\r\nHost: 10.209.234.177\r\n\r\n"
        sock.send(request.encode())

        response = ''
        while True:
            data = sock.recv(1024)
            if not data:
                break
            response += data.decode()

        header, _, body = response.partition('\r\n\r\n')
        if '200 OK' in header:
            json_data = json.loads(body)
            print(json_data)
        else:
            print('Request failed.')

    except Exception as e:
        print(e)

    finally:
        sock.close()

if __name__ == "__main__":
    main()
