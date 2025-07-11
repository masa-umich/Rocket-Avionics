import socket

HOST = "192.168.0.10"  # Server's hostname or IP address
PORT = 5000        # Port used by the server

with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    s.connect((HOST, PORT))
    s.sendall(b"Hello, world")
    while(1):
        data = s.recv(1024)
        print(f"Received {len(data)!r}")
