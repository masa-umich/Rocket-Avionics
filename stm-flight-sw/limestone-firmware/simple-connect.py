import socket

HOST = "192.168.0.10"
PORT = 5000

with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    s.connect((HOST, PORT))
    while(1):
        data = s.recv(1024)
        print(f"Received {len(data)!r}")
