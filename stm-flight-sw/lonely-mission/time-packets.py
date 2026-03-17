import socket
import time

UDP_IP = "0.0.0.0"
UDP_PORT = 1234

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
sock.bind((UDP_IP, UDP_PORT))
while True:
    start = time.time()
    data, addr = sock.recvfrom(3)
    print(1 / (time.time() - start))
