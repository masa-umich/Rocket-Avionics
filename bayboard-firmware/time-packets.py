import socket
import time

UDP_IP = "192.168.0.5"
UDP_PORT = 1234

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))
while True:
    start = time.time()
    data, addr = sock.recvfrom(3)
    print(1 / (time.time() - start))