import socket
import datetime
import math
import time

UDP_IP = "192.168.0.5"
UDP_PORT = 1234

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))
start = time.time()
while True:
    data, addr = sock.recvfrom(9)
    ns_time = int.from_bytes(data[0:8], 'big', signed=True)
    dt = datetime.datetime.fromtimestamp(ns_time / 1e9, tz=datetime.timezone.utc)
    print(abs(dt - datetime.datetime.now(datetime.timezone.utc)), "\t\t\t\t", time.time() - start)
    #print(ns_time)
    print()
    start = time.time()
    #print(data[8:9].hex())
