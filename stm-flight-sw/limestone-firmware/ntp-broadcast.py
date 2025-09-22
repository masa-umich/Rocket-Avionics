# /// script
# requires-python = ">=3.13"
# dependencies = [
#     "scapy",
# ]
# ///
from scapy.all import IP, UDP, send
from scapy.layers.ntp import NTPHeader
import time

send_interval = 2 #seconds

while 1:
    ntp_packet = NTPHeader(mode = 5)

    packet = IP(dst="141.212.192.255")/UDP(dport=123)/ntp_packet

    send(packet)

    time.sleep(send_interval)
