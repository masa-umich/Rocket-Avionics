from scapy.all import IP, UDP, send
from scapy.layers.ntp import NTPHeader
import time

ntp_packet = NTPHeader(mode = 5)

packet = IP(dst="192.168.0.255")/UDP(dport=123, sport=123)/ntp_packet

send(packet)