# /// script
# requires-python = ">=3.13"
# dependencies = []
# ///
from enum import Enum
import ipaddress
import struct
import zlib


class ValveVoltage(Enum):
    Valve_12V = 0
    Valve_24V = 1

class TCGain(Enum):
    Gain_1x = 0
    Gain_2x = 1
    Gain_4x = 2
    Gain_8x = 3
    Gain_16x = 4
    Gain_32x = 5
    Gain_64x = 6
    Gain_128x = 7

pt1_offset = 0.5
pt1_range = 5000
pt1_max = 4.5

pt2_offset = 1
pt2_range = 2000
pt2_max = 3

pt3_offset = 0.5
pt3_range = 5000
pt3_max = 4.5

pt4_offset = 0.5
pt4_range = 5000
pt4_max = 5

pt5_offset = 0.5
pt5_range = 6000
pt5_max = 4.5

pt6_offset = 0.5
pt6_range = 5000
pt6_max = 4.5

pt7_offset = 1
pt7_range = 2000
pt7_max = 3

pt8_offset = 0.5
pt8_range = 5000
pt8_max = 4.5

pt9_offset = 0.5
pt9_range = 5000
pt9_max = 5

pt10_offset = 0.5
pt10_range = 6000
pt10_max = 4.5

tc1_gain = TCGain.Gain_64x
tc2_gain = TCGain.Gain_64x
tc3_gain = TCGain.Gain_64x
tc4_gain = TCGain.Gain_64x
tc5_gain = TCGain.Gain_64x
tc6_gain = TCGain.Gain_64x

vlv1_voltage = ValveVoltage.Valve_24V
vlv1_enable = 1

vlv2_voltage = ValveVoltage.Valve_24V
vlv2_enable = 1

vlv3_voltage = ValveVoltage.Valve_24V
vlv3_enable = 1

vlv4_voltage = ValveVoltage.Valve_24V
vlv4_enable = 1

vlv5_voltage = ValveVoltage.Valve_24V
vlv5_enable = 1

bay_board_number = 1
flight_computer_IP = ipaddress.IPv4Address("141.212.192.170")
bay_board_IP = ipaddress.IPv4Address("141.212.192.180")

raw_out = struct.pack('<B', bay_board_number)
raw_out += struct.pack('<f', pt1_offset) + struct.pack('<f', pt1_range) + struct.pack('<f', pt1_max)
raw_out += struct.pack('<f', pt2_offset) + struct.pack('<f', pt2_range) + struct.pack('<f', pt2_max)
raw_out += struct.pack('<f', pt3_offset) + struct.pack('<f', pt3_range) + struct.pack('<f', pt3_max)
raw_out += struct.pack('<f', pt4_offset) + struct.pack('<f', pt4_range) + struct.pack('<f', pt4_max)
raw_out += struct.pack('<f', pt5_offset) + struct.pack('<f', pt5_range) + struct.pack('<f', pt5_max)
raw_out += struct.pack('<f', pt6_offset) + struct.pack('<f', pt6_range) + struct.pack('<f', pt6_max)
raw_out += struct.pack('<f', pt7_offset) + struct.pack('<f', pt7_range) + struct.pack('<f', pt7_max)
raw_out += struct.pack('<f', pt8_offset) + struct.pack('<f', pt8_range) + struct.pack('<f', pt8_max)
raw_out += struct.pack('<f', pt9_offset) + struct.pack('<f', pt9_range) + struct.pack('<f', pt9_max)
raw_out += struct.pack('<f', pt10_offset) + struct.pack('<f', pt10_range) + struct.pack('<f', pt10_max)
raw_out += struct.pack('<B', tc1_gain.value) + struct.pack('<B', tc2_gain.value) + struct.pack('<B', tc3_gain.value)
raw_out += struct.pack('<B', tc4_gain.value) + struct.pack('<B', tc5_gain.value) + struct.pack('<B', tc6_gain.value)
raw_out += struct.pack('<B', vlv1_voltage.value) + struct.pack('<B', vlv1_enable)
raw_out += struct.pack('<B', vlv2_voltage.value) + struct.pack('<B', vlv2_enable)
raw_out += struct.pack('<B', vlv3_voltage.value) + struct.pack('<B', vlv3_enable)
raw_out += struct.pack('<B', vlv4_voltage.value) + struct.pack('<B', vlv4_enable)
raw_out += struct.pack('<B', vlv5_voltage.value) + struct.pack('<B', vlv5_enable)
raw_out += flight_computer_IP.packed + bay_board_IP.packed

print(len(raw_out))

crc = zlib.crc32(raw_out)

raw_out += struct.pack('<I', crc)

with open("eeprom.bin", 'w+b') as f:
    f.write(raw_out)