# Bay Board Firmware
Author: Felixfb </br>
Read Me last updated: 8/15/2025

## Important Notes:
Major STM32 bug: the EthIf task does not get enough stack by default, you must go into the ethernetif.c file and change the INTERFACE_THREAD_STACK_SIZE definition at the top from 350 to anything above 380 (I would pick 500 to be safe). If this is not done, the task will start corrupting memory as soon as it starts.

Major STM32 LAN8742 driver bug: The Ethernet link output does not free sent TX packets until the TX descriptors have run out of space. One fix is to add a task that waits on the tx buffer semaphore and clears the descriptors of sent packets every time the LAN chip triggers a TX complete. See ethernetif.c ethernet_patch(), along with the initialization of the task in low_level_init() for an example.

## Versioning
To keep track of the firmware version, a header file with build information is
auto-generated before each build. This build
information includes the short hash of the most recent git commit, the active branch, type
of build (Debug vs Release), and the timestamp of the build. This build info is logged in
flash and over UDP
when the firmware starts. The auto-generation script is a [uv
script](../version-info/version-gen.py) that automatically runs before a build. This STM32
project is set up to automatically run this script, however it requires that uv is
installed in the correct location. The configuration assumes that uv is installed in
~/.local/bin/uv (this is the default install location), but if your uv is installed in a different location, you can go to
Properties -> C/C++ Build -> Environment and add the location of your uv to the PATH variable.

## EEPROM Data Ordering

| Bytes | Meaning | C data type |
| :-----: | ------- | :-----------: |
| 1 | Bay Board # | uint8_t |
| 4 | PT 1 Voltage Offset | float |
| 4 | PT 1 Pressure Range | float |
| 4 | PT 1 Maximum Voltage | float |
| 4 | PT 2 Voltage Offset | float |
| 4 | PT 2 Pressure Range | float |
| 4 | PT 2 Maximum Voltage | float |
| 4 | PT 3 Voltage Offset | float |
| 4 | PT 3 Pressure Range | float |
| 4 | PT 3 Maximum Voltage | float |
| 4 | PT 4 Voltage Offset | float |
| 4 | PT 4 Pressure Range | float |
| 4 | PT 4 Maximum Voltage | float |
| 4 | PT 5 Voltage Offset | float |
| 4 | PT 5 Pressure Range | float |
| 4 | PT 5 Maximum Voltage | float |
| 4 | PT 6 Voltage Offset | float |
| 4 | PT 6 Pressure Range | float |
| 4 | PT 6 Maximum Voltage | float |
| 4 | PT 7 Voltage Offset | float |
| 4 | PT 7 Pressure Range | float |
| 4 | PT 7 Maximum Voltage | float |
| 4 | PT 8 Voltage Offset | float |
| 4 | PT 8 Pressure Range | float |
| 4 | PT 8 Maximum Voltage | float |
| 4 | PT 9 Voltage Offset | float |
| 4 | PT 9 Pressure Range | float |
| 4 | PT 9 Maximum Voltage | float |
| 4 | PT 10 Voltage Offset | float |
| 4 | PT 10 Pressure Range | float |
| 4 | PT 10 Maximum Voltage | float |
| 1 | TC 1 Gain | uint8_t |
| 1 | TC 2 Gain | uint8_t |
| 1 | TC 3 Gain | uint8_t |
| 1 | TC 4 Gain | uint8_t |
| 1 | TC 5 Gain | uint8_t |
| 1 | TC 6 Gain | uint8_t |
| 1 | Valve 1 Voltage | uint8_t |
| 1 | Valve 1 Enable | uint8_t |
| 1 | Valve 2 Voltage | uint8_t |
| 1 | Valve 2 Enable | uint8_t |
| 1 | Valve 3 Voltage | uint8_t |
| 1 | Valve 3 Enable | uint8_t |
| 1 | Valve 4 Voltage | uint8_t |
| 1 | Valve 4 Enable | uint8_t |
| 1 | Valve 5 Voltage | uint8_t |
| 1 | Valve 5 Enable | uint8_t |
| 4 | Flight Computer IP | uint32_t |
| 4 | Bay Board IP | uint32_t |

### Notes
The PT configuration floats are **little-endian**, but the IP addresses are **big-endian**.

The Bay Board # directly corresponds to the board, e.g. Bay Board 1 is 1

TC gain chart:
| Gain | Config Value |
| :---: | :----------: |
| 1x   | 0x00         |
| 2x   | 0x01         |
| 4x   | 0x02         |
| 8x   | 0x03         |
| 16x   | 0x04         |
| 32x   | 0x05         |
| 64x   | 0x06         |
| 128x   | 0x07         |

For the valve voltage configuration, 12V is 0x00, 24V is 0x01

The eeprom config file must named eeprom.bin and must have 32 bit CRC32 checksum at the end of the file of the rest of the config. This 32 bit checksum should be little-endian

You can then send the eeprom.bin using any TFTP client

Refer to [eeprom-bin-generate.py](eeprom-bin-generate.py) for an example or to actually generate a config

## TCP Keep-Alive

Idle timeout - 5 seconds

Probe interval - 3 seconds

Probe count - 3