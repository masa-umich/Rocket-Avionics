# Limestone Firmware
Author: Felixfb, Jackmh </br>
Read Me last updated: 6/23/2025

## Note:
This is the location for the flight computer firmware for Limelight. This README will be updated more as the firmware is written. Currently the firmware is just an empty project that is configured for the flight computer. All peripherals, Ethernet, clock configuration, RTOS, and LWIP are set up, along with a collection of hardware drivers that have virtual folders linked to the board-drivers directory of this repo.

## Important Notes:
Major STM32 bug: the EthIf task does not get enough stack by default, you must go into the ethernetif.c file and change the INTERFACE_THREAD_STACK_SIZE definition at the top from 350 to anything above 380 (I would pick 500 to be safe). If this is not done, the task will start corrupting memory as soon as it starts.

Major STM32 LAN8742 driver bug: The Ethernet link output does not free sent TX packets until the TX descriptors have run out of space. One fix is to add a task that waits on the tx buffer semaphore and clears the descriptors of sent packets every time the LAN chip triggers a TX complete. See ethernetif.c ethernet_patch(), along with the initialization of the task in low_level_init() for an example.


## EEPROM Data Ordering

| Byte  | Meaning | C data type |
| :-----: | ------- | :-----------: |
| 1 - 4 | PT 1 Voltage Offset | float |
| 5 - 8 | PT 1 Pressure Range | float |
| 9 - 12 | PT 1 Maximum Voltage | float |
| 13 - 16 | PT 2 Voltage Offset | float |
| 17 - 20 | PT 2 Pressure Range | float |
| 21 - 24 | PT 2 Maximum Voltage | float |
| 25 - 28 | PT 3 Voltage Offset | float |
| 29 - 32 | PT 3 Pressure Range | float |
| 33 - 36 | PT 3 Maximum Voltage | float |
| 37 - 40 | PT 4 Voltage Offset | float |
| 41 - 44 | PT 4 Pressure Range | float |
| 45 - 48 | PT 4 Maximum Voltage | float |
| 49 - 52 | PT 5 Voltage Offset | float |
| 53 - 56 | PT 5 Pressure Range | float |
| 57 - 60 | PT 5 Maximum Voltage | float |
| 61      | TC 1 Gain | uint8_t |
| 62      | TC 2 Gain | uint8_t |
| 63      | TC 3 Gain | uint8_t |
| 64      | Valve 1 Voltage | uint8_t |
| 65      | Valve 1 Enable | uint8_t |
| 66      | Valve 2 Voltage | uint8_t |
| 67      | Valve 2 Enable | uint8_t |
| 68      | Valve 3 Voltage | uint8_t |
| 69      | Valve 3 Enable | uint8_t |
| 70 - 73 | DAQ PC/Limewire IP | uint32_t |
| 74 - 77 | Flight Computer IP | uint32_t |
| 78 - 81 | Bay Board 1 IP | uint32_t |
| 82 - 85 | Bay Board 2 IP | uint32_t |
| 86 - 89 | Bay Board 3 IP | uint32_t |
| 90 - 93 | Flight Recorder IP | uint32_t |

### Notes
The PT configuration floats are **little-endian**, but the IP addresses are **big-endian**.

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