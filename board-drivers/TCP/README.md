# Ethernet Driver
This is the ethernet driver for the flight computer and bay boards. This was originally made by Evan Eidt.

* [LAN8742A-CZ Datasheet](https://ww1.microchip.com/downloads/en/DeviceDoc/DS_LAN8742_00001989A.pdf)
* Type: Ethernet
* Interface: Ethernet-MAC interface for STM32H7 Chip
* Driver Requirements:
    * Init
    * Send Command
    * Read Register
    * Write Register
    * Transmit Data
    * Recieve Data
    
---
## Todo:
- TCP client for recieving data. This will be used for inter-board communication.
- test tcp client
- Document setup process for Nucleo board

I will write more documentation for this later.

