# Ethernet STM Project Template for Nucleo H723ZG Boards
Original Ethernet Driver Author: Evan Eidt\
H7 Chip Port: Jack Hammerberg

This is the repository for the template project for ethernet usage on H7 chips. Basically, setting up ethernet on H7 chips is a massive pain because of the MPU and other memory things that don't exist on other boards so we have this repository so that you don't have to waste a solid hour or two doing that yourself.

## Usage
1. Clone this repo
2. Open the project using STM32 CubeIDE
    (*if it asks you to migrate to the latest version, you should be able to fine*)
3. **IMPORTANT** This project assumes you have the current drivers installed and has a linked folder for them. To fix these dependencies you'll have to:
    1. go and clone the dev-drivers branch
    2. right click on the ETH project in CubeIDE and go to properties
    3. Go to C/C++ General Setting Dropdown, go to the bottom and go to `Paths and Symbols`
    4. Go to the `Source Locations` tab
    5. Click `Link Folder` and navigate to your `dev-drivers` path. Apply & Close
    6. You should now see the dev-drivers folder in the ETH project, but it has other drivers that are not the TCP driver that will cause compile errors
    7. Highlight all folders inside the dev-drivers folder (the folders for the other device drivers) except for the tcp driver folder
    8. Right click, `Resource Configuration`, `Exclude from Build`, Select `Debug` & `Release` and apply
4. Attach your Nucleo H723ZG board through Micro-USB and ethernet
5. Configure your computer to have a static IP that is `192.168.50.x` **where x is NOT** 10 (because that's the IP of the board) and a subnet mask of `255.255.255.0`\
Exactly how to do this vary by OS so look up how to do this on your laptop.
6. Upload the code, to confirm that it's running the middle LED should start blinking every second
7. Open telnet or whatever the Linux/MacOS equivelant of telnet is and connect to `telnet 192.168.50.10 50000`
8. Expected behavior is the string "heartbeat" being sent every 500ms, you can send a command to turn on and off LED1 by typing a capital `A` into the terminal.

There is a known bug that successive commands sent to the board will eventually cause a hard fault, we're working on it.

## Setting up this project from scratch
This was such a mess to get working, I legitematly don't remember all of the steps exactly so you may have to troubleshoot and debug even if you follow these instructions
1. Copy the exact MPU settings from this IOC file
2. Enable FreeRTOS (**CMSIS v1**) & lwip
3. In the FreeRTOS configuration change these values:
    - `MAX_PRIORITIES = 16`
    - `USE_COUNTING_SEMAPHORES = Enabled`
    - `TOTAL_HEAP_SIZE = 25600 Bytes`
    - `Heap Location = heap_4`
    - `Advanced Settings -> USE_NEWLIB_REENTRANT = Enabled`
4. Make the tasks:
    - `tcpTask - Above Normal Priority - 512 Word Stack Size`
    - `HeartbeatTask - Below Normal Priority - 256 Word Stack Size`
    - `rxMessages - Idle Priority - 256 Word Stack Size`
5. Copy the code for those tasks from this repo
6. In the lwip configuration change these values:
    - `DHCP = Disabled`
    - `IP_ADDRESS = 192.168.050.010`
    - `SUBNET_MASK = 255.255.255.000`\
    Key Options:
    - `MEM_SIZE = 32232`
    - `LWIP_RAM_HEAP_POINTER = 0x30000200`

That might work??? idk you try it