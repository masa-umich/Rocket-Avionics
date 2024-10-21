# Lonely Mission DAQ

Author: jackmh\
Last updated: 10-21-2024

## Usage:

### To write sensor data:
- Make sure the mode jumper is set to GROUND (black)
- Turn on the board (give it power or press reset w/power)
#### **IMPORTANT!!**
If the board for whatever reason gets reset, either through the power blinking in and out or the reset button being pushed, YOUR DATA WILL GET AUTOMATICALLY ERASED!

So when you're changing the board over from sensor writing mode to data reading mode, MAKE SURE THE JUMPER IS SET TO POWER (red)!

### To read sensor data:
Install Python3 if you haven't already\
Install the Serial Python Module:
```python -m pip install pyserial```
- **SET THE MODE JUMPER TO POWER (red)**\
Flight data will be erased if you do not do this before giving the board power!
- Connect the board to your computer with a Micro-USB cable\
(this will also give the board power)
- Run the Python script `read_flash.py`\
This will automatically make a file (called data.bin or dataX.bin where X is a number) which contains the raw binary data directly from the board. Depending on how long the board has been recording this may take a while
- Run the Python script `convert_bin.py`\
This will ask you what binary file you want to convert and what to name the CSV (spreadsheet) it'll put your flight data into.

In the event there's an issue with your binary data or the conversion of it, let Jack H (@jackmh) know.