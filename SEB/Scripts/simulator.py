import serial.tools.list_ports
import serial
import time
import random
# Note: close the serial monitor in Arduino IDE before running this script.
# Otherwise, the port will be busy and you will get an error.

# MS5611 Command Definitions (from datasheet and STM32 C code)
# Convert D1 (pressure) commands
MS5611_CMD_CONVERT_D1_BASE = 0x40
MS5611_CMD_CONVERT_D1_OSR_256  = 0x40
MS5611_CMD_CONVERT_D1_OSR_512  = 0x42
MS5611_CMD_CONVERT_D1_OSR_1024 = 0x44
MS5611_CMD_CONVERT_D1_OSR_2048 = 0x46
MS5611_CMD_CONVERT_D1_OSR_4096 = 0x48
D1_COMMANDS = [
    MS5611_CMD_CONVERT_D1_OSR_256, MS5611_CMD_CONVERT_D1_OSR_512,
    MS5611_CMD_CONVERT_D1_OSR_1024, MS5611_CMD_CONVERT_D1_OSR_2048,
    MS5611_CMD_CONVERT_D1_OSR_4096
]

# Convert D2 (temperature) commands
MS5611_CMD_CONVERT_D2_BASE = 0x50
MS5611_CMD_CONVERT_D2_OSR_256  = 0x50
MS5611_CMD_CONVERT_D2_OSR_512  = 0x52
MS5611_CMD_CONVERT_D2_OSR_1024 = 0x54
MS5611_CMD_CONVERT_D2_OSR_2048 = 0x56
MS5611_CMD_CONVERT_D2_OSR_4096 = 0x58
D2_COMMANDS = [
    MS5611_CMD_CONVERT_D2_OSR_256, MS5611_CMD_CONVERT_D2_OSR_512,
    MS5611_CMD_CONVERT_D2_OSR_1024, MS5611_CMD_CONVERT_D2_OSR_2048,
    MS5611_CMD_CONVERT_D2_OSR_4096
]

global ADC_result, PROM_values
ADC_result = 9085466 # Placeholder for ADC result

def get_ports():
    boards = []
    ports = list(serial.tools.list_ports.comports())
    for p in ports:
        if p.vid == 1027: # FTDI Vendor ID 1027
            boards.append(p.device)
    return boards

def cmd_d1_convert(osr):
    print("D1 Convert with OSR:", osr)
    # TODO: Replace with actual conversion logic
    ADC_result = random.randint(0, 0xFFFFFF)
    return ADC_result # No response, just internal operation

def cmd_d2_convert(osr):
    print("D2 Convert with OSR:", osr)
    # TODO: Replace with actual conversion logic
    ADC_result = random.randint(0, 0xFFFFFF)
    return ADC_result # No response, just internal operation

def parse_data(data):
    match data: # All other unique commands
        case 0x40:
            return cmd_d1_convert(256)
        case 0x42:
            return cmd_d1_convert(512)
        case 0x44:
            return cmd_d1_convert(1024)
        case 0x46: 
            return cmd_d1_convert(2048)
        case 0x48: 
            return cmd_d1_convert(4096)
        case 0x40:
            return cmd_d2_convert(256)
        case 0x42: 
            return cmd_d2_convert(512)
        case 0x44: 
            return cmd_d2_convert(1024)
        case 0x46: 
            return cmd_d2_convert(2048)
        case 0x48:
            return cmd_d2_convert(4096)
        case _:
            print("INVALID_COMMAND")
            return None

def main():
    if not get_ports():
        while not get_ports():
            print('No boards found. Make sure the board is connected. Retrying...', end='\r', flush=True)
        print()
    time.sleep(1) # The serial port takes a second before we can open it
    board = serial.Serial(get_ports()[0], 115200, timeout=0.1) # Open the serial port
    # Note: timeout=0.1 is needed so that we can read the serial port without blocking, allowing for keyboard interrupts
    print("Connected to", get_ports()[0])

    while True:
        try:
            data = board.read(1) # Read one byte
            if not data:
                continue
            data = int.from_bytes(data, 'big') # Convert to integer
            print("Received: " + hex(data))
            # response = parse_data(data)
            # if response == None:
            #     continue
            # print("Responding with: " + hex(response))
            response = bytes([0x8A, 0x9F, 0x1A])
            print("Responding with:", response.hex())
            message_sent = board.write(response)
            board.flush()
            print("Sent response of length:", message_sent)
        except KeyboardInterrupt:
            print("Exiting...")
            board.close()
            break
        except serial.SerialException as e:
            print("Serial exception:", e)
            break
            
if __name__ == '__main__':
    main()