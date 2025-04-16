import serial.tools.list_ports
import serial
import time
import random
# Note: close the serial monitor in Arduino IDE before running this script.
# Otherwise, the port will be busy and you will get an error.

global ADC_result, PROM_values
ADC_result = 0 # Placeholder for ADC result
PROM_values = [ 40127, 36924, 23317, 23282, 33464, 28312 ]

def get_ports():
    boards = []
    ports = list(serial.tools.list_ports.comports())
    for p in ports:
        if p.vid == 1027: # FTDI Vendor ID 1027
            boards.append(p.device)
    return boards

def cmd_reset():
    print("Reset")
    return None # No response needed

def cmd_d1_convert(osr):
    print("D1 Convert with OSR:", osr)
    # TODO: Replace with actual conversion logic
    ADC_result = random.randint(0, 0xFFFFFF)
    return None # No response, just internal operation

def cmd_d2_convert(osr):
    print("D2 Convert with OSR:", osr)
    # TODO: Replace with actual conversion logic
    ADC_result = random.randint(0, 0xFFFFFF)
    return None # No response, just internal operation

def cmd_adc_read():
    print("ADC Read")
    rx_buffer = bytes([0xFE,                  # this byte doesn't make sense to be here, but it's what the real chip does
                  (ADC_result >> 16) & 0xFF,  # High byte
                  (ADC_result >> 8) & 0xFF,   # Middle byte
                   ADC_result & 0xFF])        # Low byte
    return rx_buffer

def cmd_prom_read(addr):
    prom_buffer = 


    print("PROM Read from address:", addr)
    rx_buffer = bytes([0xFE,
                  (PROM_values[addr] >> 8) & 0xFF,  # High byte
                  PROM_values[addr] & 0xFF])         # Low byte
    return rx_buffer

def parse_data(data):
    # take in the SPI command byte and compare it to all possible valid SPI commands
    if (data & 0xF0) == 0xA0: # All possible PROM read commands
        # Interpret bits 0, 1, and 2 of the command byte as the address
        return cmd_prom_read(((0x0E & (data & 0x0F)) >> 1))
    match data: # All other unique commands
        case 0x1E:
            return cmd_reset()
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
        case 0x00:
            return cmd_adc_read()
        case _:
            print("INVALID_COMMAND")
            return None

def main():
    if not get_ports():
        while not get_ports():
            print('No boards found. Make sure the board is connected. Retrying...', end='\r', flush=True)
        print()
    time.sleep(1) # The serial port takes a second before we can open it
    board = serial.Serial(get_ports()[0], 115200) # Open the serial port
    print("Connected to", get_ports()[0])

    while True:
        try:
            data = board.read(1) # Read one byte
            if data:
                data = int.from_bytes(data, 'big') # Convert to integer
                print("Received: " + hex(data))
                response = parse_data(data)
                if response == None:
                    continue
                print("Responding with: " + hex(int(response)))
                board.write(response)

        except serial.SerialException as e:
            print("Serial exception:", e)
            break
        except KeyboardInterrupt:
            print("Exiting...")
            break

if __name__ == '__main__':
    main()