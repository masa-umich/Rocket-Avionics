import serial.tools.list_ports
import serial
import time
# Note: close the serial monitor in Arduino IDE before running this script.
# Otherwise, the port will be busy and you will get an error.

def get_ports():
    boards = []
    ports = list(serial.tools.list_ports.comports())
    for p in ports:
        if p.vid == 1027: # FTDI Vendor ID 1027
            boards.append(p.device)
    return boards

def cmd_reset():
    print("Reset")
    return 0x00 # Return a dummy value for reset

def cmd_d1_convert(osr):
    print("D1 Convert with OSR:", osr)
    return 0x00

def cmd_d2_convert(osr):
    print("D2 Convert with OSR:", osr)
    return 0x00

def cmd_adc_read():
    print("ADC Read")
    return 0x00

def cmd_prom_read(addr):
    print("PROM Read from address:", addr)
    return 0x00

def parse_data(data):
    # take in the SPI command byte and compare it to all possible valid SPI commands
    match data:
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
        case 0xA0:
            return cmd_prom_read(0)
        case 0xA1:
            return cmd_prom_read(1)
        case 0xA2:
            return cmd_prom_read(2)
        case 0xA3:
            return cmd_prom_read(3)
        case 0xA4:
            return cmd_prom_read(4)
        case 0xA5:
            return cmd_prom_read(5)
        case 0xA6:
            return cmd_prom_read(6)
        case 0xA7:
            return cmd_prom_read(7)
        case 0xA8:
            return cmd_prom_read(8)
        case 0xA9:
            return cmd_prom_read(9)
        case 0xAA:
            return cmd_prom_read(10)
        case 0xAB:
            return cmd_prom_read(11)
        case 0xAC:
            return cmd_prom_read(12)
        case 0xAD:
            return cmd_prom_read(13)
        case 0xAE:
            return cmd_prom_read(14)
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
                print("Responding with: " + hex(response))
                board.write(response)

        except serial.SerialException as e:
            print("Serial exception:", e)
            break
        except KeyboardInterrupt:
            print("Exiting...")
            break

if __name__ == '__main__':
    main()