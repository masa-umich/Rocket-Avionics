import serial.tools.list_ports
import serial
# Note: close the serial monitor in Arduino IDE before running this script.
# Otherwise, the port will be busy and you will get an error.

def get_ports():
    boards = []
    ports = list(serial.tools.list_ports.comports())
    for p in ports:
        if p.vid == 1155: # STM Vendor ID 1155
            boards.append(p.device)
    return boards

def main():
    while not get_ports():
        print('No boards found. Make sure the board is connected.', end='\r', flush=True)
    board = serial.Serial(get_ports()[0], 115200) # Open the serial port
    print("\nConnected to", get_ports()[0])
    with open('data.bin', 'wb') as bin:
        try:
            while True:
                # Read data from the serial port
                data = board.read(512) # Read 512 bytes
                print(data)
                # Save to binary file
                bin.write(data)
        except KeyboardInterrupt:
            print('Exiting...')
            board.close()

if __name__ == '__main__':
    main()