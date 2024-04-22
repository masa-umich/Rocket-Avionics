import serial.tools.list_ports
import serial
import time
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
        print('No boards found. Make sure the board is connected. Retrying...', end='\r', flush=True)
    board = serial.Serial(get_ports()[0], 115200) # Open the serial port
    print("\nConnected to", get_ports()[0])
    
    bytes_read_total = 0
    start_time = time.time()
    
    with open('data.bin', 'wb') as bin:
        try:
            while True:
                # Read data from the serial port
                data = board.read(512) # Read 512 bytes
                bytes_read_total += len(data)
                #print(data)
                
                # Save to binary file
                bin.write(data)
                
                # Calculate transfer speed
                elapsed_time = time.time() - start_time
                transfer_speed = bytes_read_total / (1024 * elapsed_time)  # Convert bytes to KB and time to seconds
                print(f"Transfer speed: {transfer_speed:.2f} KB/s", end='\r', flush=True)
        except KeyboardInterrupt:
            print('\nExiting...')
            board.close()

if __name__ == '__main__':
    main()