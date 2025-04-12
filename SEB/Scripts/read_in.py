import serial.tools.list_ports
import serial
import time
# Note: close the serial monitor in Arduino IDE before running this script.
# Otherwise, the port will be busy and you will get an error.

global default_data_file_name
default_data_file_name = 'data'

def get_ports():
    boards = []
    ports = list(serial.tools.list_ports.comports())
    for p in ports:
        if p.vid == 1027: # FTDI Vendor ID 1027
            boards.append(p.device)
    return boards

def main():
    if not get_ports():
        while not get_ports():
            print('No boards found. Make sure the board is connected. Retrying...', end='\r', flush=True)
        print()
    time.sleep(1) # The serial port takes a second before we can open it
    board = serial.Serial(get_ports()[0], 115200) # Open the serial port
    print("Connected to", get_ports()[0])
    
    bytes_read_total = 0
    data = []
    
    # Messy code that finds the next available data file name if the default is already taken
    i = 0
    data_file_name = default_data_file_name
    while True:
        try: 
            if open(data_file_name + '.bin', 'rb'):
                i += 1
                data_file_name = default_data_file_name + str(i)
        except:
            break

    with open(data_file_name + '.bin', 'wb') as bin:
        try:
            # print data as it comes in
            while True:
                bytes_read = board.in_waiting
                if bytes_read > 0:
                    bytes_read_total += bytes_read
                    data.append(board.read(bytes_read))
                    print('\rBytes read:', bytes_read_total, end='', flush=True)
        except KeyboardInterrupt:
            pass
        for data in data:
            bin.write(data)
        board.close()
        print('\nExiting...')

if __name__ == '__main__':
    main()