import struct

global timeout_bytes, header
timeout_bytes = 1024*8
header = 'Time (ms),Pressure (hPa),Temperature (C),Accel X (m/s^2),Accel Y (m/s^2),Accel Z (m/s^2),Gyro X (deg/s),Gyro Y (deg/s),Gyro Z (deg/s)\n'

def read_until(file, match):
    data = b''
    while True:
        byte = file.read(1)
        data += byte

        # Every 10th byte, print the last 10 bytes
        # if len(data) % 10 == 0:
        #     print(data[-(len(match)):])

        if data[-(len(match)):] == match:
            print("Header found, starting data conversion...")
            break
        # If it's been a while since we've seen any data, assume we've timed out
        if data[-timeout_bytes:] == b'\x00' * timeout_bytes:
            print("Timed out, probably end of file")
            break
    return data

def main():
    # Interpret the binary file
    # Data structure:
    # 64 bit time stamp - Milliseconds since program start
    # 32 bit float - Pressure
    # 32 bit float - Temperature
    # 32 bit float*3 - Accel X, Y, Z
    # 32 bit float*3 - Gyro X, Y, Z
    # Total size: 8 + 4 + 4 + 12 + 12 = 40 Bytes per reading
    data_file_name = input('Enter the name of the binary file, with its file extension, to convert: ')
    csv_file_name = input('Enter the name of the CSV file to write to: ')
    try:
        with open(data_file_name, 'rb') as bin, open(csv_file_name, 'w') as csv:
            print("File opened successfully, finding header...")
            # Write the header to the CSV file
            csv.write(header)

            # Advance the pointer to the first data point (after this header)
            # The flash chip has this weird behavior where the first like page of data is gibberish and not where we started writing
            # So we write a bunch of 00s at the beginning of the file to skip over that, and then a header to signal the start of the data
            data = read_until(bin, b'\xff\x00\xff\x00\xff\x00\xff\x00\xff\x00')

            while True:
                # Get 40 byte chunks from the binary file
                data = bin.read(40)

                # If the data is all 0xFF or 0x00, we've (probably) reached the end of the file
                # This does kind of assume that you're never going to write 40 contiguous bytes of 0xFF or 0x00 EVER
                # If you do, you'll have to come up with a different way to detect the end of the file
                if (data == (b'\xff' * (len(data)))) or (data == (b'\x00' * (len(data)))):
                    print("Bad block of data, probably end of file")
                    break
                try:
                    # Convert raw binary to human-readable data using the data structure above
                    time = struct.unpack('Q', data[:8])[0]
                    pressure = struct.unpack('<f', data[8:12])[0]
                    temperature = struct.unpack('<f', data[12:16])[0] 
                    accel_x = struct.unpack('<f', data[16:20])[0] 
                    accel_y = struct.unpack('<f', data[20:24])[0] 
                    accel_z = struct.unpack('<f', data[24:28])[0] 
                    gyro_x = struct.unpack('<f', data[28:32])[0] 
                    gyro_y = struct.unpack('<f', data[32:36])[0] 
                    gyro_z = struct.unpack('<f', data[36:40])[0]
                    # Break if any of the data is NaN
                    if any([x != x for x in [time, pressure, temperature, accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z]]):
                        # If the data is written correctly, this should never happen
                        # In reality, a power cycle or other error could cause this, it's very hard to shut down the system gracefully
                        print('NaN data, probably end of file')
                        break
                except struct.error:
                    print('Error unpacking data, continuing...')
                    continue
                except KeyboardInterrupt:
                    print('Exiting...')
                    break
                # Store to CSV
                csv.write(f'{time},{pressure},{temperature},{accel_x},{accel_y},{accel_z},{gyro_x},{gyro_y},{gyro_z}\n')
                #print(f'{time},{pressure},{temperature},{accel_x},{accel_y},{accel_z},{gyro_x},{gyro_y},{gyro_z}')
    except FileNotFoundError:
        print('File not found')

if __name__ == '__main__':
    main()