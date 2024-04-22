import struct

def main():
    with open('data.bin', 'rb') as bin, open('data.csv', 'w') as csv:
        # Write the header to the CSV file
        csv.write('Time,Pressure,Temperature,Accel X,Accel Y,Accel Z,Gyro X,Gyro Y,Gyro Z\n')
        # Interpret the binary file
        # Data structure:
        # 64 bit time stamp - Milliseconds since program start
        # 32 bit float - Pressure
        # 32 bit float - Temperature
        # 32 bit float*3 - Accel X, Y, Z
        # 32 bit float*3 - Gyro X, Y, Z
        # Total size: 8 + 4 + 4 + 12 + 12 = 40 Bytes per reading
        # Get 40 byte chunks from the binary file
        while True:
            data = bin.read(40)
            if not data:
                break
            time = struct.unpack('Q', data[:8])[0]
            pressure = struct.unpack('<f', data[8:12])[0]
            temperature = struct.unpack('<f', data[12:16])[0] 
            accel_x = struct.unpack('<f', data[16:20])[0] 
            accel_y = struct.unpack('<f', data[20:24])[0] 
            accel_z = struct.unpack('<f', data[24:28])[0] 
            gyro_x = struct.unpack('<f', data[28:32])[0] 
            gyro_y = struct.unpack('<f', data[32:36])[0] 
            gyro_z = struct.unpack('<f', data[36:40])[0] 
            # Store to CSV
            csv.write(f'{time},{pressure},{temperature},{accel_x},{accel_y},{accel_z},{gyro_x},{gyro_y},{gyro_z}\n')
            print(f'{time},{pressure},{temperature},{accel_x},{accel_y},{accel_z},{gyro_x},{gyro_y},{gyro_z}')

if __name__ == '__main__':
    main()