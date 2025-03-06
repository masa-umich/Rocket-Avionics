import time 
import math
import sys
import threading
from queue import Queue

def command_processing(command_queue):
    
    while True:
        #read input from the command_line and place it on the queue
        command = sys.stdin.readline().strip()
        if command: 
            command_queue.put(command)

class Environment:
    P_sea_level = 1 #1 atmosphere
    P_11k = 0.23015758656029023 #atmospheres (calculated with the equation below)
    lapse_rate = 0.0065 #Celsius per meter
    T_sea_level = 293 #Kelvin (20 Celsius)
    T_iso = 216.65 #temp is roughly constant in isothermic region
    g = 9.8 #accel. due to gravity
    M = 0.02896 #Molar mass of air kg/mol
    R = 8.315 #gas constant

    
    def compute_pressure(self, height):
        
        if height < 11000:
            P = (self.P_sea_level 
            * (1 - ((self.lapse_rate * height) / (self.T_sea_level)))
            ** ((self.g * self.M) /(self.R * self.lapse_rate)))

        else:
            exponent = - (self.g * self.M / (self.R * self.T_iso)) * (height - 11000)
            P = self.P_11k * math.exp(exponent)
        
        P = P * 1.01325 * 1000 #convert from atmospheres to millibar
        return P

    
def main():

    env = Environment()
    command_queue = Queue()

    data_file = open('height_data.txt')
    string_of_heights = data_file.read()
    data_file.close()
    list_of_heights = string_of_heights.split()

    #convert all values to floats
    list_of_heights = [float(height) for height in list_of_heights]

    #command_processing function runs on a separate thread
    input_thread = threading.Thread(target=command_processing, args=(command_queue,), daemon=True)
    input_thread.start()

    start_time = time.perf_counter()

    for height_value in list_of_heights:

        elapsed = time.perf_counter() - start_time

        current_height = height_value

        while not command_queue.empty():
            command = command_queue.get()
                
            if command in ("h"):
                print(f"height: {round(current_height,3)} m, time: {round(elapsed,3)} s")
            elif command in ("p"):
                print(
                    f"height: {round(current_height, 3)} m, "
                    f"pressure: {round(env.compute_pressure(current_height), 3)} millibar, "
                    f"time: {round(elapsed, 3)} s"
                )
            else:
                print(f"'{command}' is an invalid command. (ignored)")

        # Sleep before logging the next value
        time.sleep(0.01)

if __name__ == "__main__":
    main()

