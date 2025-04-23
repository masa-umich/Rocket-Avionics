import time
import baro_sim

#EXAMPLE USAGE#
data_file = open('height_data.txt')
string_of_heights = data_file.read()
data_file.close()
list_of_heights = string_of_heights.split()

#pull every tenth value... 1, 11, 21, 31, etc.
list_of_heights = (list_of_heights[::10])
env = baro_sim.Environment()
baro = baro_sim.Barometer()

#send 10 readings per second
#height_data.txt contains readings every 0.01 seconds 
#so if you want 50 readings per second do list_of_heights[::2] with time.sleep(0.02)
for height_value in list_of_heights:
    
    time.sleep(0.1)
    P_true  = env.compute_pressure(float(height_value))
    T_true  = env.compute_temp(float(height_value))

    #inject noise
    P_noisy = P_true + baro_sim.gaussian_random(0.0, 1.5)
    T_noisy = T_true + baro_sim.gaussian_random(0.0, 8)

    #convert to D1, D2 for transmission
    D1, D2 = baro.guess_and_check(P_noisy, T_noisy)

    print(f"Height: {height_value} m  |  P: {P_true/100:.2f} mbar |  T: {T_true/100:.2f} °C")
    print("Encoded as: D1 =", D1, " D2 =", D2)



