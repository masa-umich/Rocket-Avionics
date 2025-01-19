#height_data.txt is most recent height readings from MASTRAN
#pulled from Xs variable (row 11) in variable export from 
#most recent MASTRAN simulation (1/12/2025)

#program sends/prints 10 readings per second, simulating "real-time" data transmission

import time

data_file = open('height_data.txt')
string_of_heights = data_file.read()
data_file.close()
list_of_heights = string_of_heights.split()

#pull every tenth value... 1, 11, 21, 31, etc.
list_of_heights = (list_of_heights[::10])

#send 10 readings per second
#height_data.txt contains readings every 0.01 seconds 
#so if you want 50 readings per second do list_of_heights[::2] with time.sleep(0.02)
for height_value in list_of_heights:
    print(height_value)
    time.sleep(0.1)
 
