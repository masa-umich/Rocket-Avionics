import csv
import math
import random

def generate_flight_data(filename="sim_rocket_data.csv"):
    # Simulation settings
    dt = 0.02  # 50 Hz (20ms intervals)
    t = 0.0
    y = 0.0    # Altitude (m)
    v = 0.0    # Velocity (m/s)
    
    # Target flight parameters 
    meco_time = 20.0       # Updated to 20 second burn duration
    thrust_accel = 37.5    # Constant thrust acceleration (m/s^2)
    g = 9.80665            # Gravity
    k_drag = 0.00015       # Drag factor during ascent/coast
    k_parachute = 0.05     # Drag factor under parachute
    
    # Standard Atmosphere constants
    P0 = 1013.25
    T0 = 288.15            # 15C in Kelvin
    L = 0.0065             # Temperature lapse rate
    
    data = []
    apogee_reached = False
    
    print("Simulating flight (unconstrained apogee, 20s burn)...")
    
    while True:
        # 1. Determine Thrust
        current_thrust = thrust_accel if t < meco_time else 0.0
            
        # 2. Determine Aerodynamic Drag
        # Drag acts in the opposite direction of velocity
        drag_magnitude = (k_drag if not apogee_reached else k_parachute) * (v ** 2)
        drag_accel = -drag_magnitude if v > 0 else drag_magnitude
            
        # 3. Calculate Accelerations
        # Kinematic acceleration (true change in velocity)
        a_kinematic = current_thrust - g + drag_accel
        
        # Proper acceleration (what the IMU actually measures). 
        # When resting on the pad (a_kin = 0), proper accel is +g.
        a_proper = a_kinematic + g
        
        # 4. Integrate Velocity and Position
        v += a_kinematic * dt
        y += v * dt
        
        # Apogee detection
        if v < 0 and not apogee_reached:
            apogee_reached = True
            print(f"Apogee reached at {t:.2f} seconds. Altitude: {y:.2f} meters.")
            
        # Ground impact detection
        if y < 0 and t > 1.0:
            break
            
        # 5. Simulate Sensors
        # Standard Atmosphere Model for Temperature and Pressure
        # Prevent edge case math errors if altitude goes extremely high
        temp_K = max(T0 - L * y, 0.1) 
        pressure = P0 * (1 - L * y / T0) ** 5.25588 if (1 - L * y / T0) > 0 else 0
        
        # Add random noise to sensors to mimic real-world data
        p1 = pressure + random.gauss(0, 0.05)
        p2 = pressure + random.gauss(0, 0.05)
        t1 = temp_K - 273.15 + random.gauss(0, 0.1)
        t2 = temp_K - 273.15 + random.gauss(0, 0.1)
        
        # Accelerometer Y-axis measures proper acceleration
        xl_y1 = a_proper + random.gauss(0, 0.1)
        xl_y2 = a_proper + random.gauss(0, 0.1)
        
        # Off-axes (X, Z) and Gyros (W) are roughly 0 + noise
        xl_x = random.gauss(0, 0.05)
        xl_z = random.gauss(0, 0.05)
        w_x = random.gauss(0, 0.02)
        w_y = random.gauss(0, 0.02)
        w_z = random.gauss(0, 0.02)
        
        # Append row in the requested format
        row = [
            int(t * 1000),
            round(p1, 4), round(p2, 4), round(t1, 4), round(t2, 4),
            round(xl_x, 4), round(xl_y1, 4), round(xl_z, 4),
            round(w_x, 4), round(w_y, 4), round(w_z, 4),
            round(xl_x, 4), round(xl_y2, 4), round(xl_z, 4),
            round(w_x, 4), round(w_y, 4), round(w_z, 4)
        ]
        data.append(row)
        
        t += dt

    # 6. Write to CSV
    headers = [
        "timestamp_ms","bar1_hPa","bar2_hPa","bar1_temp_C","bar2_temp_C",
        "imu1_XL_x","imu1_XL_y","imu1_XL_z","imu1_W_x","imu1_W_y","imu1_W_z",
        "imu2_XL_x","imu2_XL_y","imu2_XL_z","imu2_W_x","imu2_W_y","imu2_W_z"
    ]
    
    with open(filename, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(headers)
        writer.writerows(data)
        
    print(f"Success! Data saved to '{filename}'.")
    print(f"Total flight duration: {t:.2f} seconds.")

if __name__ == "__main__":
    generate_flight_data()