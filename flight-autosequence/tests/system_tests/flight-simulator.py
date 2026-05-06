import math
import csv
import random

# --- Fixed Rocket Hardware ---
THRUST_N = 13344.0       # 3000 lbs in Newtons
DRY_MASS = 239.0         # kg
MASS_FLOW = 5.88         # kg/s (Roughly equates to 106kg over 18s)
AREA = 0.08              # m^2, cross-sectional area of the rocket (assumed 0.32m diameter)

def run_sim(burn_duration, add_noise=False):
    """Runs the physics sim. Returns max altitude and (optionally) the telemetry log."""
    SAMPLE_RATE = 20         
    DT = 1.0 / SAMPLE_RATE
    
    t = 0.0
    h = 0.0                  
    v = 0.0                  
    mass = DRY_MASS + (MASS_FLOW * burn_duration) # Wet mass at liftoff
    max_h = 0.0
    
    data_log = [["Time_s", "Pressure_hPa", "Accel_X_ms2", "Accel_Y_ms2", "Accel_Z_ms2"]]
    
    while True:
        # 1. Environment
        temp = 288.15 - 0.0065 * h
        if temp < 216.65: temp = 216.65 
        
        pressure = 1013.25 * ((temp / 288.15) ** 5.25588)
        density = (pressure * 100) / (287.05 * temp)
        mach = abs(v) / math.sqrt(1.4 * 287.05 * temp)
        
        # 2. Aerodynamics
        cd = 0.4 
        if mach > 0.8:
            cd = 0.4 + 0.8 * math.exp(-((mach - 1.0) / 0.15)**2) 
            if mach > 1.2: cd = 0.6
            
        drag_force = 0.5 * density * (v**2) * cd * AREA
        if v < 0: drag_force = -drag_force # Drag opposes movement
        
        # 3. Engine State
        if t <= burn_duration:
            thrust = THRUST_N
            mass -= MASS_FLOW * DT
            engine_on = True
        else:
            thrust = 0.0
            mass = DRY_MASS
            engine_on = False

        # 4. Acceleration
        a_z_ideal = (thrust - drag_force) / mass - 9.80665

        # 5. Sensor Noise & Logging (Only on final run)
        if add_noise:
            if engine_on:
                a_x = random.uniform(-1.5, 1.5)
                a_y = random.uniform(-1.5, 1.5)
                a_z = a_z_ideal + random.uniform(-2.0, 2.0)
            else:
                a_x = random.uniform(-0.1, 0.1)
                a_y = random.uniform(-0.1, 0.1)
                a_z = a_z_ideal + random.uniform(-0.2, 0.2)
                
            measured_pressure = pressure
            if 0.85 < mach < 1.15:
                measured_pressure += random.uniform(-15.0, 15.0) 

            data_log.append([round(t, 2), round(measured_pressure, 2), round(a_x, 2), round(a_y, 2), round(a_z, 2)])
        
        # 6. Step Forward
        v += a_z_ideal * DT
        h += v * DT
        t += DT
        
        if h > max_h:
            max_h = h
            
        # Stop simulation 2 seconds after apogee is reached
        if v < 0 and (max_h - h) > 50:
            break
            
    return max_h, data_log

# ==========================================
# Main Execution: Target Seeking
# ==========================================
print("--- Liquid Rocket Test Data Generator ---")
target_str = input("Enter target apogee in meters (e.g., 5000): ")
target_apogee = float(target_str)

print(f"\nCalculating required burn time to hit {target_apogee}m...")

# Bisection Search Algorithm
low_burn = 0.1
high_burn = 60.0 # Max fuel capacity assumption
best_burn = 0.0

for i in range(30): # 30 iterations is plenty for millimeter precision
    mid_burn = (low_burn + high_burn) / 2.0
    sim_apogee, _ = run_sim(mid_burn, add_noise=False)
    
    if sim_apogee < target_apogee:
        low_burn = mid_burn
    else:
        high_burn = mid_burn
        
best_burn = (low_burn + high_burn) / 2.0

# Verify we didn't hit our hard ceiling
if best_burn >= 59.9:
    print("\nWARNING: Target apogee exceeds max fuel capacity (60s burn) for this airframe.")
else:
    print(f"Target Acquired! Required MECO at T+ {round(best_burn, 2)} seconds.")
    print("Generating noisy sensor telemetry file...")
    
    # Run final simulation with noise and generate CSV
    final_apogee, final_data = run_sim(best_burn, add_noise=True)
    
    filename = f"flight-autosequence/testing/target_flight_data-apogee_{target_apogee}.csv"
    with open(filename, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerows(final_data)
        
    print(f"Success! Telemetry saved to {filename}")
    print(f"Actual simulated apogee: {round(final_apogee, 2)}m")