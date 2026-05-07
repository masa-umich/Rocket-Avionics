import pandas as pd
import matplotlib.pyplot as plt

def plot_flight_data(filename="tests/system_tests/AI_generated_testing/sim_sensor_data.csv"):
    try:
        df = pd.read_csv(filename)
    except FileNotFoundError:
        print(f"Error: Could not find '{filename}'. Run the generator script first.")
        return

    # Convert timestamp to seconds
    time_s = df['timestamp_ms'] / 1000.0
    
    # Calculate altitude from barometric pressure
    # Formula: h = 44330 * [1 - (P/P0)^(1/5.255)]
    P0 = 1013.25
    altitude = 44330.0 * (1.0 - (df['bar1_hPa'] / P0) ** (1 / 5.255))
    
    # Extract Y-axis proper acceleration
    accel_y = df['imu1_XL_y']
    
    # Set up the plot
    fig, ax1 = plt.subplots(figsize=(12, 6))

    # Plot Altitude (Primary Y-axis)
    color = 'tab:blue'
    ax1.set_xlabel('Time (s)', fontweight='bold')
    ax1.set_ylabel('Barometric Altitude (m)', color=color, fontweight='bold')
    ax1.plot(time_s, altitude, color=color, linewidth=2, label='Altitude')
    ax1.tick_params(axis='y', labelcolor=color)
    ax1.grid(True, linestyle='--', alpha=0.6)

    # Plot Acceleration (Secondary Y-axis)
    ax2 = ax1.twinx()
    color = 'tab:red'
    ax2.set_ylabel('Proper Acceleration (m/s²)', color=color, fontweight='bold')
    ax2.plot(time_s, accel_y, color=color, alpha=0.5, linewidth=1.5, label='IMU Y-Axis')
    ax2.tick_params(axis='y', labelcolor=color)

    # Annotate key flight events
    plt.axvline(x=16, color='black', linestyle='-.', alpha=0.8, label='Target MECO (~16s)')
    plt.axvline(x=55, color='green', linestyle='-.', alpha=0.8, label='Target Apogee (~55s)')

    # Add legends
    lines_1, labels_1 = ax1.get_legend_handles_labels()
    lines_2, labels_2 = ax2.get_legend_handles_labels()
    ax1.legend(lines_1 + lines_2, labels_1 + labels_2, loc='upper left')

    plt.title('Simulated Rocket Flight: Altitude & Acceleration', fontsize=14, fontweight='bold')
    fig.tight_layout()
    plt.show()

if __name__ == "__main__":
    plot_flight_data()