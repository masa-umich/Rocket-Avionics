import pandas as pd
import matplotlib.pyplot as plt

def plot_altitude_and_acceleration(csv_filename, results_filename=None, start_row=0):
    print("Loading flight data...")
    
    # Read the CSV file
    df = pd.read_csv(csv_filename, sep=';', skiprows=1, decimal=',')

    if start_row > 0:
        print(f"Fast-forwarding to row {start_row}...")
        df = df.iloc[start_row:].copy()

    # Convert time from milliseconds to seconds
    start_time_ms = df['time (ms)'].iloc[0]
    df['time_s'] = (df['time (ms)'] - start_time_ms) / 1000.0

    # Create the plot with two y-axes
    fig, ax1 = plt.subplots(figsize=(12, 7)) 

    # Plot Altitude
    color1 = 'tab:blue'
    ax1.set_xlabel('Time since startup (seconds)')
    ax1.set_ylabel('Barometric Altitude (m)', color=color1)
    ax1.plot(df['time_s'], df['baro-altitude (m)'], color=color1, label='Altitude', linewidth=2)
    ax1.tick_params(axis='y', labelcolor=color1)
    ax1.grid(True, linestyle='--', alpha=0.5)

    # Plot Vertical Acceleration
    ax2 = ax1.twinx()  
    color2 = 'tab:orange'
    ax2.set_ylabel('Vertical Acceleration (m/s²)', color=color2)
    ax2.plot(df['time_s'], df['vert-accel (m/s2)'], color=color2, label='Vertical Acceleration', linewidth=1.5, alpha=0.7)
    ax2.tick_params(axis='y', labelcolor=color2)

    # =========================================================
    # NEW LOGIC: Parse the autosequence text file and plot lines
    # =========================================================
    if results_filename:
        print("Loading autosequence events...")
        events = {}
        
        # 1. Parse the text file
        with open(results_filename, 'r') as file:
            in_timestamps_section = False
            for line in file:
                line = line.strip()
                
                # Check if we are entering or leaving the timestamps section
                if "--- TIMESTAMPS" in line:
                    in_timestamps_section = True
                    continue
                elif in_timestamps_section and line.startswith("---"):
                    break # We hit the next section, stop parsing!
                
                # If we are inside the section, extract the event and time
                if in_timestamps_section and ":" in line and "ms" in line:
                    event_name, time_str = line.split(":", 1)
                    time_ms = float(time_str.replace("ms", "").strip())
                    events[event_name.strip()] = time_ms / 1000.0 # Convert to seconds
        
        # 2. Plot the vertical lines
        # Define a list of high-contrast colors to cycle through
        event_colors = ['tab:red', 'tab:green', 'tab:purple', 'tab:brown', 'tab:pink', 'tab:gray', 'tab:olive', 'tab:cyan']
        
        for i, (event_name, time_s) in enumerate(events.items()):
            # Grab the next color in the list
            color = event_colors[i % len(event_colors)]
            
            # Draw the dotted line AND assign it a label for the legend
            ax1.axvline(x=time_s, color=color, linestyle=':', linewidth=2, alpha=0.8, label=event_name)

    plt.title(f'Altitude and Vertical Acceleration vs. Time (Starting from Row {start_row})')
    
    # 3. Combine legends from both axes into one master legend
    lines1, labels1 = ax1.get_legend_handles_labels()
    lines2, labels2 = ax2.get_legend_handles_labels()
    
    # Place the legend in the upper right corner
    ax1.legend(lines1 + lines2, labels1 + labels2, loc='upper right', framealpha=0.9)

    fig.tight_layout()
    
    # Display the graph
    plt.show()

# Run the function
plot_altitude_and_acceleration(
    csv_filename='flightDatalm3_testing/flightData_lm3.csv', 
    results_filename='flightDatalm3_testing/test-autosequence-flightdata.out', 
    start_row=2090
)