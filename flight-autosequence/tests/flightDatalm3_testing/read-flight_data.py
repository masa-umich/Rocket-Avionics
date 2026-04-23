import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import sys
import os

def load_pf2(filename):
    """Parse a PerfectFlite .pf2 file into a DataFrame with time_s and altitude in meters."""
    data_rows = []
    in_data = False

    with open(filename, 'r') as f:
        for line in f:
            line = line.strip()
            if line.startswith("Data:"):
                in_data = True
                continue
            if in_data and line:
                parts = [p.strip() for p in line.split(',')]
                if len(parts) >= 2:
                    try:
                        time_s = float(parts[0])
                        alt_ft = float(parts[1])
                        data_rows.append({'time_s': time_s, 'baro-altitude (m)': alt_ft * 0.3048})
                    except ValueError:
                        continue

    df = pd.DataFrame(data_rows)
    # Normalize time to start at 0
    df['time_s'] = df['time_s'] - df['time_s'].iloc[0]
    return df


def load_csv(filename, start_row=0):
    """Load the semicolon-delimited CSV flight data."""
    df = pd.read_csv(filename, sep=';', skiprows=1, decimal=',')

    if start_row > 0:
        print(f"Fast-forwarding to row {start_row}...")
        df = df.iloc[start_row:].copy()

    start_time_ms = df['time (ms)'].iloc[0]
    df['time_s'] = (df['time (ms)'] - start_time_ms) / 1000.0
    return df


def parse_results_file(results_filename):
    """Parse all sections from the autosequence results file."""
    events = {}
    kinematics = {}
    fallback = {}
    start_row = 0

    with open(results_filename, 'r') as file:
        current_section = None
        for line in file:
            line = line.strip()

            if line.startswith("Start row:"):
                start_row = int(line.split(":")[1].strip())
                continue

            if "--- TIMESTAMPS" in line:
                current_section = "timestamps"
                continue
            elif "--- POST LOCKOUT KINEMATICS" in line:
                current_section = "kinematics"
                continue
            elif "--- FALLBACK TIMER PREDICTIONS" in line:
                current_section = "fallback"
                continue
            elif line.startswith("---"):
                current_section = None
                continue

            if current_section == "timestamps" and ":" in line and "ms" in line:
                event_name, time_str = line.split(":", 1)
                time_ms = float(time_str.replace("ms", "").strip())
                events[event_name.strip()] = time_ms / 1000.0

            elif current_section == "kinematics" and ":" in line:
                key, val_str = line.split(":", 1)
                val = float(val_str.strip().split()[0])
                kinematics[key.strip()] = val

            elif current_section == "fallback" and ":" in line:
                key, val_str = line.split(":", 1)
                val_str = val_str.strip()
                if val_str.endswith("ms"):
                    fallback[key.strip()] = float(val_str.replace("ms", "").strip()) / 1000.0
                elif val_str.endswith("m"):
                    fallback[key.strip()] = float(val_str.replace("m", "").strip())

    return events, kinematics, fallback, start_row


def plot_altitude_and_acceleration(flight_filename, results_filename=None, start_row=0,
                                    events=None, kinematics=None, fallback=None):
    print("Loading flight data...")

    ext = os.path.splitext(flight_filename)[1].lower()
    is_pf2 = ext == '.pf2'

    if is_pf2:
        df = load_pf2(flight_filename)
        has_accel = False
        print("Detected PF2 file — altitude only, no acceleration channel.")
    else:
        df = load_csv(flight_filename, start_row)
        has_accel = 'vert-accel (m/s2)' in df.columns

    fig, ax1 = plt.subplots(figsize=(12, 7))

    color1 = 'tab:blue'
    ax1.set_xlabel('Time (seconds)')
    ax1.set_ylabel('Barometric Altitude (m)', color=color1)
    ax1.plot(df['time_s'], df['baro-altitude (m)'], color=color1, label='Altitude', linewidth=2)
    ax1.tick_params(axis='y', labelcolor=color1)
    ax1.grid(True, linestyle='--', alpha=0.5)

    if has_accel:
        ax2 = ax1.twinx()
        color2 = 'tab:orange'
        ax2.set_ylabel('Vertical Acceleration (m/s²)', color=color2)
        ax2.plot(df['time_s'], df['vert-accel (m/s2)'], color=color2, label='Vertical Acceleration',
                 linewidth=1.5, alpha=0.7)
        ax2.tick_params(axis='y', labelcolor=color2)
    else:
        ax2 = None

    if results_filename and events is not None:
        # --- Plot event vertical lines (dotted) ---
        event_colors = ['tab:red', 'tab:green', 'tab:purple', 'tab:brown',
                        'tab:pink', 'tab:gray', 'tab:olive', 'tab:cyan']
        for i, (event_name, time_s) in enumerate(events.items()):
            color = event_colors[i % len(event_colors)]
            ax1.axvline(x=time_s, color=color, linestyle=':', linewidth=2,
                        alpha=0.8, label=event_name)

        # --- Plot fallback prediction vertical lines (dash-dot, heavier) ---
        if fallback:
            t_apogee_pred = fallback.get('Apogee time', None)
            t_drogue_pred = fallback.get('Drogue deploy time', None)
            t_main_pred   = fallback.get('Main deploy time', None)

            fallback_vlines = {
                'Fallback Apogee':        (t_apogee_pred, 'tab:red'),
                'Fallback Drogue Deploy':  (t_drogue_pred, 'tab:orange'),
                'Fallback Main Deploy':    (t_main_pred,   'tab:green'),
            }
            for label, (t, color) in fallback_vlines.items():
                if t is not None:
                    ax1.axvline(x=t, color=color, linestyle='-.', linewidth=2.5,
                                alpha=0.9, label=label)

        # --- Plot kinematic prediction segments ---
        if kinematics and fallback and 'Lockout Ends At' in events:
            t_snapshot = events['Lockout Ends At'] + kinematics.get('Delay after lockout', 0) / 1000.0
            h0 = kinematics.get('Altitude', None)
            v0 = kinematics.get('Velocity', None)
            a0 = kinematics.get('Acceleration', None)

            t_apogee_pred = fallback.get('Apogee time', None)
            t_drogue_pred = fallback.get('Drogue deploy time', None)
            t_main_pred   = fallback.get('Main deploy time', None)
            h_apogee_pred = fallback.get('Apogee altitude', None)
            h_drogue_pred = fallback.get('Drogue deploy altitude', None)
            h_main_pred   = fallback.get('Main deploy altitude', None)

            if None not in (h0, v0, a0):
                print(f"Plotting kinematic curve from t={t_snapshot:.2f}s: h={h0}m, v={v0}m/s, a={a0}m/s²")

                # Segment 1: quadratic coast phase from snapshot to predicted apogee
                t_coast_duration = -v0 / a0
                t_rel = np.linspace(0, t_coast_duration, 300)
                h_quad = h0 + v0 * t_rel + 0.5 * a0 * t_rel**2
                ax1.plot(t_snapshot + t_rel, h_quad, color='red', linestyle='--',
                         linewidth=2, alpha=0.85, label='Predicted coast (quadratic)')

                # Segment 2: linear descent from apogee to drogue deploy
                if None not in (h_apogee_pred, h_drogue_pred, t_apogee_pred, t_drogue_pred):
                    ax1.plot([t_apogee_pred, t_drogue_pred], [h_apogee_pred, h_drogue_pred],
                             color='orange', linestyle='--', linewidth=2, alpha=0.85,
                             label='Predicted drogue descent (linear)')

                # Segment 3: linear descent from drogue to main deploy
                if None not in (h_drogue_pred, h_main_pred, t_drogue_pred, t_main_pred):
                    ax1.plot([t_drogue_pred, t_main_pred], [h_drogue_pred, h_main_pred],
                             color='darkred', linestyle='--', linewidth=2, alpha=0.85,
                             label='Predicted main descent (linear)')

    title = f'Altitude vs. Time — {os.path.basename(flight_filename)}'
    if not is_pf2:
        title += f' (Starting from Row {start_row})'
    plt.title(title)

    lines1, labels1 = ax1.get_legend_handles_labels()
    if ax2 is not None:
        lines2, labels2 = ax2.get_legend_handles_labels()
    else:
        lines2, labels2 = [], []
    ax1.legend(lines1 + lines2, labels1 + labels2, loc='upper right', framealpha=0.9)

    fig.tight_layout()
    plt.show()


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python script.py <flight_file.csv|.pf2> [results_filename] [start_row]")
        sys.exit(1)

    flight_file = sys.argv[1]
    results_file = sys.argv[2] if len(sys.argv) > 2 else 'test-autosequence-flightdata.out'

    events, kinematics, fallback, start_row_from_file = parse_results_file(results_file)

    ext = os.path.splitext(flight_file)[1].lower()
    if ext == '.pf2':
        start = 0  # PF2 always starts from beginning
    else:
        start = int(sys.argv[3]) if len(sys.argv) > 3 else start_row_from_file

    plot_altitude_and_acceleration(
        flight_filename=flight_file,
        results_filename=results_file,
        start_row=start,
        events=events,
        kinematics=kinematics,
        fallback=fallback
    )