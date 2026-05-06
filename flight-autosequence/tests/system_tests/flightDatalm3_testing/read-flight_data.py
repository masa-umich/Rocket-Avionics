# /// script
# requires-python = ">=3.13"
# dependencies = [
#     "matplotlib>=3.10.9",
#     "numpy>=2.4.4",
#     "pandas>=3.0.2",
#     "tk>=0.1.0",
# ]
# ///

import pandas as pd
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
import matplotlib.colors as mcolors
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import numpy as np
import sys
import os
import tkinter as tk
from tkinter import ttk


# ── Data loaders ────────────────────────────────────────────────────────────

def load_pf2(filename):
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
                        data_rows.append({
                            'time_s':            float(parts[0]),
                            'baro-altitude (m)': float(parts[1]) * 0.3048
                        })
                    except ValueError:
                        continue
    df = pd.DataFrame(data_rows)
    df['time_s'] -= df['time_s'].iloc[0]
    return df


def load_csv(filename, start_row=0):
    df = pd.read_csv(filename, sep=';', skiprows=1, decimal=',')
    if start_row > 0:
        df = df.iloc[start_row:].copy()
    df['time_s'] = (df['time (ms)'] - df['time (ms)'].iloc[0]) / 1000.0
    return df


def parse_results_file(results_filename):
    events, kinematics, fallback = {}, {}, {}
    start_row = 0
    current_section = None
    with open(results_filename, 'r') as f:
        for line in f:
            line = line.strip()
            if line.startswith("Start row:"):
                start_row = int(line.split(":")[1].strip())
            elif "--- TIMESTAMPS" in line:
                current_section = "timestamps"
            elif "--- POST LOCKOUT KINEMATICS" in line:
                current_section = "kinematics"
            elif "--- FALLBACK TIMER PREDICTIONS" in line:
                current_section = "fallback"
            elif line.startswith("---"):
                current_section = None
            elif current_section == "timestamps" and ":" in line and "ms" in line:
                k, v = line.split(":", 1)
                events[k.strip()] = float(v.replace("ms", "").strip()) / 1000.0
            elif current_section == "kinematics" and ":" in line:
                k, v = line.split(":", 1)
                kinematics[k.strip()] = float(v.strip().split()[0])
            elif current_section == "fallback" and ":" in line:
                k, v = line.split(":", 1)
                v = v.strip()
                if v.endswith("ms"):
                    fallback[k.strip()] = float(v.replace("ms", "").strip()) / 1000.0
                elif v.endswith("m"):
                    fallback[k.strip()] = float(v.replace("m", "").strip())
    return events, kinematics, fallback, start_row


# ── Main GUI ─────────────────────────────────────────────────────────────────

def run_gui(flight_filename, results_filename, start_row, events, kinematics, fallback):
    ext = os.path.splitext(flight_filename)[1].lower()
    is_pf2 = ext == '.pf2'

    if is_pf2:
        df = load_pf2(flight_filename)
        has_accel = False
    else:
        df = load_csv(flight_filename, start_row)
        has_accel = 'vert-accel (m/s2)' in df.columns

    # ── Build figure ─────────────────────────────────────────────────────────
    fig, ax1 = plt.subplots(figsize=(11, 6))
    fig.patch.set_facecolor('#ffffff')
    ax1.set_facecolor('#ffffff')
    ax1.tick_params(colors='#222222')
    ax1.xaxis.label.set_color('#222222')
    ax1.yaxis.label.set_color('#1a56db')
    for spine in ax1.spines.values():
        spine.set_edgecolor('#cccccc')
    ax1.set_xlabel('Time (s)')
    ax1.set_ylabel('Altitude (m)', color='#1a56db')
    ax1.grid(True, linestyle='--', alpha=0.4, color='#dddddd')

    layer_data = []

    def add(label, color, artist):
        layer_data.append([label, mcolors.to_hex(color), artist])

    # Altitude
    ln, = ax1.plot(df['time_s'], df['baro-altitude (m)'],
                   color='#1a56db', linewidth=2.5)
    add('Altitude', '#1a56db', ln)

    # Acceleration (twin axis)
    if has_accel:
        ax2 = ax1.twinx()
        ax2.set_facecolor('#ffffff')
        ax2.tick_params(colors='#222222')
        ax2.yaxis.label.set_color('#e8500a')
        for spine in ax2.spines.values():
            spine.set_edgecolor('#cccccc')
        ax2.set_ylabel('Vert. Accel. (m/s²)', color='#e8500a')
        ln2, = ax2.plot(df['time_s'], df['vert-accel (m/s2)'],
                        color='#e8500a', linewidth=2, alpha=0.9)
        add('Vert. Acceleration', '#e8500a', ln2)

    # Event vlines — vivid saturated colors, thick
    event_colors = ['#e60026', '#1e8c1e', '#7c3aed', '#c47d00',
                    '#0090b5', '#0047ab', '#c4003c', '#007a6e']
    for i, (name, t) in enumerate(events.items()):
        c = event_colors[i % len(event_colors)]
        vl = ax1.axvline(x=t, color=c, linestyle=':', linewidth=2.5, alpha=1.0)
        add(name, c, vl)

    # Fallback vlines — dash-dot, even thicker
    if fallback:
        fb_map = {
            'Fallback Apogee':        (fallback.get('Apogee time'),        '#e60026'),
            'Fallback Drogue Deploy': (fallback.get('Drogue deploy time'), '#e8500a'),
            'Fallback Main Deploy':   (fallback.get('Main deploy time'),   '#1e8c1e'),
        }
        for name, (t, c) in fb_map.items():
            if t is not None:
                vl = ax1.axvline(x=t, color=c, linestyle='-.', linewidth=3.0, alpha=1.0)
                add(name, c, vl)

    # Kinematic curves
    if kinematics and fallback and 'Lockout Ends At' in events:
        t0 = events['Lockout Ends At'] + kinematics.get('Delay after lockout', 0) / 1000.0
        h0 = kinematics.get('Altitude')
        v0 = kinematics.get('Velocity')
        a0 = kinematics.get('Acceleration')

        t_ap = fallback.get('Apogee time');        h_ap = fallback.get('Apogee altitude')
        t_dr = fallback.get('Drogue deploy time'); h_dr = fallback.get('Drogue deploy altitude')
        t_mn = fallback.get('Main deploy time');   h_mn = fallback.get('Main deploy altitude')

        if None not in (h0, v0, a0):
            dur = -v0 / a0
            t_r = np.linspace(0, dur, 300)
            ln_c, = ax1.plot(t0 + t_r, h0 + v0*t_r + 0.5*a0*t_r**2,
                             color='#e60026', linestyle='--', linewidth=2.5, alpha=1.0)
            add('Predicted coast', '#e60026', ln_c)

            if None not in (t_ap, t_dr, h_ap, h_dr):
                ln_d, = ax1.plot([t_ap, t_dr], [h_ap, h_dr],
                                 color='#e8500a', linestyle='--', linewidth=2.5, alpha=1.0)
                add('Predicted drogue descent', '#e8500a', ln_d)

            if None not in (t_dr, t_mn, h_dr, h_mn):
                ln_m, = ax1.plot([t_dr, t_mn], [h_dr, h_mn],
                                 color='#c4003c', linestyle='--', linewidth=2.5, alpha=1.0)
                add('Predicted main descent', '#c4003c', ln_m)

    ax1.set_title(f'Altitude vs Time — {os.path.basename(flight_filename)}',
                  color='#222222', fontsize=12, pad=10)
    fig.tight_layout()

    # ── Tkinter window ────────────────────────────────────────────────────────
    root = tk.Tk()
    root.title("Flight Data")
    root.configure(bg='#f5f5f5')

    canvas = FigureCanvasTkAgg(fig, master=root)
    canvas.draw()
    canvas.get_tk_widget().pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

    sidebar = tk.Frame(root, bg='#ebebeb', width=200)
    sidebar.pack(side=tk.RIGHT, fill=tk.Y)
    sidebar.pack_propagate(False)

    tk.Label(sidebar, text='LAYERS', bg='#ebebeb', fg='#888888',
             font=('Courier', 9, 'bold'), pady=10).pack()

    ttk.Separator(sidebar, orient='horizontal').pack(fill=tk.X, padx=10, pady=4)

    btn_frame = tk.Frame(sidebar, bg='#ebebeb')
    btn_frame.pack(fill=tk.BOTH, expand=True, padx=8, pady=4)

    buttons = []

    def make_callback(i):
        def toggle():
            artist = layer_data[i][2]
            artist.set_visible(not artist.get_visible())
            if artist.get_visible():
                buttons[i].config(relief=tk.RAISED,
                                  bg=layer_data[i][1],
                                  fg=_contrast(layer_data[i][1]))
            else:
                buttons[i].config(relief=tk.SUNKEN,
                                  bg='#cccccc', fg='#999999')
            canvas.draw()
        return toggle

    def _contrast(hex_color):
        r, g, b = mcolors.to_rgb(hex_color)
        return '#ffffff' if (0.299*r + 0.587*g + 0.114*b) < 0.55 else '#111111'

    for i, (label, hex_col, _) in enumerate(layer_data):
        btn = tk.Button(
            btn_frame,
            text=label,
            command=make_callback(i),
            bg=hex_col,
            fg=_contrast(hex_col),
            activebackground=hex_col,
            font=('Courier', 8),
            relief=tk.RAISED,
            bd=1,
            anchor='w',
            padx=6,
            pady=3,
            wraplength=170,
            justify=tk.LEFT,
            cursor='hand2',
        )
        btn.pack(fill=tk.X, pady=2)
        buttons.append(btn)

    def on_close():
        plt.close('all')
        root.quit()
        root.destroy()

    root.protocol("WM_DELETE_WINDOW", on_close)
    root.mainloop()


# ── Entry point ───────────────────────────────────────────────────────────────

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python script.py <flight.csv|.pf2> [results.out] [start_row]")
        sys.exit(1)

    flight_file  = sys.argv[1]
    results_file = sys.argv[2] if len(sys.argv) > 2 else 'test-autosequence-flightdata.out'

    events, kinematics, fallback, start_row_from_file = parse_results_file(results_file)

    ext = os.path.splitext(flight_file)[1].lower()
    start = 0 if ext == '.pf2' else (
        int(sys.argv[3]) if len(sys.argv) > 3 else start_row_from_file
    )

    run_gui(flight_file, results_file, start, events, kinematics, fallback)