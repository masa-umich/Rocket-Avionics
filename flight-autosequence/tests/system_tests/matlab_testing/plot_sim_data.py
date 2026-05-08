"""
plot_flight_data.py

Plots pressure vs time and vertical acceleration vs time from a 3-row data file.

Expected file format (comma-separated, ~10 000 values per row):
  Row 1: static pressure   [Pa]
  Row 2: vertical accel    [m/s²]
  Row 3: time              [s]

Usage:
    python plot_flight_data.py data-IDENTIFIER.txt
    e.g. python plot_flight_data.py data-ideal.txt  →  saves flight_plots-ideal.png
"""

import sys
import os
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.ticker as ticker

# ── Config ───────────────────────────────────────────────────────────────────
DELIMITER      = ","
PRESSURE_UNITS = "Pa"
FIGSIZE        = (14, 8)

# Hardcoded pressure thresholds for deployment altitudes [Pa]
# Adjust these to match your simulation's ground pressure / atmosphere model
P_5K_FT = 78511.0     # Pa at 5 000 ft (1 524 m), standard atmosphere
P_1K_FT = 90717.0     # Pa at 1 000 ft (  305 m), standard atmosphere
# ─────────────────────────────────────────────────────────────────────────────


def derive_output_name(filepath: str) -> str:
    """
    data-ideal.txt  →  flight_plots-ideal.png
    data-3k.txt     →  flight_plots-3k.png
    anything_else   →  flight_plots.png
    """
    base = os.path.basename(filepath)
    name, _ = os.path.splitext(base)
    if "-" in name:
        identifier = name.split("-", 1)[1]
        return f"tests/system_tests/matlab_testing/flight_plots-{identifier}.png"
    return "flight_plots.png"


def load_data(filepath: str) -> tuple:
    """Parse a 3-row text file into (pressure, accel, time) numpy arrays."""
    rows = []
    with open(filepath, "r") as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            values = [float(v) for v in line.split(DELIMITER) if v.strip()]
            rows.append(np.array(values))

    if len(rows) != 3:
        raise ValueError(
            f"Expected exactly 3 data rows, found {len(rows)}. "
            "Check DELIMITER or file format."
        )

    pressure, accel, time = rows
    if not (len(pressure) == len(accel) == len(time)):
        raise ValueError(
            f"Row lengths differ: pressure={len(pressure)}, "
            f"accel={len(accel)}, time={len(time)}"
        )

    print(f"Loaded {len(time)} samples | "
          f"t = [{time[0]:.3f}, {time[-1]:.3f}] s | "
          f"P = [{pressure.min():.1f}, {pressure.max():.1f}] {PRESSURE_UNITS} | "
          f"a = [{accel.min():.2f}, {accel.max():.2f}] m/s^2")

    return pressure, accel, time


def find_descent_crossing(pressure: np.ndarray, time: np.ndarray,
                           threshold_pa: float, apogee_idx: int):
    """
    After apogee, find the first time pressure rises back above threshold_pa
    (rocket descending through that altitude). Returns interpolated timestamp
    in seconds, or None if never crossed in the data.
    """
    for i in range(apogee_idx, len(pressure) - 1):
        if pressure[i] <= threshold_pa <= pressure[i + 1]:
            frac = (threshold_pa - pressure[i]) / (pressure[i + 1] - pressure[i])
            return float(time[i] + frac * (time[i + 1] - time[i]))
    return None


def add_event_vline(ax, t: float, label: str, color: str, y_frac: float = 0.97):
    """Draw a vertical dashed line with a rotated timestamp label."""
    ax.axvline(t, color=color, linewidth=1.2, linestyle="--", alpha=0.85)
    ylim = ax.get_ylim()
    y_pos = ylim[0] + y_frac * (ylim[1] - ylim[0])
    ax.text(t, y_pos, f" {label}\n {t:.1f} s",
            color=color, fontsize=7.5, va="top", ha="left",
            bbox=dict(boxstyle="round,pad=0.2", fc="white", ec=color, alpha=0.75))


def plot(pressure: np.ndarray, accel: np.ndarray, time: np.ndarray,
         out_file: str) -> None:

    # ── Compute event timestamps ──────────────────────────────────────────
    idx_apogee = int(np.argmin(pressure))
    t_apogee   = float(time[idx_apogee])

    p_5k = P_5K_FT
    p_1k = P_1K_FT

    t_5k = find_descent_crossing(pressure, time, p_5k, idx_apogee)
    t_1k = find_descent_crossing(pressure, time, p_1k, idx_apogee)

    print(f"Apogee  : {t_apogee:.2f} s  (P = {pressure[idx_apogee]:.1f} Pa)")
    print(f"5 000 ft: {t_5k:.2f} s" if t_5k else "5 000 ft: not reached in data")
    print(f"1 000 ft: {t_1k:.2f} s" if t_1k else "1 000 ft: not reached in data")

    # ── Plot ──────────────────────────────────────────────────────────────
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=FIGSIZE, sharex=True)
    fig.suptitle("Flight Simulation Data", fontsize=15, fontweight="bold", y=0.98)

    # Pressure
    ax1.plot(time, pressure, color="#2196F3", linewidth=0.9, label="Static Pressure")
    ax1.set_ylabel(f"Pressure ({PRESSURE_UNITS})", fontsize=11)
    ax1.set_title("Pressure vs Time", fontsize=11)
    ax1.grid(True, linestyle="--", alpha=0.4)
    ax1.legend(fontsize=9)
    ax1.yaxis.set_major_formatter(ticker.FormatStrFormatter("%.0f"))

    # Accel
    ax2.plot(time, accel, color="#F44336", linewidth=0.9, label="Vertical Accel")
    ax2.axhline(0, color="black", linewidth=0.6, linestyle="--", alpha=0.6)
    ax2.set_xlabel("Time (s)", fontsize=11)
    ax2.set_ylabel("Acceleration (m/s^2)", fontsize=11)
    ax2.set_title("Vertical Acceleration vs Time", fontsize=11)
    ax2.grid(True, linestyle="--", alpha=0.4)
    ax2.legend(fontsize=9)

    # Annotate peak thrust
    idx_max = int(np.argmax(accel))
    ax2.annotate(
        f"Peak thrust\n{accel[idx_max]:.1f} m/s^2 @ {time[idx_max]:.1f} s",
        xy=(time[idx_max], accel[idx_max]),
        xytext=(time[idx_max] + (time[-1] - time[0]) * 0.03, accel[idx_max] * 0.85),
        arrowprops=dict(arrowstyle="->", color="black"),
        fontsize=8, color="black",
        bbox=dict(boxstyle="round,pad=0.3", fc="white", ec="gray", alpha=0.8),
    )

    # Event vertical lines — drawn after plots so ylims are finalised
    for ax in (ax1, ax2):
        add_event_vline(ax, t_apogee, "Apogee",   color="#9C27B0", y_frac=0.97)
        if t_5k:
            add_event_vline(ax, t_5k, "5 000 ft", color="#FF9800", y_frac=0.85)
        if t_1k:
            add_event_vline(ax, t_1k, "1 000 ft", color="#4CAF50", y_frac=0.73)

    plt.tight_layout()
    plt.savefig(out_file, dpi=150, bbox_inches="tight")
    print(f"Saved -> {out_file}")
    #plt.show()


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python plot_flight_data.py data-IDENTIFIER.txt")
        sys.exit(1)

    filepath = sys.argv[1]
    out_file = derive_output_name(filepath)

    pressure, accel, time = load_data(filepath)
    plot(pressure, accel, time, out_file)