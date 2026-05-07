"""
plot_flight_data.py

Plots pressure vs time and vertical acceleration vs time from a 3-row data file.

Expected file format (comma-separated, ~10 000 values per row):
  Row 1: static pressure   [Pa or hPa – label adjusted below]
  Row 2: vertical accel    [m/s²]
  Row 3: time              [s]

Usage:
    python plot_flight_data.py                        # uses default filename below
    python plot_flight_data.py my_data.txt
"""

import sys
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.ticker as ticker

# ── Config ──────────────────────────────────────────────────────────────────
DEFAULT_FILE   = "data.txt"   # change if your file has a different name
DELIMITER      = ","                  # change to None for whitespace-separated
PRESSURE_UNITS = "Pa"                 # change to "hPa" if needed
FIGSIZE        = (14, 8)
# ────────────────────────────────────────────────────────────────────────────


def load_data(filepath: str) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
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
          f"a = [{accel.min():.2f}, {accel.max():.2f}] m/s²")

    return pressure, accel, time


def plot(pressure: np.ndarray, accel: np.ndarray, time: np.ndarray) -> None:
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=FIGSIZE, sharex=True)
    fig.suptitle("Flight Simulation Data", fontsize=15, fontweight="bold", y=0.98)

    # ── Pressure ──────────────────────────────────────────────────────────
    ax1.plot(time, pressure, color="#2196F3", linewidth=0.9, label="Static Pressure")
    ax1.set_ylabel(f"Pressure ({PRESSURE_UNITS})", fontsize=11)
    ax1.set_title("Pressure vs Time", fontsize=11)
    ax1.grid(True, linestyle="--", alpha=0.5)
    ax1.legend(fontsize=9)
    ax1.yaxis.set_major_formatter(ticker.FormatStrFormatter("%.1f"))

    # Annotate min pressure (apogee)
    idx_min = int(np.argmin(pressure))
    ax1.annotate(
        f"Apogee\n{pressure[idx_min]:.1f} {PRESSURE_UNITS} @ {time[idx_min]:.1f} s",
        xy=(time[idx_min], pressure[idx_min]),
        xytext=(time[idx_min] + (time[-1] - time[0]) * 0.03, pressure[idx_min] + (pressure.max() - pressure.min()) * 0.05),
        arrowprops=dict(arrowstyle="->", color="black"),
        fontsize=8, color="black",
        bbox=dict(boxstyle="round,pad=0.3", fc="white", ec="gray", alpha=0.8),
    )

    # ── Acceleration ───────────────────────────────────────────────────────
    ax2.plot(time, accel, color="#F44336", linewidth=0.9, label="Vertical Accel")
    ax2.axhline(0, color="black", linewidth=0.6, linestyle="--", alpha=0.6)
    ax2.set_xlabel("Time (s)", fontsize=11)
    ax2.set_ylabel("Acceleration (m/s²)", fontsize=11)
    ax2.set_title("Vertical Acceleration vs Time", fontsize=11)
    ax2.grid(True, linestyle="--", alpha=0.5)
    ax2.legend(fontsize=9)

    # Annotate max acceleration (peak thrust)
    idx_max = int(np.argmax(accel))
    ax2.annotate(
        f"Peak thrust\n{accel[idx_max]:.1f} m/s² @ {time[idx_max]:.1f} s",
        xy=(time[idx_max], accel[idx_max]),
        xytext=(time[idx_max] + (time[-1] - time[0]) * 0.03, accel[idx_max] * 0.85),
        arrowprops=dict(arrowstyle="->", color="black"),
        fontsize=8, color="black",
        bbox=dict(boxstyle="round,pad=0.3", fc="white", ec="gray", alpha=0.8),
    )

    plt.tight_layout()
    out = "flight_plots.png"
    plt.savefig(out, dpi=150, bbox_inches="tight")
    print(f"Saved → {out}")
    plt.show()


if __name__ == "__main__":
    filepath = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_FILE
    pressure, accel, time = load_data(filepath)
    plot(pressure, accel, time)