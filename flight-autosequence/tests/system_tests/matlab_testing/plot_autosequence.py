"""
plot_results.py

Plots pressure vs time from a 3-row data file, overlaid with apogee, drogue,
and main deployment timestamps parsed from an autosequence results file.

Usage:
    python plot_results.py data-IDENTIFIER.txt results-IDENTIFIER.txt

Output:
    pressure_results-IDENTIFIER.png
"""

import sys
import os
import re
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.ticker as ticker

# ── Config ────────────────────────────────────────────────────────────────────
DELIMITER      = ","
PRESSURE_UNITS = "Pa"
FIGSIZE        = (14, 5)
# ─────────────────────────────────────────────────────────────────────────────


def derive_output_name(data_filepath: str) -> str:
    """data-ideal.txt → pressure_results-ideal.png"""
    base = os.path.basename(data_filepath)
    name, _ = os.path.splitext(base)
    if "-" in name:
        identifier = name.split("-", 1)[1]
        return f"pressure_results-{identifier}.png"
    return "pressure_results.png"


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
        raise ValueError(f"Expected 3 data rows, found {len(rows)}.")

    pressure, accel, time = rows
    if not (len(pressure) == len(accel) == len(time)):
        raise ValueError("Row lengths differ.")

    print(f"Loaded {len(time)} samples | "
          f"t = [{time[0]:.3f}, {time[-1]:.3f}] s | "
          f"P = [{pressure.min():.1f}, {pressure.max():.1f}] {PRESSURE_UNITS}")
    return pressure, accel, time


def parse_results(filepath: str) -> dict:
    """
    Extract timestamps (ms since ignition) and altitudes from the results file.
    Returns a dict with keys: apogee_ms, drogue_ms, main_ms,
                               apogee_alt, drogue_alt, main_alt.
    Missing values are returned as None.
    """
    patterns = {
        "apogee_ms":  r"Apogee Timestamp\s*:\s*([\d.]+)\s*ms",
        "drogue_ms":  r"Drogue Timestamp\s*:\s*([\d.]+)\s*ms",
        "main_ms":    r"Main Timestamp\s*:\s*([\d.]+)\s*ms",
        "apogee_alt": r"Apogee Altitude\s*:\s*([\d.]+)\s*m",
        "drogue_alt": r"Drogue Deploy Altitude\s*:\s*([\d.]+)\s*m",
        "main_alt":   r"Main Deploy Altitude\s*:\s*([\d.]+)\s*m",
    }

    with open(filepath, "r") as f:
        content = f.read()

    results = {}
    for key, pattern in patterns.items():
        m = re.search(pattern, content)
        results[key] = float(m.group(1)) if m else None

    print(f"Results  | "
          f"Apogee={results['apogee_ms']} ms  "
          f"Drogue={results['drogue_ms']} ms  "
          f"Main={results['main_ms']} ms")
    return results


def add_event_vline(ax, t_s: float, label: str, alt_m, color: str,
                    y_frac: float) -> None:
    """Vertical dashed line with a label showing timestamp and altitude."""
    ax.axvline(t_s, color=color, linewidth=1.3, linestyle="--", alpha=0.9)
    ylim = ax.get_ylim()
    y_pos = ylim[0] + y_frac * (ylim[1] - ylim[0])
    alt_str = f"\n {alt_m:.0f} m" if alt_m is not None else ""
    ax.text(t_s, y_pos,
            f" {label}\n {t_s:.2f} s{alt_str}",
            color=color, fontsize=8, va="top", ha="left",
            bbox=dict(boxstyle="round,pad=0.25", fc="white", ec=color, alpha=0.8))


def plot(pressure: np.ndarray, time: np.ndarray,
         results: dict, out_file: str) -> None:

    # Timestamps in results file are ms since ignition.
    # The data file's time axis starts at t=time[0].
    # Ignition is assumed to be at t=0 in the data (offset if needed below).
    t_ignition = time[0]   # adjust if your data starts before ignition

    def to_data_time(ms):
        return t_ignition + ms / 1000.0 if ms is not None else None

    t_apogee = to_data_time(results["apogee_ms"])
    t_drogue = to_data_time(results["drogue_ms"])
    t_main   = to_data_time(results["main_ms"])

    fig, ax = plt.subplots(1, 1, figsize=FIGSIZE)
    fig.suptitle("Pressure vs Time — Autosequence Events",
                 fontsize=14, fontweight="bold", y=1.01)

    ax.plot(time, pressure, color="#2196F3", linewidth=0.9, label="Static Pressure")
    ax.set_xlabel("Time (s)", fontsize=11)
    ax.set_ylabel(f"Pressure ({PRESSURE_UNITS})", fontsize=11)
    ax.grid(True, linestyle="--", alpha=0.4)
    ax.yaxis.set_major_formatter(ticker.FormatStrFormatter("%.0f"))

    # Draw event lines (after plot so ylims are set)
    events = [
        (t_apogee, "Apogee",  results["apogee_alt"], "#9C27B0", 0.97),
        (t_drogue, "Drogue",  results["drogue_alt"], "#FF9800", 0.82),
        (t_main,   "Main",    results["main_alt"],   "#4CAF50", 0.67),
    ]
    legend_handles = [
        plt.Line2D([0], [0], color="#2196F3", linewidth=0.9, label="Static Pressure"),
    ]
    for t_s, label, alt, color, y_frac in events:
        if t_s is not None:
            add_event_vline(ax, t_s, label, alt, color, y_frac)
            legend_handles.append(
                plt.Line2D([0], [0], color=color, linewidth=1.3,
                           linestyle="--", label=f"{label} ({t_s:.2f} s)")
            )

    ax.legend(handles=legend_handles, fontsize=9, loc="upper right")
    plt.tight_layout()
    plt.savefig(out_file, dpi=150, bbox_inches="tight")
    print(f"Saved -> {out_file}")
    plt.show()


if __name__ == "__main__":
    if len(sys.argv) < 3:
        print("Usage: python plot_results.py data-IDENTIFIER.txt results-IDENTIFIER.txt")
        sys.exit(1)

    data_file    = sys.argv[1]
    results_file = sys.argv[2]
    out_file     = derive_output_name(data_file)

    pressure, _, time = load_data(data_file)
    results           = parse_results(results_file)
    plot(pressure, time, results, out_file)