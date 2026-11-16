#!/usr/bin/env python3
"""Plot roll and pitch from a telemetry CSV.

Install: python -m pip install matplotlib
Example: python Sim/plot_telemetry.py Sim/telemetry_20260101_120000_000000.csv
Save without opening a window: python Sim/plot_telemetry.py capture.csv --save plot.png --no-show
"""

import argparse
import csv
import math
from pathlib import Path
import sys


def load_telemetry(path):
    """Load validated readings, retaining their order and firmware timestamps."""
    times, rolls, pitches = [], [], []
    with path.open(newline="", encoding="utf-8-sig") as source:
        reader = csv.DictReader(source, skipinitialspace=True)
        required = {"time_ms", "roll_deg", "pitch_deg"}
        if reader.fieldnames is None or not required.issubset(reader.fieldnames):
            raise ValueError("CSV must have time_ms,roll_deg,pitch_deg columns")
        for row in reader:
            try:
                values = [float(row[key]) for key in ("time_ms", "roll_deg", "pitch_deg")]
                if not all(math.isfinite(value) for value in values) or values[0] < 0:
                    raise ValueError("non-finite value or negative timestamp")
            except (ValueError, TypeError) as exc:
                raise ValueError(f"Invalid telemetry at CSV line {reader.line_num}") from exc
            times.append(values[0] / 1000.0)
            rolls.append(values[1])
            pitches.append(values[2])
    if not times:
        raise ValueError("CSV contains no readings")
    return times, rolls, pitches


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("csv_file", type=Path, help="Telemetry CSV to visualize")
    parser.add_argument("--save", type=Path, help="Save plot to an image, e.g. plot.png")
    parser.add_argument("--no-show", action="store_true", help="Do not open a plot window (requires --save)")
    args = parser.parse_args()
    if args.no_show and args.save is None:
        parser.error("--no-show requires --save")

    try:
        times, rolls, pitches = load_telemetry(args.csv_file)
    except (OSError, ValueError, csv.Error) as exc:
        print(f"Cannot read telemetry: {exc}", file=sys.stderr)
        return 1

    try:
        import matplotlib
        if args.no_show:
            matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        print("Missing dependency: run python -m pip install matplotlib", file=sys.stderr)
        return 1

    fig, ax = plt.subplots(figsize=(10, 5), layout="constrained")
    ax.plot(times, rolls, label="Roll", color="tab:blue")
    ax.plot(times, pitches, label="Pitch", color="tab:orange")
    ax.set(title=args.csv_file.name, xlabel="Firmware time (s)", ylabel="Angle (degrees)")
    ax.grid(True, alpha=0.3)
    ax.legend()
    try:
        if args.save:
            args.save.parent.mkdir(parents=True, exist_ok=True)
            fig.savefig(args.save, dpi=150)
            print(f"Saved plot to {args.save}")
        if not args.no_show:
            plt.show()
    except (OSError, ValueError) as exc:
        print(f"Cannot render plot: {exc}", file=sys.stderr)
        return 1
    finally:
        plt.close(fig)
    return 0


if __name__ == "__main__":
    sys.exit(main())
