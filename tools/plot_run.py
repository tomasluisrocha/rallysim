"""Plot columns from a telemetry CSV. A wrapper for `python tools/rally.py columns`.

Usage:
    python tools/plot_run.py out/runs/st_circle.csv time speed yaw_rate
    python tools/plot_run.py out/runs/st_circle.csv time speed --save out/scratch/speed.png
"""
import sys

from rallytools.cli import main

if __name__ == "__main__":
    sys.exit(main(["columns"] + sys.argv[1:]))
