"""Where a four-wheel run's tyres sit on their force curve. Same as: python tools/rally.py tyre-points RUN ...

  python tools/tyre_points.py out/runs/fw_brake.csv
  python tools/tyre_points.py out/runs/fw_brake.csv --save [PNG] --min-speed 2
"""
import os
import sys

sys.path.insert(0, os.path.dirname(__file__))

from rallytools.cli import main


if __name__ == "__main__":
    sys.exit(main(["tyre-points"] + sys.argv[1:]))
