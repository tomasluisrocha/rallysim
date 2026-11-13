"""Plot the single-tyre rig: force against slip for each tyre curve.

A wrapper for `python tools/rally.py tyre-rig`; see its --help. Make the CSV first with:
    build/Release/rallysim tyre_rig
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from rallytools.cli import main  # noqa: E402

if __name__ == "__main__":
    sys.exit(main(["tyre-rig", *sys.argv[1:]]))
