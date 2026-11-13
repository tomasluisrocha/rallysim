"""Run one scenario over a range of one setting and plot how the results change with it.

A wrapper: `python tools/sweep.py ...` is `python tools/rally.py sweep ...`.
"""
import os
import sys

sys.path.insert(0, os.path.dirname(__file__))

from rallytools.cli import main


if __name__ == "__main__":
    sys.exit(main(["sweep", *sys.argv[1:]]))
