"""Run every scenario and make every picture; the same as `python tools/rally.py run-all`."""
import sys

from rallytools.cli import main

if __name__ == "__main__":
    sys.exit(main(["run-all", *sys.argv[1:]]))
