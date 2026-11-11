import os
import sys

sys.path.insert(0, os.path.dirname(__file__))

from rallytools.cli import main


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
