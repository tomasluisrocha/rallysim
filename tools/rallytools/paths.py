"""The out/ folder layout, finding rallysim, and running it."""
from __future__ import annotations

import subprocess
from pathlib import Path

from .run import WHEELS  # noqa: F401  (re-exported)

ROOT = Path(__file__).resolve().parent.parent.parent  # the sim/ folder
OUT = ROOT / "out"
RUNS = OUT / "runs"            # one telemetry CSV per scenario (rallysim <scenario>)
RIG = OUT / "rig"              # tyre_rig.csv (rallysim tyre_rig)
PLOTS = OUT / "plots"
VIEWS = PLOTS / "views"        # topdown --static, one per run
TYRES = PLOTS / "tyres"        # the rig plot, and each four-wheel run's tyres on their curve
COMPARE = PLOTS / "compare"    # compare.py
SWEEP_PLOTS = PLOTS / "sweeps" # sweep.py: one picture per sweep
SWEEPS = OUT / "sweeps"        # sweep.py: each sweep's runs and its summary table, one folder per sweep
GIFS = OUT / "gif"             # topdown --gif (<run>.gif) and view3d --gif (<run>_3d.gif)
SNAPSHOTS = OUT / "snapshots"  # saved copies of runs/ and rig/ to compare against (run_all.py --snapshot NAME)
SCRATCH = OUT / "scratch"      # one-off experiments; nothing writes here on its own

_CANDIDATES = ("build/Release/rallysim.exe", "build/rallysim.exe", "build/Release/rallysim", "build/rallysim",
               "build/Debug/rallysim.exe", "build/Debug/rallysim")


def find_exe(given=None, root=ROOT):
    """The rallysim executable: `given`, or the first build found under build/ (Release first)."""
    if given:
        return Path(given)
    for candidate in _CANDIDATES:
        path = Path(root) / candidate
        if path.exists():
            return path
    raise ValueError("rallysim not found under build/; build it first or pass --exe")


def run_command(command, cwd=ROOT):
    """Run one command from the sim folder; (ok, text) with its output."""
    done = subprocess.run([str(c) for c in command], cwd=cwd, capture_output=True, text=True)
    return done.returncode == 0, (done.stdout + done.stderr).strip()


def label_of(path):
    """A short name for a run file: 'fw_brake', or 'linear/fw_brake' for a snapshot."""
    path = Path(path).resolve()
    try:
        parts = path.relative_to(SNAPSHOTS).parts
        return f"{parts[0]}/{path.stem}"
    except ValueError:
        return path.stem
