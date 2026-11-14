"""Shared helpers for the tools: the out/ folder layout, reading telemetry, and the tyre curve.

The tyre curve lives in rallytools/tyre.py; its names are re-exported here for the older scripts.
"""
import math
import re
import subprocess
import sys
from pathlib import Path

import numpy as np
import pandas as pd

from rallytools.paths import (COMPARE, GIFS, OUT, PLOTS, RIG, ROOT, RUNS, SCRATCH, SNAPSHOTS, SWEEP_PLOTS,  # noqa: F401
                              SWEEPS, TYRES, VIEWS, WHEELS, label_of, run_command)
from rallytools import paths as _paths


def find_exe(given=None, root=ROOT):
    """The rallysim executable: `given`, or the first build found under build/ (Release first)."""
    try:
        return _paths.find_exe(given, root)
    except ValueError as error:
        sys.exit(str(error))


def read_units(path):
    """{column: unit} from line 1 of a telemetry file ("name[unit]" after "units:")."""
    with open(path, encoding="utf-8") as f:
        first = f.readline()
    units = {}
    if "units:" in first:
        text_after_units = first.split("units:", 1)[1]
        units = {name: unit for name, unit in re.findall(r"(\S+?)\[([^\]]*)\]", text_after_units)}
    return units


def read_meta(path):
    """key=value pairs on line 1 of a telemetry file, between dt and the units (numbers only)."""
    with open(path, encoding="utf-8") as f:
        first = f.readline()
    meta = {}
    if first.startswith("#"):
        for token in first[1:].split("units:")[0].split():
            key, sep, value = token.partition("=")
            if sep:
                try:
                    meta[key] = float(value)
                except ValueError:
                    pass
    return meta


def read_run(path):
    """(data, meta) of a telemetry file."""
    return pd.read_csv(path, comment="#"), read_meta(path)


from rallytools.overhead import CAR_DEFAULTS, with_car_defaults  # noqa: E402,F401


HEADINGS = ("yaw",)  # world-frame headings: atan2 of the orientation, so they jump between +pi and -pi


def unwrap_headings(data):
    """Remove the 2 pi jumps from the heading columns, in place, so a car going round a circle shows a heading
    that keeps climbing (as the bicycle's integrated yaw does) instead of a sawtooth. Same heading, plotted
    continuously; a file that is already continuous is unchanged."""
    for column in HEADINGS:
        if column in data.columns:
            data[column] = np.unwrap(data[column].to_numpy(dtype=float))
    return data


def is_four_wheel(data):
    return "wheel.FL.slipRatio" in data.columns

from rallytools.tyre import (TMEASY_DEFAULTS, TMEASY_FIELDS, TYRE_DEFAULTS, CURVES, axis_edge, check_against_rig,  # noqa: E402,F401
                             curve_name, curve_shape, grip, marks, normalised_slip, pure_slip_curve,
                             reference_curves, shape, slip_axis_label, tmeasy_at, tmeasy_combine, tyre_forces,
                             tyre_from_meta)
