"""One telemetry file read into one Run. numpy and pandas only."""
from __future__ import annotations

import io
import os
import re
from dataclasses import dataclass
from typing import Optional

import numpy as np
import pandas as pd

KINDS = ("rigid_body", "bicycle", "single_track", "four_wheel")
WHEELS = ("FL", "FR", "RL", "RR")

_UNIT = re.compile(r"([^\s\[\]]+)\[([^\]]*)\]")
_VERTICAL = {0: "planar", 1: "springs", 2: "unsprung"}


@dataclass(frozen=True)
class Run:
    name: str
    kind: str
    schema: Optional[int]
    dt: Optional[float]
    meta: dict
    units: dict
    data: pd.DataFrame

    def column(self, name):
        if name not in self.data.columns:
            raise ValueError(f"run '{self.name}': no column '{name}'")
        return self.data[name].to_numpy(dtype=float)

    def has(self, name):
        return name in self.data.columns

    @property
    def vertical(self):
        if self.kind != "four_wheel":
            return None
        if "vertical" in self.meta:
            return _VERTICAL.get(int(self.meta["vertical"]))
        if "springs" in self.meta:
            return "springs" if self.meta["springs"] == 1 else "planar"
        return None


def _kind(columns, where):
    cols = set(columns)
    if "quat.w" in cols:
        return "rigid_body"
    if "wheel.FL.slipRatio" in cols:
        return "four_wheel"
    if "axle.front.Fy" in cols:
        return "single_track"
    if "yaw" in cols and "speed" in cols:
        return "bicycle"
    raise ValueError(f"{where}: not a telemetry file this library knows (no quat.w, wheel, axle or yaw/speed columns)")


def _parse_line1(line):
    schema, dt, meta, units = None, None, {}, {}
    head, _, tail = line[1:].partition("units:")
    for token in head.split():
        key, sep, value = token.partition("=")
        if not sep:
            continue
        try:
            number = float(value)
        except ValueError:
            continue
        if key == "schema":
            schema = int(number)
        elif key == "dt":
            dt = number
        else:
            meta[key] = number
    units = {n: u for n, u in _UNIT.findall(tail)}
    return schema, dt, meta, units


def load(source, name=None):
    if hasattr(source, "read"):
        text = source.read()
        where = name or "run"
    else:
        with open(source, encoding="utf-8") as f:
            text = f.read()
        where = os.fspath(source)
        if name is None:
            name = os.path.splitext(os.path.basename(where))[0]
    name = name or "run"
    first = text.split("\n", 1)[0]
    schema, dt, meta, units = _parse_line1(first) if first.startswith("#") else (None, None, {}, {})
    try:
        data = pd.read_csv(io.StringIO(text), comment="#")
    except (pd.errors.ParserError, pd.errors.EmptyDataError) as e:
        raise ValueError(f"{where}: cannot read as telemetry: {e}") from e
    return Run(name, _kind(data.columns, where), schema, dt, meta, units, data)


def from_frame(data, kind=None, name="run", schema=None, dt=None, meta=None, units=None):
    if kind is None:
        kind = _kind(data.columns, name)
    elif kind not in KINDS:
        raise ValueError(f"run '{name}': unknown kind '{kind}'")
    return Run(name, kind, schema, dt, dict(meta or {}), dict(units or {}), data)
