import os
from pathlib import Path

import pandas as pd


def write_run(path, data, schema=6, dt=0.001, meta=None, units=None):
    """Write a small telemetry file in the format emitted by rallysim."""
    path = Path(path)
    meta = meta or {}
    units = units or {}
    facts = [f"schema={schema}", f"dt={repr(dt)}"]
    facts.extend(f"{key}={repr(value)}" for key, value in meta.items())
    unit_text = " ".join(f"{column}[{units.get(column, '')}]" for column in data.columns)
    lines = [f"# {' '.join(facts)} units: {unit_text}\n",
             ",".join(data.columns) + "\n"]
    for row in data.itertuples(index=False, name=None):
        lines.append(",".join(repr(value) for value in row) + "\n")
    path.write_text("".join(lines), encoding="utf-8", newline="")


def need(test, condition, why):
    if condition:
        return
    if os.environ.get("RALLYTOOLS_CI") == "1":
        test.fail(why)
    test.skipTest(why)
