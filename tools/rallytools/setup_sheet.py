"""Format line-1 setup parameters from multiple telemetry runs."""
from __future__ import annotations

from .run import Run


def rows(runs):
    """Return setup keys, values by run, and whether each row differs."""
    runs = list(runs)
    keys = []
    seen = set()
    for run in runs:
        for key in run.meta:
            if key not in seen:
                seen.add(key)
                keys.append(key)

    result = []
    for key in keys:
        values = [run.meta.get(key) if key in run.meta else None for run in runs]
        differs = not all(value is not None and value == values[0] for value in values)
        result.append((key, values, differs))
    return result


def text(runs, changed_only=False):
    """Return a tab-separated setup sheet."""
    runs = list(runs)
    if not runs:
        return ""
    lines = ["\tkey\t" + "\t".join(run.name for run in runs)]
    for key, values, differs in rows(runs):
        if changed_only and not differs:
            continue
        marker = "*" if differs else ""
        rendered = [repr(value) if value is not None else "-" for value in values]
        lines.append(marker + "\t" + key + "\t" + "\t".join(rendered))
    return "\n".join(lines)
