"""Generate the small telemetry fixtures used by the tools tests."""
from __future__ import annotations

import argparse
import os
from pathlib import Path
import re
import subprocess
import tempfile


ROOT = Path(__file__).resolve().parents[2]
FIXTURES = Path(__file__).resolve().parent / "fixtures"
RUNS = (
    ("rigid_body.csv", ("thrown",)),
    ("bicycle.csv", ("circle",)),
    ("single_track.csv", ("st_lane",)),
    ("four_wheel_springs.csv", ("fw_brake",)),
    ("four_wheel_unsprung.csv", ("fw_brake", "--set", "vertical=unsprung")),
)
REMOVED = tuple(
    f"wheel.{wheel}.{column}"
    for wheel in ("FL", "FR", "RL", "RR")
    for column in ("tyreDefl", "wheelZ", "stopF")
)


def executable(value):
    if value:
        return Path(value)
    for candidate in (ROOT / "build" / "Release" / "rallysim.exe", ROOT / "build" / "rallysim"):
        if candidate.exists():
            return candidate
    raise SystemExit("rallysim executable not found; use --exe")


def run_fixture(exe, arguments):
    with tempfile.TemporaryDirectory() as temporary:
        output = Path(temporary) / "run.csv"
        result = subprocess.run(
            [os.fspath(exe), *arguments, os.fspath(output)],
            cwd=ROOT,
            capture_output=True,
            text=True,
        )
        if result.returncode:
            raise SystemExit(result.stderr.strip() or f"rallysim failed for {' '.join(arguments)}")
        with output.open("r", encoding="utf-8", newline="") as source:
            lines = [source.readline() for _ in range(102)]
    if len(lines) != 102 or any(line == "" for line in lines):
        raise SystemExit(f"rallysim output for {' '.join(arguments)} has fewer than 102 lines")
    return "".join(lines)


def write_fixture(name, text):
    FIXTURES.mkdir(exist_ok=True)
    with (FIXTURES / name).open("w", encoding="utf-8", newline="\n") as destination:
        destination.write(text)


def schema5(text):
    lines = text.splitlines(keepends=True)
    metadata = lines[0].replace("schema=6", "schema=5").replace("vertical=1 ", "")
    for column in REMOVED:
        metadata = re.sub(rf"\s+{re.escape(column)}\[[^]]*\]", "", metadata)
    header = lines[1].rstrip("\n").split(",")
    indices = [index for index, column in enumerate(header) if column in REMOVED]
    keep = [index for index in range(len(header)) if index not in indices]
    lines[0] = metadata
    for line_number in range(1, len(lines)):
        ending = "\n" if lines[line_number].endswith("\n") else ""
        fields = lines[line_number].rstrip("\n").split(",")
        lines[line_number] = ",".join(fields[index] for index in keep) + ending
    return "".join(lines)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--exe")
    args = parser.parse_args()
    exe = executable(args.exe)
    for name, command in RUNS:
        write_fixture(name, run_fixture(exe, command))
    write_fixture("four_wheel_schema5.csv", schema5((FIXTURES / "four_wheel_springs.csv").read_text(
        encoding="utf-8"
    )))


if __name__ == "__main__":
    main()
