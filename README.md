# Rally Physics Simulator

![Four-wheel car taking a hairpin](docs/img/fw_hairpin3d.gif)

A small, deterministic rally vehicle simulator written in C++17, with Python
tools for replaying telemetry and inspecting the physics. It is an experiment
in building a vehicle model from first principles: rigid-body dynamics,
load transfer, suspension, wheel rotation, combined-slip tyres, braking and
driving.

The project includes:

- kinematic bicycle, dynamic single-track and four-wheel vehicle models;
- linear, peak-slide and TMeasy-inspired tyre curves;
- scripted scenarios for circles, lane changes, braking, launch and handbrake
  turns;
- finite-state checks, golden tests and CTest coverage;
- CSV telemetry with explicit units and vehicle metadata;
- top-down and 3D replays, tyre plots and animated GIFs.

## Build and test

Requirements: CMake 3.20+, a C++17 compiler, Python and the packages in
[`tools/requirements.txt`](tools/requirements.txt).

```text
cmake -S . -B build
cmake --build build --config Release
ctest --test-dir build -C Release --output-on-failure
```

Install the Python plotting dependencies and generate the showcase:

```text
py -m pip install -r tools/requirements.txt
py tools/run_all.py
```

Use `py tools/run_all.py --gif` to regenerate the animated dashboards. Output
is written under `out/`, which is ignored by git.

## A few outputs

![Four-wheel hairpin](docs/img/fw_hairpin.png)
![Handbrake replay](docs/img/fw_handbrake.png)
![Tyre curves](docs/img/tyre_rig.png)

## Where to look next

- [`docs/tools.md`](docs/tools.md) - plotting and replay tools.
- [`docs/telemetry_schema.md`](docs/telemetry_schema.md) - CSV format.
- [`docs/decisions.md`](docs/decisions.md) - modelling conventions and rationale.
- [`apps/main.cpp`](apps/main.cpp) - available scenarios and the CLI.

This is a research and learning project, not a production-ready driving
simulator. Parameters are deliberately visible and many models are simplified
so that their behaviour can be tested and understood.
