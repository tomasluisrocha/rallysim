# tools/

Python tools that read what the simulator writes and draw it. They never touch the physics: `rallysim` writes CSV
files, these only read them. Everything goes under `out/`, which git ignores, so all of it can be deleted and
remade with `run_all.py`.

Run them from the `sim/` folder with `py tools/<tool>.py` (or `python`); each has `--help`.
They need numpy, pandas and matplotlib, and Pillow for GIFs:

```
py -m pip install numpy pandas matplotlib pillow
```

## Quick start

```
cmake --build build --config Release      # build rallysim first; the tools only run what is built
py tools/run_all.py                       # every scenario + the tyre rig, and all the pictures (about 10 s)
py tools/topdown.py out/runs/fw_handbrake.csv
```

## The tools

| Tool | Does |
|---|---|
| `run_all.py` | Runs every scenario and the rig, then makes the views, the tyre plots and (with `--gif`) the GIFs |
| `topdown.py` | The car seen from above: interactive dashboard, one frame, the whole path, or a GIF |
| `tyre_plot.py` | The tyre rig's force curves, every curve and load in one figure |
| `tyre_points.py` | A four-wheel run's tyres on their force curve, for every wheel and moment |
| `compare.py` | Two runs overlaid channel by channel, with their difference |
| `plot_run.py` | Any columns of a run against any other |
| `simdata.py` | Not a tool: the `out/` paths, reading files, and the tyre curve in Python, shared by the others |

### run_all.py

```
py tools/run_all.py                         every scenario + the rig, then the plots (no GIFs)
py tools/run_all.py --gif                   also the dashboard GIFs (slower: about 45 s each, several at once)
py tools/run_all.py --only fw_brake fw_launch
py tools/run_all.py --snapshot linear       afterwards, keep a copy in out/snapshots/linear
py tools/run_all.py --no-plots              just the CSVs
```

It asks `rallysim list` for the scenario names, so a new scenario added to `apps/main.cpp` is picked up on its own.

### topdown.py

```
py tools/topdown.py out/runs/fw_handbrake.csv                          dashboard: play, pause, scrub
py tools/topdown.py out/runs/fw_handbrake.csv --at 3.0 --save frame.png one dashboard frame
py tools/topdown.py out/runs/fw_handbrake.csv --static --save path.png  the whole path with the car along it
py tools/topdown.py out/runs/fw_handbrake.csv --gif run.gif --start 2 --end 5
```

Dashboard keys: space play/pause, left/right 0.1 s (shift: 1 s), up/down playback speed, home/end.
It reads every kind of run (rigid body, bicycle, single-track, four-wheel) and draws the car from line 1 of the file.

### tyre_plot.py

```
py tools/tyre_plot.py                                  reads out/rig/tyre_rig.csv
py tools/tyre_plot.py --save out/plots/tyres/tyre_rig.png
py tools/tyre_plot.py --curve peak_slide brush --combined brush
```

Fx against slip ratio and Fy against slip angle at each load (colour: load; dashed linear, dotted brush, solid
peak_slide), then the combined sweeps of one curve: the side force falling as the wheel slips, and Fy against Fx
inside the grip circle. Make the CSV with `rallysim tyre_rig` (or `run_all.py`).

### tyre_points.py

```
py tools/tyre_points.py out/runs/fw_brake.csv
py tools/tyre_points.py out/runs/fw_brake.csv --save        to out/plots/tyres/fw_brake.png
```

Every tyre at every moment as a point on its curve: across, the normalised slip (1 is where the linear tyre
reaches its grip); up, the grip used, |F| / mu(Fz) Fz. On these axes every load collapses onto one curve, so the four
wheels share one plot. Hard braking climbs the curve and pins at the locked edge; steady cornering sits on it. The
slips in the file are the steady ones while the force comes from the relaxed ones, so in fast transients the
points loop off the curve (that is the relaxation lag, not an error). Below 1 m/s (`--min-speed`) points are left out.

### compare.py

```
py tools/compare.py A.csv B.csv                               default channels for the kind of run
py tools/compare.py A.csv B.csv speed beta wheel.FL.slipRatio pick the channels
py tools/compare.py A.csv B.csv --labels old new --save out/plots/compare/brake.png
py tools/compare.py --against linear fw_brake                 out/snapshots/linear/runs/fw_brake.csv vs out/runs
py tools/compare.py --against linear --all                    every run in both, into out/plots/compare/
```

A dashed, B solid, B - A underneath each channel. It prints the largest difference of each channel, both final
values, and which line-1 facts (car parameters) differ between the files, so you can see what changed.

### plot_run.py

```
py tools/plot_run.py out/runs/st_circle.csv time pos.z vel.z
py tools/plot_run.py out/runs/st_circle.csv time pos.z --save pos_z.png
```

`yaw` is drawn unwrapped by `plot_run.py` and `compare.py`; `plot_run.py --wrapped` plots it as written. See
[Wrapped and unwrapped yaw](#wrapped-and-unwrapped-yaw) below.

## out/

| Folder | Holds | Written by |
|---|---|---|
| `out/runs/` | One telemetry CSV per scenario (`docs/telemetry_schema.md`) | `rallysim <scenario>` |
| `out/rig/` | `tyre_rig.csv`: force against slip for each tyre curve, no car | `rallysim tyre_rig` |
| `out/plots/views/` | A top-down picture of each run | `topdown.py --static` |
| `out/plots/tyres/` | `tyre_rig.png`, and each four-wheel run's tyres on their curve | `tyre_plot.py`, `tyre_points.py` |
| `out/plots/compare/` | Two runs overlaid, with their difference | `compare.py --save` |
| `out/gif/` | The dashboard of each run as an animation | `topdown.py --gif` |
| `out/snapshots/<name>/` | Saved copies of `runs/` and `rig/`, to compare later runs against | `run_all.py --snapshot <name>` |
| `out/scratch/` | One-off experiments | by hand; nothing writes here on its own |

`rallysim <scenario> [path]` writes to `out/runs/<scenario>.csv` (the rig to `out/rig/`) unless given a path;
`rallysim list` prints the scenario names.

## A typical check after a physics change

```
py tools/run_all.py --snapshot before          # before the change: keep the old results
(make the change, rebuild)
py tools/run_all.py                            # the new results and pictures
py tools/compare.py --against before --all     # what moved, run by run, into out/plots/compare/
```

## Keeping the tools in step with the C++

- Look columns up by name, never by position (`docs/telemetry_schema.md`).
- `simdata.py` repeats the tyre curve of `vehicle/tyre.hpp` (`grip`, `curve_shape`) so `tyre_points.py` can draw it.
  If the C++ curve changes, change it there too: `tyre_points.py` warns when the two differ from
  `out/rig/tyre_rig.csv` by more than 1 N.
- A new tyre parameter the tools need goes on line 1 of the four-wheel files (`four_wheel_meta`) and into
  `simdata.TYRE_DEFAULTS`.

## Wrapped and unwrapped yaw

`yaw` is the car's heading in the world frame: the angle from the world x axis, counter-clockwise. The single-track
and four-wheel files take it from the body's orientation (`yaw_angle` in `core/rigid_body.hpp`, an `atan2`), so it
is always between -pi and pi, like a compass: after 179 degrees comes -179 degrees, the same direction a little
further round. The bicycle integrates its yaw (`yaw += r dt`), so its files keep counting.

Both say where the car points; they only label it differently. Going round a circle:

```
time ->        lap 1                         lap 2
wrapped:    0 -> 90 -> 180 | -180 -> -90 -> 0 -> 90 -> 180 | -180 ...
unwrapped:  0 -> 90 -> 180 ->  180 ->  270 -> 360 -> 450 -> 540 ...
```

Wrapped, it plots as a sawtooth, one tooth per lap. Unwrapped (`np.unwrap`, which removes each 2 pi jump), the angle
keeps counting, like an odometer, and plots as one continuous line whose slope is the yaw rate.

| To see | Use |
|---|---|
| How many turns the car made, how smoothly it turns | unwrapped (the default in `plot_run.py` and `compare.py`) |
| Which way the car points now, as a compass direction | wrapped (`plot_run.py --wrapped`) |
| Two runs compared | unwrapped: no false spikes when one run crosses +-pi a moment before the other |

The CSV keeps the wrapped value, the usual convention for a stored heading; only the plots unwrap it
(`simdata.unwrap_headings`). `compare.py` also wraps the difference B - A back into -pi to pi, so two runs a whole
lap apart do not show a 2 pi difference. `topdown.py` only uses yaw to turn the car drawing, where it makes no
difference.
