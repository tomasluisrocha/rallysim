# tools/

Python tools that read what the simulator writes and draw it. They never touch the physics: `rallysim` writes CSV
files, these only read them. Everything goes under `out/`, which git ignores, so all of it can be deleted and
remade with `run_all.py`.

Run them from the `sim/` folder with `py tools/<tool>.py` (or `python`); each has `--help`.
They need numpy, pandas and matplotlib, and Pillow for GIFs, all listed in `tools/requirements.txt`:

```
py -m pip install -r tools/requirements.txt
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
| `view3d.py` | A four-wheel run in 3D (isometric by default): the body's roll, pitch and ride height, and every force where it acts |
| `tyre_plot.py` | The tyre rig's force curves, every curve and load in one figure |
| `tyre_points.py` | A four-wheel run's tyres on their force curve, for every wheel and moment |
| `compare.py` | Two runs overlaid channel by channel, with their difference |
| `plot_run.py` | Any columns of a run against any other |
| `simdata.py` | Not a standalone tool: shared paths, file readers and tyre curves used by the other tools |
| `input_bars.py` | Not a standalone tool: the driver's inputs as bar charts, for `topdown.py` and `view3d.py` |

### run_all.py

```
py tools/run_all.py                         every scenario + the rig, then the plots (no GIFs)
py tools/run_all.py --gif                   also the dashboard GIFs (slower: about 45 s each, several at once)
py tools/run_all.py --only fw_brake fw_launch
py tools/run_all.py --snapshot linear       afterwards, keep a copy in out/snapshots/linear
py tools/run_all.py --no-plots              just the CSVs
```

It asks `rallysim list` for the scenario names, so a new scenario added to `apps/main.cpp` is picked up on its own.
A scenario whose state stops being finite (a NaN or infinity, `core/finite.hpp`) is listed as FAILED with
`rallysim`'s message (the field, step and time); its CSV ends where the run stopped, and the script exits with 1.

The generated GIFs and plots are the quickest way to browse a complete run:

![Generated four-wheel scenario output](img/fw_hairpin.png)

### topdown.py

```
py tools/topdown.py out/runs/fw_handbrake.csv                          dashboard: play, pause, scrub
py tools/topdown.py out/runs/fw_handbrake.csv --at 3.0 --save frame.png one dashboard frame
py tools/topdown.py out/runs/fw_handbrake.csv --static --save path.png  the whole path with the car along it
py tools/topdown.py out/runs/fw_handbrake.csv --gif run.gif --start 2 --end 5
```

Dashboard keys: space play/pause, left/right 0.1 s (shift: 1 s), up/down playback speed, home/end.
It reads every kind of run (rigid body, bicycle, single-track, four-wheel) and draws the car from line 1 of the file.

The dashboard shows the car on its path with each tyre's force, coloured by the grip in use, and at the CG the sum of
the tyre forces (orange) and their yaw moment (purple arc, longer for a larger moment). Around it: the whole run, the
g-g diagram (the car's acceleration in g, from the tyre forces over the mass), the load on each wheel, the driver's
inputs (the pedals upright, the steer across, left of zero when steering left), and sideslip, yaw rate and speed
over time. The static view (`--static`) draws only the tyre forces.

![Top-down replay](img/tool-topdown.gif)

### view3d.py

```
py tools/view3d.py out/runs/fw_handbrake.csv                           play, pause, scrub, drag to turn the view
py tools/view3d.py out/runs/fw_brake.csv --at 1.6 --save brake3d.png    one frame
py tools/view3d.py out/runs/fw_handbrake.csv --gif out/gif/fw_handbrake_3d.gif --start 2.3 --end 4.3
py tools/view3d.py out/runs/fw_circle.csv --chase --perspective        behind the car, perspective
```

The vertical side of a run, which `topdown.py` cannot show: the body at its real pose (roll, pitch and ride height,
exaggerated x4 by default, `--exaggerate 1` for true scale; the HUD gives the true angles), wheels that steer and spin
(a locked wheel's spoke stops), struts coloured by how loaded each corner is. At each contact patch: the vertical load
(blue, standing just outboard of the wheel), the tyre force on the ground (black), and the grip mu(Fz) Fz as a disc
coloured by the share in use, so the black arrow reaching the disc's edge is the tyre at its limit; at the CG, the
velocity (green), the sum of the tyre forces (orange) and their yaw moment (purple arc, longer for a larger moment).
Every force uses one scale, printed in the HUD, and is drawn over the body and wheels so none hides behind the car.
The HUD also shows the driver's inputs as bars. The camera is isometric (a parallel projection, so arrows keep their
true length anywhere in the picture); `p` switches to perspective, `c` to a chase camera. Four-wheel runs only. GIFs
take about 7 s per second of run.

![3D replay](img/tool-view3d.gif)

### tyre_plot.py

```
py tools/tyre_plot.py                                  reads out/rig/tyre_rig.csv
py tools/tyre_plot.py --save out/plots/tyres/tyre_rig.png
py tools/tyre_plot.py --curve peak_slide                      every panel on peak_slide
py tools/tyre_plot.py --curve peak_slide tmeasy --combined peak_slide
```

Fx against slip ratio and Fy against slip angle at each load (colour: load; dashed linear, dotted brush, dash-dot
peak_slide, solid tmeasy), then the combined sweeps of one curve (`--combined`, else the first `--curve`, else tmeasy,
the car's tyre): the side force
falling as the wheel slips, and Fy against Fx inside the grip circle. Make the CSV with `rallysim tyre_rig` (or
`run_all.py`). Comparing peak_slide and tmeasy here shows what the two-load tables buy: tmeasy's side force peaks at
about 9-10 degrees at every load, peak_slide's moves from 6 to 20.

Use this tool to study the tyre in isolation. The top-left and top-right
panels are pure longitudinal and pure lateral tests: they show how stiffness,
peak grip and sliding behaviour change with vertical load. The lower-left panel
shows combined slip, where braking or driving consumes some of the force
available for cornering. The lower-right panel shows the same result as a
force envelope; every trace should remain inside the grey `mu Fz` grip circle.

![Tyre force curves](img/tool-tyre-plot.png)

### tyre_points.py

```
py tools/tyre_points.py out/runs/fw_brake.csv
py tools/tyre_points.py out/runs/fw_brake.csv --save        to out/plots/tyres/fw_brake.png
```

Every tyre at every moment as a point on its curve: across, the normalised slip; up, the grip used, |F| / mu(Fz) Fz.
For linear and peak_slide tyres the slip is scaled so 1 is where the linear tyre reaches its grip, and every load
collapses onto one curve. For tmeasy tyres (the car's) it is sigma, each slip over its own peak slip, so 1 is the peak
in every direction; each direction has its own curve, drawn at the nominal load, and mixed slips sit between them.
Hard braking climbs the curve and pins at the locked edge; steady cornering sits on it. The
slips in the file are the steady ones while the force comes from the relaxed ones, so in fast transients the
points loop off the curve (that is the relaxation lag, not an error). Below 1 m/s (`--min-speed`) points are left out.

Use this tool to study a complete four-wheel run rather than the tyre model in
isolation. The top-left panel answers "how close is each wheel to its tyre
curve?"; the top-right shows how each wheel shares its available grip between
longitudinal and lateral force. The bottom panels show when slip builds and
how much grip is used over time. This makes braking, corner entry, wheelspin
and handbrake events easy to identify, while the wheel colours show which
corner is responsible.

![Tyre operating points](img/tool-tyre-points.png)

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

![Run comparison](img/tool-compare.png)

### plot_run.py

```
py tools/plot_run.py out/runs/st_circle.csv time pos.z vel.z
py tools/plot_run.py out/runs/st_circle.csv time pos.z --save pos_z.png
py tools/plot_run.py out/runs/fw_brake.csv time energy.residual          the energy books' residual
py tools/plot_run.py out/runs/fw_launch.csv time work.drive work.tyre energy.total   where the energy went
```

Every four-wheel run carries its energy books (`energy.*`, `work.*`, `energy.residual`; `telemetry_schema.md`).
After a physics change, a residual that grows with time and does not halve when dt halves points at a bug.

`yaw` is drawn unwrapped by `plot_run.py` and `compare.py`; `plot_run.py --wrapped` plots it as written. See
[Wrapped and unwrapped yaw](#wrapped-and-unwrapped-yaw) below.

![Energy plot](img/tool-plot-run.png)

## out/

| Folder | Holds | Written by |
|---|---|---|
| `out/runs/` | One telemetry CSV per scenario (`telemetry_schema.md`) | `rallysim <scenario>` |
| `out/rig/` | `tyre_rig.csv`: force against slip for each tyre curve, no car | `rallysim tyre_rig` |
| `out/plots/views/` | A top-down picture of each run | `topdown.py --static` |
| `out/plots/tyres/` | `tyre_rig.png`, and each four-wheel run's tyres on their curve | `tyre_plot.py`, `tyre_points.py` |
| `out/plots/compare/` | Two runs overlaid, with their difference | `compare.py --save` |
| `out/gif/` | The dashboard of each run as an animation | `topdown.py --gif` |
| `out/snapshots/<name>/` | Saved copies of `runs/` and `rig/`, to compare later runs against | `run_all.py --snapshot <name>` |

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

- Look columns up by name, never by position (`telemetry_schema.md`).
- `simdata.py` repeats the tyre of `vehicle/tyre.hpp` (`grip`, `tyre_forces` for every curve, `tmeasy_at`,
  `tmeasy_combine`, `pure_slip_curve`) so `tyre_points.py` can put the tyres on their curves. If the C++ tyre changes,
  change it there too: `tyre_points.py` warns when the two differ by more than 1 N on any row of
  `out/rig/tyre_rig.csv` (every curve, sweep and load).
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
