# Telemetry schema (version 1)

One CSV file per run. Text, comma-separated, one row per physics step.

## Layout
- Line 1: comment starting with `#`: schema version, dt, then each column with its unit.
- Line 2: column names.
- Line 3 onward: one row per step, one number per column, 17 significant digits.

Example:
```
# schema=1 dt=0.001 units: step[count] time[s] pos.x[m] pos.y[m] pos.z[m] ...
step,time,pos.x,pos.y,pos.z,...
0,0,0,0,100,...
1,0.001,0,0,99.99999...,...
```

## Columns
| Name | Unit | Meaning |
|---|---|---|
| step | count | step index, starts at 0 |
| time | s | step * dt |
| pos.x, pos.y, pos.z | m | position, world frame |
| vel.x, vel.y, vel.z | m/s | velocity, world frame |
| quat.w, quat.x, quat.y, quat.z | - | orientation, body to world |
| omega.x, omega.y, omega.z | rad/s | angular velocity, body frame |

## Rules
- Names are dotted paths: group.component (`pos.x`, later `wheel.FL.Fz`).
- Never rename a column or change its meaning or unit without bumping the schema
  version and writing a converter for old files.
- New columns may be appended; plotting scripts must look columns up by name,
  never by position.
- Store SI units. Convert (kelvin to Celsius, rad to degrees) only when plotting.