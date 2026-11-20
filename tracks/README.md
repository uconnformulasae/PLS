# Tracks

Track centre lines, one `.csv` per track, grouped by competition: `michigan_2026/` (FSAE Michigan 2026, surveyed by
the team) and `2024/`. A car uses a track through its car file:

```yaml
vehicle:
  events:
    autocross_track: ../../tracks/michigan_2026/autocross_2026.csv
    endurance_track: ../../tracks/michigan_2026/endurance_2026_full_site.csv
```

The acceleration (75 m straight) and the skidpad (9.125 m circle, the FSAE rules) are built from `events.accel_length`
and `events.skidpad_radius`; their files here are for drawing only. The website lists every `.csv` in this folder in
its track picker (by file name, so keep names unique).

## Format

Columns `s_m, x_m, y_m, curvature_1pm` (distance, plan view, curvature + left), as the files here; or just `x_m, y_m`
(smoothed into a centre line), or `lat, lon` in degrees. A RaceStudio `.csv` export of a logged run also works: the
course is rebuilt from GPS and the IMU (`lapsim/validation/logs.py`).
