# Contracts

What each part of the simulation promises the others. Change one of these only together with everything
that relies on it, and say so in `docs/decision_records.md`.

## Units

SI everywhere inside the code (m, kg, N, s, rad, W, J, Pa, V, A, ohm). Exceptions, kept for readability:
fields ending in `_deg` are degrees, temperatures are degC, engine and motor speeds are rpm. A car file may
give any value as a number in that SI unit or as a string with a unit (`"350 lbf/in"`, `"11 in"`, `"80 kW"`,
`"14 mOhm"`); `lapsim/config/units.py` converts it. No physical constant of the car goes in the code: add it
to the car file and the schema.

## Car files

One YAML file per car, `cars/<car>/vehicle.yaml`, with the top-level keys

| key | what |
|---|---|
| `name`, `kind` | the car's name; `ic` or `ev` |
| `base` | (optional) another car file whose `vehicle:` section this one starts from |
| `confirmed` | (with `base`) dotted keys of `vehicle:` values that are real measurements |
| `vehicle:` | chassis, tires, suspension, aero, brakes, events, solver settings: `config/schema.py` |
| `powertrain:` | IC: `config/schema_ic.py`; EV: `config/schema_ev.py` (`EVConfig`, with its own `confirmed`) |
| `pack:` | EV only: `config/schema_ev.py` (`PackConfig`) |

File paths inside a section are relative to the car file. An unknown key is an error, so a typo stops the
load instead of being ignored. Each schema field carries `unit`, `note` and `source` metadata
(`lapsim register <car>` lists them).

## Override keys

Every place that changes a value for one run uses the same key: `<prefix>:<dotted.path>` with prefix

| prefix | object | example |
|---|---|---|
| `vehicle` | `vehicle:` section | `vehicle:front.spring_rate` |
| `pt` | IC `powertrain:` section | `pt:final_drive.rear_sprocket` |
| `ev` | EV `powertrain:` section | `ev:strategy.power_limit` |
| `pack` | EV `pack:` section | `pack:cell.r0` |

`Car.set`, `lapsim run --set`, the notebooks' sliders and the website (run, sweep, export, team designs)
all use these keys. They are the contract with designs already published from the website: renaming a
field breaks those designs, so keep a field's name once it has been on the site. (The website skips a key
the model no longer has and says so in the run's notes instead of failing.)

## Power modules (`lapsim/subsystems/power/`)

`base.py` is the contract. A power module subclasses `core.vehicle.Vehicle` and sets, after
`Vehicle.__init__`:

| attribute | what |
|---|---|
| `pt` | drive-force limit: `force(v)` [N] at the rear contact patches, `unlimited` switch |
| `dt` | the driveline; the chassis reads `dt.r` (rolling radius used by the differential) |
| `diff` | `subsystems.driveline.Differential` |
| `ptc` | the module's configuration |
| `coast_torque(v)` | axle torque on the differential's coast side while braking |

and its solver (`solvers/qss_ic.py`, `solvers/qss_ev.py`) provides `solve(track, veh, tggv, tables, v0, flying)`
and `run_events(...) -> core.events.EventResults`.

## Results

`EventResults` (`core/events.py`): `times`, `points`, `laps`, `endurance`, `tggv`, `vehicle`, `tables`,
`notes`, `table()`. Every lap result is a `core.results.LapResult` (`s, t, v, ax, ay, state, time,
energy_J`, `frame()`) and the power modules' laps add `pt` (powertrain / accumulator channels), `limit`
(what limits the car at each point: braking, cornering, traction, power, ...) and `summary`.
`write_run` writes a run to a folder (`events.csv`, `endurance.csv`, `laps/*.csv`, `metadata.json` with
the car file's hash and the git commit); `validation.compare` reads two back.

## Website bridge (`assets/py/lapsim_web.py` on the `web/lapsim-site` branch)

The page (`assets/js/*.js`) calls these functions, in the browser through Pyodide and natively in
`.github/build.py`, and receives plain JSON: `schema`, `run`, `sweep`, `ev_studies`, `tracks`, `geometry`,
`preview`, `brake_stops`, `airfoil`, `wing_stack`, `export_files`, `import_hardpoints`, `import_frame`,
`log_import`, `sensitivity`, `optimise`, `trackside`, `transient`. Their names, arguments and the shape of
what they return are the contract with the page; the bridge is the only place that knows both sides.
