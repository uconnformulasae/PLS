# Predictive LapSim

A lap simulation for the team's Formula SAE cars (ICE and EV). It predicts lap time, speed, energy use, pack current, state of charge and pack temperature for the acceleration, skidpad, autocross and endurance events.

> **Project Status:** under active development (2026–27 season). The quasi-steady-state (QSS) simulation of the CT-17 IC and CT-17 EV runs from this repository as one package, `lapsim`, with the same results as before the move. A time-domain check, notebooks for every subsystem and the website run on the same package. Telemetry validation, higher-fidelity vehicle models and batch computing come next; see [Next steps](#next-steps).

## What it does

- **QSS solver** (`--solver qss`): the calibrated simulation the team has used for the events, built from separate subsystem modules: tires, aero, suspension kinematics from the hard points, brakes, the IC engine and gearbox or the EV motor, inverter and accumulator.
- **Transient check** (`--solver transient`): drives the car forward in time (with a driver) along the QSS speed profile, with pitch, roll, heave, yaw and tire lag.
- **Car files** for the CT-17 IC and CT-17 EV: every value is checked for its key and units when a file loads, and carries its source.
- **A parameter register** that lists every parameter with its value, unit, source and confidence.
- **Notebooks with widgets** for every subsystem.
- **The website**: the same simulation in the browser, with editors, import / export and team publishing (`web/`).
- **A test suite** that protects the calibrated results and checks the physics against hand calculations.

## Repository layout

```
lapsim/                  Python package
  config/                schema, loader, parameter register
    schema.py              vehicle section (chassis, tires, suspension, aero, brakes, events)
    schema_ic.py           IC powertrain section       schema_ev.py   EV powertrain and pack sections
    loader.py              load_car(): car file -> Car (set values, build, run)
    register.py            every parameter; confirmed / placeholder status
    units.py               "350 lbf/in" -> SI
  core/                  vehicle, integrator, track, events, results
  solvers/               qss.py (method), qss_ic.py, qss_ev.py, transient.py
  subsystems/            one module per part of the car
    aero.py  tires.py  suspension.py  hardpoints.py  brakes.py  driveline.py  frame.py
    power/
      base.py            what every power module must provide
      ic.py              engine, cooling, electrics, the IC car
      ev.py              motor and inverter limits, the EV car
      battery.py         pack voltage, resistance, limits, state of charge
  studies/               sensitivity, setup optimisation, trackside adjustment
  validation/            tools that compare two runs; logged runs (logs.py)
  cli.py                 the `lapsim` command
cars/
  ct17_ic/vehicle.yaml   + hard points, engine dyno curve, frame tubes, aero outline (website)
  ct17_ev/vehicle.yaml   the CT-17 IC chassis (base) + the EV powertrain and pack
tracks/                  track centrelines (michigan_2026/, 2024/)
notebooks/               one notebook per subsystem, with widgets
tests/                   see "Testing" below
scripts/make_golden.py   regenerates the frozen reference results
docs/                    contracts, decision records, model cards
web/                     the website (web/README.md)
```

Telemetry and large result files are not stored in git. Keep them in the team's shared data folder under `raw/` (never edited) and `clean/`.

## Installing

Requires Python 3.11 or newer.

```bash
git clone https://github.com/uconnformulasae/PLS.git
cd PLS
python -m venv .venv
source .venv/bin/activate          # Windows: .venv\Scripts\activate
pip install -e ".[dev,notebooks]"
pytest -m "not slow"
```

If the tests do not pass on a fresh clone, **open an issue** before changing anything.

## Running the sim

```bash
# The cars in cars/
lapsim cars

# Check a car file for errors, and count confirmed / placeholder values
lapsim check-config ct17_ev

# List every parameter with its unit, source and note
lapsim register ct17_ev

# Run the events
lapsim run --car ct17_ev
lapsim run --car ct17_ic --event autocross
lapsim run --car ct17_ic --event skidpad --solver transient

# Change a parameter for one run without editing the file
lapsim run --car ct17_ic --event autocross --set pt:final_drive.rear_sprocket=40 --set "vehicle:mass.cg_height=11.5 in"

# Choose where results go, and compare two runs
lapsim run --car ct17_ic --out results/sprocket_40
lapsim compare results/baseline results/sprocket_40
```

(`python -m lapsim ...` works without installing.) A run written with `--out` is a folder holding the event table, every channel of every lap as `.csv`, and a `metadata.json` with the car file's hash, the changed values and the git commit. That is enough to reproduce any result later.

**Events:** `acceleration`, `skidpad`, `autocross`, `endurance`.

From Python:

```python
import lapsim
car = lapsim.load_car("ct17_ev")
car.set("ev:strategy.power_limit", "60 kW")
res = car.run()
res.table()                         # times and points
res.laps["autocross"].frame()       # every channel along the lap
```

## Car files

Each car is a single YAML file, `cars/<car>/vehicle.yaml`, in sections: `vehicle:` (chassis, tires, suspension, aero, brakes, events), `powertrain:` and, for the EV, `pack:`. The EV file starts from the IC car's chassis (`base: ../ct17_ic/vehicle.yaml`) and lists only what is different. The comments beside each value record where it came from.

```yaml
vehicle:
  mass:
    mass_no_driver: 211.4 kg          # B67
    cg_height: 11 in                  # N, B67 (279.4 mm, physical test)
powertrain:
  final_drive:
    front_sprocket: 11                # PT55  11T Yamaha R6 sprocket
    rear_sprocket: 32                 # PT55  MJ Corse 32T
```

Rules:

- Use **SI units**, or write the unit (`"350 lbf/in"`, `"11 in"`, `"80 kW"`).
- **NEVER** put a physical constant directly in the code. Add it to the car file and the schema.
- The loader stops at a key it does not know, so typos and leftover values are caught early.
- On the EV, a value becomes a real measurement when its key is added to `confirmed`; until then the register and the website report it as a PLACEHOLDER.
- The same `<section>:<path>` keys (`vehicle:`, `pt:`, `ev:`, `pack:`) are used by `--set`, the notebooks and the website's export: paste an exported value under that section.

## Notebooks

`notebooks/` has one notebook per subsystem. Each opens on the car file's values with sliders for the ones that matter; plots update as you move them, and a button runs the events.

| notebook | what |
|---|---|
| `00_full_car` | either car, the four events, what limits the car along the lap |
| `01_aero` | downforce, drag and balance vs speed and in corners; component table |
| `02_tires` | peak friction vs load and camber, lateral curve, friction ellipse, g-g diagram |
| `03_suspension` | springs, ARBs, CG: camber / roll centre / toe vs travel, design report |
| `04_brakes` | brake balance vs ideal, rotor temperature over repeated stops |
| `05_driveline_ic` | engine at WOT, wheel force by gear, sprockets, shift time, diff preload |
| `06_ev_motor` | motor / inverter envelope, regen, efficiency map, power limit |
| `07_battery` | pack topology and cell model: SOC, voltage, current, temperature over the endurance |
| `08_frame` | frame mass and torsional stiffness, and what they do to the lap |
| `09_transient` | the lap in the time domain against the QSS lap |

```bash
jupyter lab notebooks/
```

## The website

`web/` is the interactive lap-simulation website (GitHub Pages): every input of both cars, editors for the frame, hard points, wings and tracks, import / export of CAD and config files, team publishing, and the simulation itself running in the browser. It runs this package; see [web/README.md](web/README.md).

```bash
python web/build.py --serve        # rebuild after changing the model, a car file or a track; commit web/site/
```

## Testing

Tests live in `tests/` and run with `pytest`.

| Folder | What it checks | Run time |
| --- | --- | --- |
| `tests/unit/` | Each subsystem against hand calculations; car-file loading and validation; the comparison tool; results saving; the command line | Seconds |
| `tests/verification/` | Integrator accuracy, QSS properties, charge balance of the accumulator, transient vs QSS agreement, every notebook runs | About a minute |
| `tests/golden/` | Regression: today's results against frozen reference results | Seconds |

```bash
pytest -m "not slow"        # everyday run, also run by CI on every push
pytest -m slow              # transient check and the notebooks, run before merging solver changes
pytest tests/unit -v        # one folder, verbose
```

Hand-calculated reference numbers are kept in one place, `tests/reference_values.py`, each with its derivation, so anyone can check them.

## Checking results

A result counts as trustworthy only if it passes two kinds of check.

**Verification: does the code do what the equations say?**

- Every subsystem returns what a hand calculation gives (tire friction, wheel rate, brake torque, scoring, the battery's temperature factor).
- The integrator reaches its expected fourth-order accuracy on a problem with an exact solution.
- The charge drawn from the accumulator equals its state-of-charge drop.
- The skidpad gives identical results turning left and right; flying laps end at the speed they start.
- The transient solver agrees with the QSS within 5% on lap time on the skidpad.

**Regression: did a change move a calibrated result?**

The golden tests rerun both cars through all four events and compare them with the frozen reference along every lap. If a change is meant to move a result, regenerate the references and explain why (it is recorded in `CHANGELOG.md`):

```bash
python scripts/make_golden.py --force --reason "Fixed driveline drag sign"
```

Checking the model against real track data (validation) uses the logged-run tools (`lapsim/validation/logs.py`, the website's log import), which is the next major piece of work.

Contracts between the parts are in [docs/contracts.md](docs/contracts.md), the reasons behind the structure in [docs/decision_records.md](docs/decision_records.md), and what each model assumes in [docs/model_cards.md](docs/model_cards.md).

## Next steps

- 7-DOF and 14-DOF vehicle models
- AiM telemetry import and replay through the pipeline
- Measured inertias, CG height and suspension geometry
- Validation against held-out track sessions
- A CT-16 EV car file (`cars/ct16_ev/`) once its data is collected
