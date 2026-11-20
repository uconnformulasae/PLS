---
title: How the website works
---

# How the lap-simulation website works

A static website for the CT-17 IC and CT-17 EV predictive lap simulation, published by GitHub Pages from the
`web/lapsim-site` branch. It is a second way to use the simulation, next to the notebooks and the `lapsim`
command on `main`; it runs the same `lapsim` package and the same car files.

```bash
git clone -b main https://github.com/uconnformulasae/PLS.git PLS-main          # the simulation
git clone -b web/lapsim-site https://github.com/uconnformulasae/PLS.git PLS-site # the website
cd PLS-site
python .github/build.py --lapsim ../PLS-main --serve                            # build, then open http://localhost:8000
```

## How it works

| piece | what it does |
|---|---|
| `index.html`, `assets/css/`, `assets/js/`, `assets/img/` | The page: `app.js` (pages, visuals, running), `editors.js`, `tunnel.js`, `wings.js`, `workbench.js` (team publishing, import / export, optimisation), `multibody.js`, and `worker.js` (Pyodide). |
| `assets/py/lapsim_web.py` | The bridge: calls `lapsim` and returns JSON (inputs, event results, every lap channel, subsystem data, sweeps). |
| `.github/build.py` | Zips `lapsim/`, `cars/` and `tracks/` from a checkout of `main`, with the bridge, into `assets/py/lapsim.zip`, then runs both cars **from that zip** to write the baseline `assets/data/*.json`. |
| `assets/data/` | What the page reads: baseline results, drivers, pre-computed multibody and CFD runs, the team's published designs (`team/`). |

The page opens on the pre-computed baseline, so it is instant on a phone. Pressing **Run simulation**
starts Pyodide (Python in WebAssembly) in a web worker and runs the real model with the edited inputs.
The browser result for the baseline is identical to the native one. The page has to be served (GitHub Pages,
or `build.py --serve`); opened straight from disk the browser blocks its files.

## Publishing

GitHub Pages publishes this branch from its root on every commit (Settings -> Pages -> Deploy from a branch:
`web/lapsim-site`, `/ (root)`); `_config.yml` holds the site settings. The bundle and the baseline are rebuilt by
GitHub Actions, never by hand:

* after every push to `main` that passes the tests (`website` job in `main`'s `.github/workflows/tests.yml`);
* after every change to the page, its scripts or the bridge on this branch (`.github/workflows/build.yml`).

Each run commits the rebuilt `assets/` with main's commit in the message.

## What is published

Only what `.github/build.py` bundles: the `lapsim` package, the car files with their data (`cars/`: hard points,
the dyno curve, the frame tube list) and the track centre lines (`tracks/`). Raw TTC data, logs, CAD and results
are not in this repository.

**Check before making the site public:** the car files contain tire coefficients fitted from TTC data
(`tires.mu0`, `mu_fz`, ...). The TTC agreement restricts sharing tire data outside the team. A public
Pages site exposes everything in the bundle to anyone. A private repository with access-controlled Pages
needs GitHub Enterprise Cloud; Pages from a private repository on other paid plans is still a public site.

## Team styling

Colours are the variables at the top of `assets/css/app.css`. `--navy` (`#000E2F`) is sampled from the team logo.
`assets/img/logo.png` is the team mark cut out of the square logo image (white on transparent) and
`favicon.png` is the square logo. A vector (SVG) copy of the logo would be sharper on high-density screens.

## Interactive figures and live preview

`assets/js/editors.js` holds the figures you can drag and the sliders:

* **Frame editor** (Chassis & frame): drag nodes in side / top / front views, add or delete tubes, change each
  stock size's section, wall and material, or a tube's length. The frame's beam model (`lapsim/subsystems/frame.py` on `main`) recomputes mass, CG and
  torsional stiffness; the lap simulation gets the new mass and CG (and the frame's twist when that box is ticked).
* **Hard-point editor** (Suspension): drag any suspension point; the kinematics are re-solved from the edited
  OptimumK export and feed the lap simulation.
* **Car layout** (Chassis, Aero), **volumetric efficiency** and **gear ratios** (IC powertrain), **event tracks** (Overview).
* **Quick adjust** sliders on every subsystem page, for the main values in everyday units.

* **Joints**: "Add joint on tube" splits the selected tube, "Add joint" makes a free one, "Remove joint" takes a joint and
  its tubes away; "Frame joints by number", "Hard points by number" and "Link lengths" are the same geometry as tables.
* The hard-point editor draws the frame behind the suspension and keeps chassis-side points within a tab's length of a tube.
* **Driver** page: profiles from `assets/data/drivers.json` (edit it, or edit on the site and Export), the seating / sight-line
  figure, and what the chosen driver changes in the car (mass, CG, grip scale, shift time, endurance power).
* **Brakes**: rotor temperature through the lap and through endurance, repeated stops, corner hardware.
* **Aero**: each wing element alone, from a panel method in the bridge (`airfoil()`); real section coordinates can be pasted in.

* **Virtual wind tunnel (2-D)** on the Aero page (`assets/js/tunnel.js`): a lattice-Boltzmann flow solver running in
  the page on side-view slices built from `cars/ct17_ic/aero_geometry.yaml` (`tunnel_shapes()` in the bridge). Flow patterns and
  the direction of a change only: it is two-dimensional and far below the car's Reynolds number.

* **3-D flow replay** on the Aero page: the saved CFD runs (surface pressure, streamlines, smoke) rotated in the
  page, from `assets/data/cfd/*.json`. These files hold the car's surface mesh. They are pre-computed (see
  "Pre-computed data" below).

* **Real wing sections**: `assets/py/wing_sections.json` holds F1 to F3 and R1 to R4 cut out of the wing CAD (the CAD's
  fourth front flap, F4, is not on the car and is left out): shape, chord, angle, where along the span each one runs.
  The single-element card, the tunnel and the **wing stacks** card (elements solved together in ground effect,
  `wing_stack()` in the bridge) all use it. It is pre-computed (see "Pre-computed data" below).

Each of these writes an ordinary input override (`geom:frame`, `geom:hardpoints`, `geom:tracks`, or a config key),
so Export / Import, sweeps and Run simulation all see it. With **Live preview** on, a change re-computes that page's
subsystem data in the browser (`preview()` in the bridge, no lap); lap times change only with Run simulation.

## Working on the whole car, and sharing it

* **Full car simulation** lists every change made in any section ("Changes in this car") and sets the last run against
  the baseline, event by event. Run simulation only ever changes what that visitor sees.
* **Team** page: publishing is separate from running. A team member signs in with their own GitHub access token
  (fine-grained, this repository, Contents: read and write) and publishes a **car design** (inputs, frame, hard points,
  wings and its results), the **driver list**, or a **corner assembly**. Each publish is a commit made through the
  GitHub API to `assets/data/team/` on this branch (`TEAM` in `assets/js/workbench.js` names the repository and
  branch), so GitHub decides who may publish and the history shows who did. An adopted design is what
  everybody opens on. Do not edit `assets/data/team/` by hand: the site writes it.
* **Import & export**: `export_files()` in the bridge writes hard points (OptimumK format, an XYZ list, SolidWorks
  equations), the frame's tubes and joints, and the changed inputs as YAML; the page adds results and wing sections.
  Hard points and frame tubes come back in through `import_hardpoints()` / `import_frame()`.
* **Optimization**: a logged run exported from RaceStudio as .csv goes through the model's own cleaning pipeline
  (`lapsim/validation/logs.py`) and the car is simulated on the logged course (`log_import()`); grip and coast-down fits
  are offered as inputs. Sensitivity, the differential-evolution search with its robustness check, and the
  trackside-adjustment search are `lapsim.studies.optimize` and `lapsim.studies.adjust`, run with the full powertrain
  model (`sensitivity()`, `optimise()`, `trackside()`).
* **Wing designer** (`assets/js/wings.js`): every element to scale where it sits on the car; section, chord, angle,
  position, slot and skin can be changed, and the endplates' size and material. It feeds the 2-D tunnel directly and
  the lap simulation only through "Use this wing in the full car simulation".

## Multibody

The **Multibody** page (`assets/js/multibody.js`) shows the car as a multibody system, in two parts:

* **Minimum-lap-time solve** (lumped-axle chassis, optimal control, CasADi + IPOPT): the racing line, speed, steering,
  drive and brake from one optimisation, graded A to F with the checks behind the grade, next to the quasi-steady lap
  and the logged runs. IPOPT is not in the browser build of CasADi, so these are pre-computed for the baseline IC car
  (`assets/data/mb/ic.json`, see "Pre-computed data"). The EV car has no minimum-lap-time run yet.
* **Transient check** (`lapsim/solvers/transient.py` on `main`: pitch, roll, heave, yaw and tire lag, RK4 at 2 ms, plain NumPy):
  the car driven in the time domain along the quasi-steady speed profile. `.github/build.py` writes the baseline
  (`assets/data/mb/transient_<car>.json`); **Run transient check** runs it in the browser on the car with the inputs as
  they stand (`transient()` in the bridge), both cars, about 20 s.

## Pre-computed data

Three parts of the site come from tools that need software or files this repository does not hold (CasADi with
IPOPT, the CFD solver, the CAD). Their output is committed here and the build leaves it alone; the tools
(`export_mb.py`, `export_cfd.py`, `export_wings.py`) live in the team's research folder, `fsae_lapsim/web/`:

| file | what | tool |
|---|---|---|
| `assets/data/mb/ic.json` | minimum-lap-time runs of the IC car | `export_mb.py` (CasADi + IPOPT) |
| `assets/data/cfd/*.json` | 3-D CFD flow replay | `export_cfd.py` (the CFD solver, numba, wing STL files) |
| `assets/py/wing_sections.json` | wing sections cut from the CAD | `export_wings.py` (the CFD geometry tools, wing STL files) |

## Adding a visual

In `assets/js/app.js`, add one line to the list of visuals:

```js
V("aero", "my_id", "Title", "One-line description.", (body) => { /* draw into body */ }, { wide: true, off: true });
```

`off: true` means it starts switched off; the visitor turns it on with its chip. If it needs new data,
return it from `run()` in `assets/py/lapsim_web.py`.

## Limits

* Multibody optimal control (CasADi / IPOPT) and the CFD solver (Numba) do not run in a browser: both are pre-computed
  (see "Pre-computed data").
* `.xrk` logs: export the session to .csv from RaceStudio. Fitting tires from TTC data is not on the site.
* First run downloads about 40 MB (cached after). Needs a browser with WebAssembly: current Safari, Chrome, Firefox.
