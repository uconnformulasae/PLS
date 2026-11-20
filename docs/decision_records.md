# Decision records

Short records of decisions that shape the code: what was decided, why, and what it costs. Newest last.

## 1. One package, `lapsim`, in the layout of the README (2026-11-19)

The simulation used to be four packages in the team's working folder (`fsae_vd` vehicle dynamics,
`fsae_pt` IC powertrain, `fsae_ev` EV powertrain, `fsae_ev_bms` accumulator) plus the frame and CFD tools,
about 40 000 lines. Only part of it is the lap simulation. It is now one package with one module per part
of the car, keeping only what the lap simulation, the notebooks and the website use.

How "what is used" was decided: every website action (run, preview, sweep, export, import, log import,
sensitivity, optimiser, trackside search, transient check, frame and hard-point edits) was run with a
profiler on both cars, and every function never reached was examined. The models were moved, not
rewritten. Check: every lap channel of all four events of both cars is identical to the old packages
(IC bit for bit; EV within 5e-15 relative, from averaging the cell parameters in a different order), and
every website action returns the same JSON as before.

## 2. One car file per car (2026-11-19)

Each car used three or four YAML files (vehicle, powertrain, accumulator, track set). Now `cars/<car>/vehicle.yaml`
holds every section, and the EV file starts from the IC car's chassis with `base:` and lists only what is
different. The track choice is the `events.*_track` path in the car file. The website's override keys
(`vehicle:`, `pt:`, `ev:`, `pack:`) did not change, so designs saved or published from the site still load.

## 3. What was left out, and where it is

Not needed by the lap simulation or the website, and left in the team's research folder (`fsae_lapsim/`):

| left out | why |
|---|---|
| `fsae_ev_bms` state estimation (Kalman filters, live CAN tools, data cleaning, reports) | a separate tool for logged pack data; the simulation needs only the pack's equivalent circuit (`power/battery.py`) |
| `fsae_cfd` (3-D CFD solver) and the CAD files | the website's 3-D flow replay and wing sections are pre-computed data on the website branch (`assets/data/cfd/`, `assets/py/wing_sections.json`) |
| multibody optimal control (`ocp`, `mb_study`, the CasADi symbolic models) | needs IPOPT, never ran in the browser; its results are pre-computed (`assets/data/mb/<car>.json` on the website branch) |
| `export_mb.py`, `export_cfd.py`, `export_wings.py` | they produce that pre-computed data and need the tools above |
| Magic Formula 6.1 tire, TTC fitting | not selected by either car (both use the simple peak-friction tire) |
| tire dynamics (carcass, wheel spin, temperature) | used only by an option of the transient check that was off |
| road elevation, banking, grip maps and track widths | no track file has them; every track is flat |
| `.xrk` reading | needs a native library; RaceStudio exports `.csv`, which the log import reads |
| plots, notebook helpers, report writers, calibration and identification scripts | replaced by the notebooks in `notebooks/` |
| suspension design study, frame plots, rod / rocker checks | not used by the simulation or the site |

The car-file values that only those parts read (the tire-dynamics section; the BMS sensor, filter, cleaning,
channel and CAN sections; the MF 6.1, TTC, damper-curve and aero-map file names) were removed from the car
files and the schema.

## 4. The studies take a runner (2026-11-19)

Sensitivity, the optimiser and the trackside search ran the car through a module-level `run_events`
that the website replaced at run time. They now take `runner(cfg, events)` explicitly
(`lapsim.studies.runner_for(car)`), so they always use the car's own power-module solver.

## 5. The EV transient check uses the motor limit (fixed 2026-11-19)

The transient (time-domain) check built its drive-force limit from the IC car's gear tables. For the EV
this failed and fell back to a tire-limited drive (the note "no powertrain limit (tire-limited drive)" in
its results). The restructure first kept that behaviour so the results could be compared; then
`solvers/transient.py` was given the EV's drive force from `EVTables` (`F * m / meq` at the start-of-event
accumulator state, the same limit the EV's QSS envelope uses). EV transient times moved: acceleration
3.756 -> 3.81 s, autocross 43.898 -> 43.934 s, skidpad unchanged (5.046 s). The IC car is unchanged.

## 6. Notebooks

One notebook per subsystem in `notebooks/`, each with widgets bound to car-file values by the override
keys. They are stored without outputs (they are interactive, and outputs make every change a large diff);
`tests/verification/test_notebooks.py` runs them all.

## 7. Two branches: the simulation on `main`, the website on `web/lapsim-site` (2026-11-20)

`main` holds what the simulation needs: the `lapsim` package, the car files, the tracks, the notebooks, the
tests, `scripts/make_golden.py` and these documents. `web/lapsim-site` holds only the website: `index.html`,
`_config.yml`, `docs/` (how the site works), `assets/` (style sheet, scripts, images, the bridge
`assets/py/lapsim_web.py`, the simulation bundle and the data the page reads) and `.github/` (the build
script and its workflow). GitHub Pages publishes that branch.

The website never copies the model by hand: `.github/build.py` on the website branch zips `lapsim/`, `cars/`
and `tracks/` from a checkout of `main` with the bridge into `assets/py/lapsim.zip` and writes the baseline
results from that zip. It runs after every push to `main` that passes the tests (`website` job in
`.github/workflows/tests.yml`) and after every change to the site itself (`.github/workflows/build.yml` on
the website branch). The override keys (`vehicle:`, `pt:`, `ev:`, `pack:`) stay the contract between them.

Dropped in the split: the page's file:// fallback (a script copy of every data file, which doubled the
site's size; the site is served by GitHub Pages, and locally by `build.py --serve`), the second Pages
workflow (Pages publishes the branch directly), and the empty blueprint files `cars/ct16_ev/vehicle.yaml`
and `lapsim/solvers/pointmass.py` (the CT-16 has no data yet; the time-domain solver is
`solvers/transient.py`).
