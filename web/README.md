# Lap-simulation website

A static website for the CT-17 IC and CT-17 EV predictive lap simulation, built for GitHub Pages.
It is a second way to use the simulation, next to the notebooks; it changes nothing in `IC/` or `ev/`.

```bash
LAPSIM_ROOT=/path/to/fsae_lapsim python web/build.py --serve      # build, then open http://localhost:8000
```

## How it works

| piece | what it does |
|---|---|
| `bridge/lapsim_web.py` | Calls `fsae_vd` / `fsae_pt` / `fsae_ev` and returns JSON: inputs, event results, every lap channel, subsystem data, sweeps. |
| `build.py` | Zips the simulation packages, configs and track files into `site/py/lapsim.zip`, then runs both cars **from that zip** to write the baseline `site/data/*.json`. |
| `site/` | The website: `index.html`, `assets/app.css`, `assets/app.js`, `assets/editors.js`, `assets/worker.js`. This folder is what gets published. |

`site/index.html` also works opened by double-click (no server): `build.py` writes a script copy of every data file
for that case. Running a simulation still needs the internet the first time, to download the Python runtime.

The page opens on the pre-computed baseline, so it is instant on a phone. Pressing **Run simulation**
starts Pyodide (Python in WebAssembly) in a web worker and runs the real model with the edited inputs.
The browser result for the baseline is identical to the native one.

Rebuild (`python web/build.py`) whenever the model, a config or a track changes, and commit `web/site/`.

## Publishing

This folder is published from this repository by `.github/workflows/pages.yml` whenever `web/site/` changes on `main`.
One-time setup: repository Settings -> Pages -> Source: GitHub Actions, then add the site link to the repository's
**About -> Website** field. The GitHub mobile app does not show Pages sites itself; it shows the README and opens
that link in its in-app browser.

The simulation source (`IC/`, `ev/`) lives in the team's `fsae_lapsim` folder, not in this repository. To rebuild:

```bash
LAPSIM_ROOT=/path/to/fsae_lapsim python web/build.py --serve
```

## What is published

Only the files listed in `BUNDLE` in `build.py`: the Python source of `fsae_vd`, `fsae_pt`, `fsae_frame`,
`fsae_ev`, `fsae_ev_bms`; the car and track-set YAML files; hardpoints, the dyno curve, the frame tube
list and the 2026 track centre lines. Raw TTC data, logs, CAD and results are never copied.

**Check before making the site public:** the car configs contain tire coefficients fitted from TTC data
(`tires.mu0`, `mu_fz`, ...). The TTC agreement restricts sharing tire data outside the team. A public
Pages site exposes everything in the bundle to anyone. A private repository with access-controlled Pages
needs GitHub Enterprise Cloud; Pages from a private repository on other paid plans is still a public site.

## Team styling

Colours are the variables at the top of `site/assets/app.css`. `--navy` (`#000E2F`) is sampled from the team logo.
`site/assets/logo.png` is the team mark cut out of the square logo image (white on transparent) and
`favicon.png` is the square logo. A vector (SVG) copy of the logo would be sharper on high-density screens.

## Interactive figures and live preview

`site/assets/editors.js` holds the figures you can drag and the sliders:

* **Frame editor** (Chassis & frame): drag nodes in side / top / front views, add or delete tubes, change each
  stock size's section, wall and material, or a tube's length. `fsae_frame`'s beam model recomputes mass, CG and
  torsional stiffness; the lap simulation gets the new mass and CG (and the frame's twist when that box is ticked).
* **Hard-point editor** (Suspension): drag any suspension point; the kinematics are re-solved from the edited
  OptimumK export and feed the lap simulation.
* **Car layout** (Chassis, Aero), **volumetric efficiency** and **gear ratios** (IC powertrain), **event tracks** (Overview).
* **Quick adjust** sliders on every subsystem page, for the main values in everyday units.

* **Joints**: "Add joint on tube" splits the selected tube, "Add joint" makes a free one, "Remove joint" takes a joint and
  its tubes away; "Frame joints by number", "Hard points by number" and "Link lengths" are the same geometry as tables.
* The hard-point editor draws the frame behind the suspension and keeps chassis-side points within a tab's length of a tube.
* **Driver** page: profiles from `web/drivers.json` (edit it, or edit on the site and Export), the seating / sight-line
  figure, and what the chosen driver changes in the car (mass, CG, grip scale, shift time, endurance power).
* **Brakes**: rotor temperature through the lap and through endurance, repeated stops, corner hardware.
* **Aero**: each wing element alone, from a panel method in the bridge (`airfoil()`); real section coordinates can be pasted in.

Each of these writes an ordinary input override (`geom:frame`, `geom:hardpoints`, `geom:tracks`, or a config key),
so Export / Import, sweeps and Run simulation all see it. With **Live preview** on, a change re-computes that page's
subsystem data in the browser (`preview()` in the bridge, no lap); lap times change only with Run simulation.

## Adding a visual

In `site/assets/app.js`, add one line to the list of visuals:

```js
V("aero", "my_id", "Title", "One-line description.", (body) => { /* draw into body */ }, { wide: true, off: true });
```

`off: true` means it starts switched off; the visitor turns it on with its chip. If it needs new data,
return it from `run()` in `bridge/lapsim_web.py`.

## Limits

* Multibody optimal control (CasADi / IPOPT) and the CFD solver (Numba) do not run in a browser.
* `.xrk` log reading and fitting from logs or TTC data are not on the site.
* First run downloads about 40 MB (cached after). Needs a browser with WebAssembly: current Safari, Chrome, Firefox.
