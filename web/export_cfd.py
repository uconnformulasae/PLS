#!/usr/bin/env python
"""Export saved fsae_cfd runs for the website's 3-D flow replay (Aero page).

    python web/export_cfd.py                      # every medium run in IC/results/cfd/runs/
    python web/export_cfd.py --resolution quick   # the quick runs instead
    python web/export_cfd.py --runs IC/results/cfd/runs/base_medium_d8a80d34a5 ...

Run it natively (it needs fsae_cfd, so numba, and the wing STL files), then `python web/build.py`.
Nothing is solved here: each run's saved mean flow field is sampled onto the car's surface and traced into
streamlines. For each run it writes web/site/data/cfd/<state>.json:

    vertices     car surface points [mm], car axes (x rearward from the front axle, y right, z up)
    groups       {group: [first face, number of faces]} into the shared face list (faces.json)
    cp           pressure coefficient at each vertex x 100 (sampled one cell off the surface)
    lines        streamlines of the mean flow: points [mm], speed / free-stream x 100, time along the line [ms]
    summary      loads with confidence intervals; group table; run set-up

The car's surface mesh is in these files: publishing the site publishes it.
"""
import argparse
import json
import sys
import warnings
from pathlib import Path

import numpy as np

WEB = Path(__file__).resolve().parent
ROOT = (WEB.parent if (WEB.parent / "IC").is_dir() else Path.cwd()).resolve()
IC = ROOT / "IC"
OUT = WEB / "site" / "data" / "cfd"
sys.path.insert(0, str(IC))
warnings.filterwarnings("ignore")

LABELS = {"base": "Straight line, static ride height", "heave": "Heave: car 15 mm lower", "pitch": "Pitch: nose 10 mm down, tail 10 mm up",
          "yaw": "Yaw: 5 degrees of sideslip"}


def _f(v, nd=3):
    """A float for JSON: rounded, None when it is not a number."""
    try:
        v = float(v)
    except (TypeError, ValueError):
        return v if isinstance(v, str) else None
    return round(v, nd) if np.isfinite(v) else None


def streamlines(res, n_y=15, n_z=9, ds=0.035, n_steps=190):
    """Streamlines of the mean flow from a rake ahead of the car, integrated with the midpoint rule."""
    from scipy.interpolate import RegularGridInterpolator
    g, f = res.grid, res.fields
    ax = (g.xc, g.yc, g.zc)
    U = [RegularGridInterpolator(ax, f[k], bounds_error=False, fill_value=np.nan) for k in ("u", "v", "w")]
    chi = RegularGridInterpolator(ax, f["chi"], bounds_error=False, fill_value=0.0)
    vinf = float(res.meta["airspeed"])
    sign = 1.0 if float(np.nanmean(f["u"][2])) > 0 else -1.0            # which way the far-field air moves along x
    yy, zz = np.meshgrid(np.linspace(-0.85, 0.85, n_y), np.linspace(0.04, 1.30, n_z))
    p = np.column_stack([np.full(yy.size, -1.55 if sign > 0 else 3.6), yy.ravel(), zz.ravel()])
    vel = lambda q: np.column_stack([u(q) for u in U])
    alive = np.ones(len(p), bool)
    pts, spd, tim = [p.copy()], [], [np.zeros(len(p))]
    v0 = vel(p)
    spd.append(np.linalg.norm(v0, axis=1))
    for _ in range(n_steps):
        v1 = vel(p)
        s1 = np.linalg.norm(v1, axis=1)
        ok = alive & np.isfinite(s1) & (s1 > 0.02 * vinf)
        mid = p + 0.5 * ds * v1 / np.maximum(s1, 1e-9)[:, None]
        v2 = vel(mid)
        s2 = np.linalg.norm(v2, axis=1)
        ok &= np.isfinite(s2) & (s2 > 0.02 * vinf)
        step = np.where(ok[:, None], ds * v2 / np.maximum(s2, 1e-9)[:, None], 0.0)
        q = p + step
        ok &= (chi(q) < 0.5) & (q[:, 0] > -1.7) & (q[:, 0] < 3.9) & (np.abs(q[:, 1]) < 1.6) & (q[:, 2] > 0.005) & (q[:, 2] < 2.2)
        alive = ok
        p = np.where(alive[:, None], q, p)
        pts.append(np.where(alive[:, None], p, np.nan))
        spd.append(np.where(alive, s2, np.nan))
        tim.append(tim[-1] + np.where(alive, ds / np.maximum(s2, 1e-9), 0.0))
    P, S, T = np.stack(pts, 1), np.stack(spd, 1), np.stack(tim, 1)        # [line, step, ...]
    lines = []
    for k in range(len(P)):
        n = int(np.isfinite(P[k, :, 0]).sum())
        if n < 12:
            continue
        lines.append({"p": np.round(P[k, :n] * 1e3).astype(int).ravel().tolist(), "s": np.round(100 * S[k, :n] / vinf).astype(int).tolist(),
                      "t": np.round(1e3 * T[k, :n]).astype(int).tolist()})
    return lines


def export(run: Path, faces_ref):
    from fsae_cfd import CarResult, VehicleMotion, build_car
    from fsae_cfd import post
    res = CarResult.load(run)
    m = res.meta
    motion = VehicleMotion(**{k: (tuple(v) if isinstance(v, list) else v) for k, v in m["motion"].items()})
    res.geom = build_car(str(IC / "configs" / "ct17_cfd.yaml"), motion.attitude())
    gone = sorted(set(m.get("labels", [])) - {p.name for p in res.geom.parts})
    if gone:                                              # the car config has changed since this run was solved
        print(f"  SKIPPED {run.name}: it was solved with parts that are no longer in configs/ct17_cfd.yaml ({', '.join(gone[:4])}"
              f"{' ...' if len(gone) > 4 else ''}). Re-solve it:  python tools/cfd_lapsim.py --stage cfd --resolution {m['resolution']}")
        return faces_ref, None
    verts, faces, cps, groups, n0, f0 = [], [], [], {}, 0, 0
    parts = sorted(post.surface_pressure(res), key=lambda pc: pc[0].group)          # one block of faces per group
    for p, cp in parts:
        v, f = p.mesh.vertices, p.mesh.faces
        verts.append(v)
        faces.append(f + n0)
        cps.append(np.nan_to_num(cp, nan=0.0))
        g = groups.setdefault(p.group, [f0, 0])
        g[1] += len(f)
        n0 += len(v)
        f0 += len(f)
    V, F, C = np.vstack(verts), np.vstack(faces), np.concatenate(cps)
    state = run.name.split("_")[0]
    summ = res.summary_table()
    out = {"state": state, "label": LABELS.get(state, state), "run": run.name, "resolution": m["resolution"], "cell_mm": round(1e3 * m["h"], 1),
           "cells": int(m["cells"]), "speed": float(m["airspeed"]), "rho": float(m["rho"]),
           "motion": {k: v for k, v in m["motion"].items() if k in ("speed", "sideslip_deg", "corner_radius", "dz_front", "dz_rear", "roll_deg")},
           "vertices": np.round(V * 1e3).astype(int).ravel().tolist(), "groups": groups,
           "cp": np.clip(np.round(100 * C), -600, 200).astype(int).tolist(),
           "summary": {str(r["quantity"]): [_f(r["mean"], 4), _f(r["95% CI +/-"], 4)] for _, r in summ.iterrows()},
           "group_table": {"columns": [str(c) for c in res.group_table().columns],
                           "rows": [[_f(v) for v in r] for r in res.group_table().itertuples(index=False)]},
           "lines": streamlines(res)}
    same_faces = faces_ref is not None and faces_ref.shape == F.shape and np.array_equal(faces_ref, F)
    if faces_ref is not None and not same_faces:
        out["faces"] = F.ravel().tolist()                 # this run's surface has its own triangles
    OUT.mkdir(parents=True, exist_ok=True)
    (OUT / f"{state}.json").write_text(json.dumps(out, separators=(",", ":"), allow_nan=False))
    print(f"  {state}: {len(V)} vertices, {len(F)} faces, {len(out['lines'])} streamlines, "
          f"drag {out['summary']['drag_N'][0]:.0f} N, downforce {out['summary']['downforce_N'][0]:.0f} N -> {(OUT / (state + '.json')).stat().st_size / 1e6:.2f} MB")
    return F, {"state": state, "label": out["label"], "file": f"{state}.json"}


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--resolution", default="medium")
    ap.add_argument("--runs", nargs="*", help="run folders (default: every <state>_<resolution>_* in IC/results/cfd/runs)")
    a = ap.parse_args()
    runs = [Path(r) for r in a.runs] if a.runs else sorted(p for p in (IC / "results" / "cfd" / "runs").glob(f"*_{a.resolution}_*") if p.is_dir())
    if not runs:
        sys.exit("no runs found: fill IC/results/cfd/runs with  python tools/cfd_lapsim.py --stage cfd --resolution medium")
    runs.sort(key=lambda p: (not p.name.startswith("base"), p.name))
    faces, index = None, []
    print(f"exporting {len(runs)} run(s) -> {OUT}")
    for r in runs:
        F, row = export(r, faces)
        if row is None:
            continue
        if faces is None:
            faces = F
            (OUT / "faces.json").write_text(json.dumps({"faces": F.ravel().tolist()}, separators=(",", ":")))
        index.append(row)
    if not index:
        sys.exit("nothing exported: the files already in web/site/data/cfd were left as they are")
    (OUT / "index.json").write_text(json.dumps({"cases": index}, indent=1))


if __name__ == "__main__":
    main()
