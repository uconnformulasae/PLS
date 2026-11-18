#!/usr/bin/env python
"""Export the multibody minimum-lap-time runs for the website's Multibody page.

    python web/export_mb.py                                   # CT-17 IC: acceleration, skidpad, autocross
    python web/export_mb.py --car ic --events skidpad         # one car, one event
    python web/export_mb.py --events autocross endurance      # endurance = one flying lap of the endurance course
    python web/export_mb.py --mesh-check                      # also re-solve on a mesh twice as fine (slower)

    LAPSIM_ROOT=/path/to/fsae_lapsim python web/export_mb.py  # when web/ lives in another repository

Run it natively: it needs CasADi with IPOPT, which the browser build of CasADi does not have (Pyodide's CasADi has
only its SQP solvers). Then run `python web/build.py`. It solves ``fsae_vd.mb_study`` (the layer under the notebook's
multibody panel) for the same car the website opens on (bridge ``_load``: ct17 configs + the 2026 track set), with
the lumped-axle model (15 states), and writes web/site/data/mb/<car>.json:

    settings     model, mesh, track half width per event
    events       per event: grade (A-F) and the checks behind it, optimal / quasi-steady / logged / replay time,
                 the optimal lap's channels along the distance, the racing line and track edges (x, y), the
                 quasi-steady speed trace

An event already in the file is kept unless it is re-run, so events can be exported one at a time.
Wall time on a laptop: acceleration and skidpad about 10 s each, autocross about 3 min, an endurance lap about 45 min.
"""
import argparse
import datetime as dt
import json
import math
import os
import subprocess
import sys
import time
import warnings
from pathlib import Path

import numpy as np

WEB = Path(__file__).resolve().parent
ROOT = Path(os.environ.get("LAPSIM_ROOT") or WEB.parent).resolve()
OUT = WEB / "site" / "data" / "mb"
os.environ.setdefault("LAPSIM_HOME", str(ROOT))
sys.path.insert(0, str(WEB / "bridge"))
sys.path.insert(0, str(ROOT / "IC"))
warnings.filterwarnings("ignore")

EVENTS = ("acceleration", "skidpad", "autocross")
DS = {"acceleration": 1.0, "skidpad": 1.0}          # m per interval; others 2 m (as the multibody notebook)
G0 = 9.80665


def _f(v, sig=5):
    try:
        v = float(v)
    except (TypeError, ValueError):
        return None
    if not math.isfinite(v):
        return None
    if v == 0:
        return 0.0
    return round(v, sig - 1 - int(math.floor(math.log10(abs(v)))))


def _a(x, sig=5):
    return [_f(v, sig) for v in np.asarray(x, float).ravel()]


def _git():
    try:
        return subprocess.run(["git", "rev-parse", "--short", "HEAD"], cwd=ROOT, capture_output=True, text=True).stdout.strip()
    except Exception:  # noqa: BLE001
        return ""


def event_json(r, settings):
    """One MBResult -> the page's JSON."""
    from fsae_vd.mb_study import racing_line
    df, tr, o = r.df, r.track, r.ocp
    s, n = df["s"].to_numpy(float), df["n"].to_numpy(float)
    m = r.mass if np.isfinite(r.mass) else 280.0
    delta = df["delta"].to_numpy(float)
    cd, sd = np.cos(delta), np.sin(delta)
    fy = (df["Fx_fl"] + df["Fx_fr"]).to_numpy() * sd + (df["Fy_fl"] + df["Fy_fr"]).to_numpy() * cd + (df["Fy_rl"] + df["Fy_rr"]).to_numpy()
    ay = fy / m
    v = df["speed"].to_numpy(float)
    ax = np.gradient(v, df["t"].to_numpy(float)) if len(v) > 2 else np.zeros_like(v)
    deg = 180 / math.pi
    ch = {"s": s, "t": df["t"], "v": v, "n": n, "ax": ax, "ay": ay,
          "delta_deg": delta * deg, "beta_deg": np.arctan2(df["v"], df["u"]) * deg, "yaw_rate": df["r"] * deg,
          "roll_deg": df["phi"] * deg, "pitch_deg": df["theta"] * deg, "heave_mm": df["z"] * 1e3,
          "F_drive": df["F_drive"], "F_brake": df["F_brake"], "downforce": df["downforce"], "drag": df["drag"],
          "kappa": df["kappa_track"]}
    for c in ("fl", "fr", "rl", "rr"):
        for k in ("Fz", "util"):
            if f"{k}_{c}" in df:
                ch[f"{k}_{c}"] = df[f"{k}_{c}"]
        if f"alpha_{c}" in df:
            ch[f"alpha_{c}_deg"] = df[f"alpha_{c}"] * deg
    for k in ("T_drive", "T_drive_max", "T_brake", "P_source"):
        if k in df:
            ch[k] = df[k]
    x, y = racing_line(tr, s, n)
    tx, ty = np.asarray(tr.x, float), np.asarray(tr.y, float)
    hd = np.unwrap(np.arctan2(np.gradient(ty), np.gradient(tx)))
    hw = 0.05 if settings.centre_line else (min(settings.half_width, 1.5) if r.event == "skidpad" else settings.half_width)
    own = getattr(tr, "has_boundaries", False) and not settings.centre_line
    wl = np.asarray(tr.w_left, float) if own else hw
    wr = np.asarray(tr.w_right, float) if own else hw
    j = np.unique(np.round(np.linspace(0, len(tx) - 1, min(len(tx), 600))).astype(int))
    wl_j = wl[j] if np.ndim(wl) else wl
    wr_j = wr[j] if np.ndim(wr) else wr
    edges = {"left": {"x": _a(tx[j] - wl_j * np.sin(hd[j])), "y": _a(ty[j] + wl_j * np.cos(hd[j]))},
             "right": {"x": _a(tx[j] + wr_j * np.sin(hd[j])), "y": _a(ty[j] - wr_j * np.cos(hd[j]))},
             "centre": {"x": _a(tx[j]), "y": _a(ty[j])}}
    q = r.qss
    qj = np.unique(np.round(np.linspace(0, len(q.s) - 1, min(len(q.s), 700))).astype(int)) if q is not None else None
    log = r.log or {}
    trace = log.get("trace")
    rp = r.replay
    out = {
        "grade": r.grade, "checks": [[lv, str(t)] for lv, t in r.checks],
        "time": _f(o.time, 6), "qss_time": _f(q.time, 6) if q is not None else None,
        "log_time": _f(log.get("time"), 5), "log_what": log.get("what"),
        "replay_time": _f(rp.time, 6) if rp is not None and rp.completed else None,
        "mesh_time": _f(r.mesh_time, 6) if r.mesh_time is not None else None,
        "optimal": bool(getattr(o, "optimal", False)), "consistent": bool(o.success),
        "constraint_violation": _f(getattr(o, "constr_viol", float("nan")), 3),
        "iterations": int(o.iterations), "solve_s": _f(o.solve_seconds, 3), "wall_s": _f(r.elapsed, 3),
        "length_m": _f(tr.length, 5), "closed": bool(tr.closed), "ds": settings.ds, "half_width": hw,
        "mass_kg": _f(m, 5), "intervals": int(len(df) - 1),
        "channels": {k: _a(np.asarray(v, float), 5) for k, v in ch.items()},
        "line": {"x": _a(x), "y": _a(y)}, "edges": edges,
        "qss": {"s": _a(np.asarray(q.s)[qj]), "v": _a(np.asarray(q.v)[qj]), "ax": _a(np.asarray(q.ax)[qj]),
                "ay": _a(np.asarray(q.ay)[qj])} if q is not None else None,
        "log_trace": {"s": _a(trace["s"].to_numpy()[::max(1, len(trace) // 700)]),
                      "v": _a(trace["v"].to_numpy()[::max(1, len(trace) // 700)])} if trace is not None else None,
    }
    return out


def export(car, events, model="axle", mesh_check=False, logs=True):
    import lapsim_web as w
    from fsae_vd import mb_study as MB
    objs = w._load(car)
    ptc = objs.get("pt")
    S = MB.Study(objs["vehicle"], ptc, None, root=ROOT / "IC")
    path = OUT / f"{car}.json"
    data = json.loads(path.read_text()) if path.exists() else {}
    data.setdefault("events", {})
    for e in events:
        st = MB.MBSettings(model=model, ds=DS.get(e, 2.0), replay=True, mesh_check=mesh_check,
                           independent_check=(e == "skidpad"), compare_logs=logs and car == "ic")
        t0 = time.time()
        print(f"{car} {e}: {model} model, {st.ds:g} m mesh, about {S.estimate_seconds(e, st):.0f} s", flush=True)
        try:
            r = S.run(e, st, stage=lambda t: print("   ", t, flush=True))
        except Exception as err:  # noqa: BLE001
            print(f"    failed: {type(err).__name__}: {err}", flush=True)
            data["events"][e] = {"error": f"{type(err).__name__}: {err}"}
            continue
        data["events"][e] = event_json(r, st)
        print(f"    {r.time:.3f} s (quasi-steady {float(r.qss.time):.3f} s), grade {r.grade}, {time.time() - t0:.0f} s", flush=True)
    data.update({"car": car, "name": str(getattr(objs["vehicle"], "name", car)), "model": model,
                 "built": dt.datetime.now().strftime("%Y-%m-%d %H:%M"), "commit": _git(),
                 "powertrain": "fsae_pt engine and gearbox (drive-force limit)" if ptc is not None else
                 "no powertrain limit in the multibody model (grip-limited drive)"})
    order = [e for e in ("acceleration", "skidpad", "autocross", "endurance", "brake_test") if e in data["events"]]
    data["events"] = {e: data["events"][e] for e in order + [e for e in data["events"] if e not in order]}
    OUT.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(data, separators=(",", ":"), allow_nan=False))
    print(f"wrote {path.relative_to(WEB)} ({path.stat().st_size // 1024} kB)")


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--car", nargs="+", default=["ic"], choices=["ic", "ev"], help="ev: the multibody study has no EV powertrain yet")
    ap.add_argument("--events", nargs="+", default=list(EVENTS))
    ap.add_argument("--model", default="axle", choices=["axle", "corner"])
    ap.add_argument("--mesh-check", action="store_true")
    ap.add_argument("--no-logs", action="store_true", help="skip the logged-run comparison (reads data/logs/*.xrk)")
    a = ap.parse_args()
    try:
        import casadi as ca
        x = ca.SX.sym("x")
        ca.nlpsol("t", "ipopt", {"x": x, "f": x ** 2}, {"ipopt.print_level": 0, "print_time": 0})
    except Exception as e:  # noqa: BLE001
        sys.exit(f"CasADi with IPOPT is needed natively ({e}). pip install casadi")
    os.chdir(ROOT / "IC")
    for car in a.car:
        export(car, a.events, a.model, a.mesh_check, not a.no_logs)


if __name__ == "__main__":
    main()
