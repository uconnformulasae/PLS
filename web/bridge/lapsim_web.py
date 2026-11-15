"""lapsim_web - JSON bridge between the lap-simulation packages and the web UI.

The website (web/site) runs this file two ways:

* ``python web/build.py`` runs it natively to write the baseline results the page shows on load;
* in the browser it runs inside Pyodide (Python compiled to WebAssembly) so a visitor can change
  any input and re-run the real simulation - the same ``fsae_vd`` / ``fsae_pt`` / ``fsae_ev`` code
  as the notebooks, not a copy of it.

Everything returned is plain JSON: numbers, strings, lists, dicts (NaN -> null).

    import lapsim_web as w
    w.schema("ic")                                   # every input, grouped by subsystem
    w.run("ev", {"ev:drivetrain.rear_sprocket": 40}) # events, laps, subsystem data
    w.sweep("ic", "vehicle:mass.cg_height", [0.25, 0.28, 0.31])
"""
from __future__ import annotations

import dataclasses as dc
import importlib.abc
import importlib.machinery
import math
import os
import sys
import time
import types
import warnings
from pathlib import Path

HOME = Path(os.environ.get("LAPSIM_HOME") or Path(__file__).resolve().parents[2])
IC, EV = HOME / "IC", HOME / "ev"
for _p in (IC, EV, EV / "fsae_ev_bms"):
    if str(_p) not in sys.path:
        sys.path.insert(0, str(_p))

MAX_POINTS = 900          # points per lap trace sent to the page
EVENTS = ("acceleration", "skidpad", "autocross", "endurance")


# --------------------------------------------------------------------------- matplotlib stand-in
class _Anything:
    """Accepts any attribute, call, index or iteration: stands in for matplotlib objects."""

    def __getattr__(self, name):
        if name.startswith("__") and name.endswith("__"):
            raise AttributeError(name)
        return _Anything()

    def __call__(self, *a, **k):
        return _Anything()

    def __getitem__(self, k):
        return _Anything()

    def __setitem__(self, k, v):
        pass

    def __iter__(self):
        return iter(())

    def __mro_entries__(self, bases):
        return (object,)


class _StubModule(types.ModuleType):
    __path__: list = []

    def __getattr__(self, name):
        if name.startswith("__") and name.endswith("__"):
            raise AttributeError(name)
        return _Anything()


class _StubFinder(importlib.abc.MetaPathFinder, importlib.abc.Loader):
    ROOTS = ("matplotlib", "mpl_toolkits")

    def find_spec(self, name, path=None, target=None):
        if name.split(".")[0] in self.ROOTS:
            return importlib.machinery.ModuleSpec(name, self, is_package=True)
        return None

    def create_module(self, spec):
        return _StubModule(spec.name)

    def exec_module(self, module):
        pass


def stub_matplotlib(force=False):
    """The plot modules import matplotlib at the top; the page draws its own charts, so in the
    browser matplotlib is replaced by an empty stand-in instead of being downloaded (about 8 MB)."""
    if not force:
        try:
            import matplotlib  # noqa: F401
            return False
        except ImportError:
            pass
    sys.meta_path.insert(0, _StubFinder())
    return True


if os.environ.get("LAPSIM_STUB_MPL") or sys.platform == "emscripten":
    stub_matplotlib(force=True)
warnings.filterwarnings("ignore")

import numpy as np  # noqa: E402
import pandas as pd  # noqa: E402


# --------------------------------------------------------------------------- JSON helpers
def _num(x, sig=5):
    try:
        x = float(x)
    except (TypeError, ValueError):
        return None
    if not math.isfinite(x):
        return None
    if x == 0:
        return 0.0
    return round(x, sig - 1 - int(math.floor(math.log10(abs(x)))))


def _j(o, sig=5):
    """Anything -> JSON-able."""
    if o is None or isinstance(o, (str, bool)):
        return o
    if isinstance(o, (np.bool_,)):
        return bool(o)
    if isinstance(o, (int, np.integer)):
        return int(o)
    if isinstance(o, (float, np.floating)):
        return _num(o, sig)
    if isinstance(o, np.ndarray):
        if o.dtype.kind in "fc":
            return [_num(v, sig) for v in o.ravel()] if o.ndim <= 1 else [_j(r, sig) for r in o]
        return [_j(v, sig) for v in o.tolist()]
    if isinstance(o, pd.DataFrame):
        return _table(o)
    if isinstance(o, pd.Series):
        return {str(k): _j(v, sig) for k, v in o.items()}
    if isinstance(o, dict):
        return {str(k): _j(v, sig) for k, v in o.items()}
    if isinstance(o, (list, tuple, set)):
        return [_j(v, sig) for v in o]
    if isinstance(o, Path):
        return o.name
    return str(o)


def _table(df, index=True):
    """DataFrame -> {columns: [...], rows: [[...], ...]} (the index becomes the first column)."""
    if df is None:
        return None
    df = df.reset_index() if index else df
    return {"columns": [str(c) for c in df.columns], "rows": [[_j(v) for v in r] for r in df.itertuples(index=False)]}


def _val(x):
    return x() if callable(x) else x


def _pick(n, n_max=MAX_POINTS):
    if n <= n_max:
        return np.arange(n)
    return np.unique(np.round(np.linspace(0, n - 1, n_max)).astype(int))


def _safe(out, key, fn):
    """Each block of subsystem data is independent: one that fails reports its error and the rest still show."""
    try:
        out[key] = fn()
    except Exception as e:  # noqa: BLE001
        out[key] = {"error": f"{type(e).__name__}: {e}"}


# --------------------------------------------------------------------------- inputs
# top-level config section -> subsystem page
GROUPS = {
    "vehicle": {"mass": "chassis", "geometry": "chassis", "tires": "tires", "tire_dynamics": "tires",
                "front": "suspension", "rear": "suspension", "anti": "suspension", "steering": "suspension",
                "adjust": "suspension", "adjustability": "suspension", "aero": "aero", "brakes": "brakes",
                "powertrain": "powertrain", "events": "events", "sim": "events"},
    "pt": {"scoring": "events", "cooling": "cooling", "ambient": "cooling", "fuel": "powertrain"},
    "ev": {"accumulator": "accumulator", "strategy": "accumulator", "endurance": "accumulator", "scoring": "events"},
    "pack": {},
}
GROUP_DEFAULT = {"vehicle": "chassis", "pt": "powertrain", "ev": "powertrain", "pack": "accumulator"}
SKIP = {"source", "listed", "confirmed", "name", "hardpoints"}
SKIP_SUFFIX = ("_file", "_track", "_dir", "_folder")
# the EV page does not edit the IC car's simple powertrain block (the EV model replaces it)
HIDE = {"ev": {"vehicle": {"powertrain"}}, "ic": {"vehicle": {"powertrain"}}}


def _scalar(v):
    return v is None or isinstance(v, (bool, int, float, str, np.integer, np.floating))


def _walk(obj, prefix, rows, meta=None):
    if dc.is_dataclass(obj):
        items = [(f.name, getattr(obj, f.name), dict(f.metadata)) for f in dc.fields(obj)]
    else:
        items = [(k, v, {}) for k, v in obj.items()]
    for name, v, md in items:
        if name in SKIP or name.startswith("_") or name.endswith(SKIP_SUFFIX):
            continue
        key = f"{prefix}{name}"
        if dc.is_dataclass(v) or isinstance(v, dict):
            _walk(v, key + ".", rows)
        elif _scalar(v):
            kind = "bool" if isinstance(v, (bool, np.bool_)) else "text" if isinstance(v, str) else "number"
            rows.append({"key": key, "value": _j(v, 7), "kind": kind, "unit": md.get("unit", ""),
                         "note": md.get("note", ""), "source": md.get("source", "")})
        elif isinstance(v, (list, tuple, np.ndarray)) and len(v):
            rows.append({"key": key, "value": _j(np.asarray(v, float) if not isinstance(v[0], str) else list(v), 7), "kind": "list",
                         "unit": md.get("unit", ""), "note": md.get("note", ""), "source": md.get("source", "")})


def _load(car, overrides=None):
    """The car's config objects, with the overrides applied: {'vehicle': cfg, 'pt' | 'ev': ..., 'pack': ...}."""
    car = car.lower()
    if car == "ic":
        import fsae_pt as pt
        import fsae_vd as vd
        from fsae_vd.track_sets import load_track_set
        cfg = vd.load_config(str(IC / "configs" / "ct17_baseline.yaml"))
        ptc = pt.load_pt_config(str(IC / "configs" / "ct17_powertrain.yaml"))
        load_track_set("ic_2026", root=IC).apply(cfg)
        objs = {"vehicle": cfg, "pt": ptc}
    elif car == "ev":
        import fsae_ev as ev
        cfg, evc, bcfg, status = ev.load_car()
        objs = {"vehicle": cfg, "ev": evc, "pack": bcfg, "_status": status}
    else:
        raise ValueError(f"car must be 'ic' or 'ev', not {car!r}")
    ov = dict(overrides or {})
    for k, v in ov.items():
        if not k.startswith("geom:"):
            _set(objs, k, v)
    if ov.get("geom:tracks"):
        _apply_tracks(cfg, ov["geom:tracks"])
    if ov.get("geom:hardpoints"):
        _apply_hardpoints(cfg, ov["geom:hardpoints"])
    if ov.get("geom:frame"):
        objs["vehicle"] = _apply_frame(cfg, ov["geom:frame"])
    return objs


def _set(objs, key, value):
    """key = '<object>:<dotted.path>', value = a number in SI or a string with units ('350 lbf/in')."""
    from fsae_vd.units import parse_quantity
    which, _, path = key.partition(":")
    obj = objs[which]
    *parents, leaf = path.split(".")
    in_dict = False
    for p in parents:
        if isinstance(obj, dict):
            obj, in_dict = obj[p], True
        else:
            obj = getattr(obj, p)
            in_dict = isinstance(obj, dict)
    if isinstance(value, str):
        s = value.strip()
        if s.lower() in ("", "null", "none"):
            value = None
        elif s.lower() in ("true", "false"):
            value = s.lower() == "true"
        else:
            try:
                value = float(s)
            except ValueError:
                value = s
    if isinstance(value, list) and not isinstance(obj, dict):
        setattr(obj, leaf, value)
        return
    if isinstance(obj, dict):
        obj[leaf] = parse_quantity(value, angle_field=leaf.endswith("_deg")) if isinstance(value, str) else value
    elif not parents or in_dict or not hasattr(objs[which], "set"):
        setattr(obj, leaf, value)
    else:
        objs[which].set(path, value)


def schema(car):
    """Every editable input of the car: [{key, value, kind, unit, note, source, group, section}]."""
    objs = _load(car)
    rows = []
    for which, obj in objs.items():
        if which.startswith("_"):
            continue
        part = []
        _walk(obj, "", part)
        hide = HIDE.get(car, {}).get(which, set())
        for r in part:
            sec = r["key"].split(".")[0]
            if sec in hide:
                continue
            r["section"] = sec if "." in r["key"] else "general"
            r["group"] = GROUPS[which].get(sec, GROUP_DEFAULT[which])
            r["key"] = f"{which}:{r['key']}"
            rows.append(r)
    out = {"car": car, "inputs": rows}
    if "_status" in objs:
        st = objs["_status"]
        out["status"] = _table(st, index=False)
    return out


# --------------------------------------------------------------------------- laps
def _lap(lr, track=None):
    df = lr.frame()
    idx = _pick(len(df))
    ch = {}
    for c in df.columns:
        col = df[c].to_numpy()[idx]
        ch[str(c)] = [str(v) for v in col] if col.dtype.kind in "OUS" else _j(np.asarray(col, float), 4)
    tr = track or getattr(lr, "track", None)
    out = {"n": int(len(idx)), "time": _num(lr.time), "channels": ch, "summary": _j(getattr(lr, "summary", None) or {})}
    if tr is not None and tr.x is not None and len(tr.x) == len(df):
        out["x"], out["y"] = _j(np.asarray(tr.x)[idx], 5), _j(np.asarray(tr.y)[idx], 5)
    return out


def _endurance_trace(laps, cols):
    """A few channels over the whole endurance (all laps joined), against elapsed time."""
    parts, t0 = [], 0.0
    for i, lr in enumerate(laps):
        df = lr.frame()
        keep = [c for c in cols if c in df.columns]
        d = df[keep].copy()
        d["t_elapsed"] = t0 + df["t"].to_numpy()
        d["lap"] = i + 1
        t0 += float(lr.time)
        parts.append(d)
    df = pd.concat(parts, ignore_index=True)
    idx = _pick(len(df), 1500)
    return {str(c): _j(df[c].to_numpy(float)[idx], 4) for c in df.columns}


def _ggv(g, n=6):
    """g-g diagrams at a few speeds from the tire / traction envelope."""
    out = []
    for i in np.unique(np.round(np.linspace(0, len(g.v) - 1, n)).astype(int)):
        ay = g.frac * g.ay_max[i] / 9.81
        out.append({"v": _num(g.v[i]), "ay": _j(ay), "ax_acc": _j(g.ax_acc[i] / 9.81), "ax_brk": _j(g.ax_brk[i] / 9.81)})
    return {"speeds": out, "v": _j(g.v), "ay_max_g": _j(g.ay_max / 9.81)}


# --------------------------------------------------------------------------- subsystems
def _aero(veh):
    v = np.linspace(0, 35, 36)
    lf, lr, d = veh.aero(v)
    tot = lf + lr
    return {"v": _j(v), "downforce_front_N": _j(lf), "downforce_rear_N": _j(lr), "drag_N": _j(d),
            "front_share_%": _j(100 * lf / np.where(tot == 0, np.nan, tot)),
            "drag_power_kW": _j(d * v / 1e3),
            "components": _table(veh.aero_model.table().round(4)) if hasattr(veh, "aero_model") else None}


def _suspension(veh):
    from fsae_vd import hardpoints, reports
    veh.pt.unlimited = True          # tire-limited, as traction_ggv does: same numbers, without the engine look-ups
    try:
        axle, car = reports.design_report(veh)
    finally:
        veh.pt.unlimited = False
    out = {"axles": _table(axle.rename_axis("quantity")), "car": _j(car)}
    kin = {}
    for which in ("front", "rear"):
        f = getattr(veh.cfg, which).hardpoint_file
        if not f:
            continue
        try:
            c = hardpoints.Corner(hardpoints.load_optimumk_csv(f), front=which == "front")
            k = hardpoints.analyse_corner(c)
            kin[which] = {"travel_mm": _j(np.asarray(k.travel) * 1e3), "camber_deg": _j(k.camber), "toe_deg": _j(k.toe),
                          "rc_height_mm": _j(np.asarray(k.rc_height) * 1e3),
                          "damper_len_mm": _j(np.asarray(k.damper_len) * 1e3), "static": _j(k.static)}
        except Exception as e:  # noqa: BLE001
            kin[which] = {"error": str(e)}
    out["kinematics"] = kin
    return out


def _brakes(veh):
    from fsae_vd import brakes
    g = np.round(np.arange(0.2, 2.01, 0.2), 2)
    return {"balance": _table(pd.DataFrame(brakes.balance_table(veh, decel_g=tuple(g))), index=False),
            "hardware": _j(brakes.hardware_summary(veh.cfg))}


def _ic_powertrain(veh, T):
    rpm = np.arange(3000, veh.ptc.engine.rev_limit + 1, 250.0) if hasattr(veh.ptc.engine, "rev_limit") else np.arange(3000, 13001, 250.0)
    w = veh.engine.wot(rpm)
    idx = _pick(len(T.v), 300)
    return {"engine": {k: _j(np.asarray(w[k], float)) for k in ("rpm", "torque", "power", "map", "bsfc_g_kwh", "ve") if k in w},
            "engine_summary": _j(veh.engine.summary()),
            "gears": _j(veh.dt.summary(veh.m)),
            "traction": {"v": _j(T.v[idx]), "force_by_gear": [_j(f[idx]) for f in T.F], "resistance": _j(T.R[idx]),
                         "best_gear": _j(T.best[idx]), "v_shift": _j(T.v_shift), "shift_rpm": _j(T.shift_rpm),
                         "v_top": _num(T.v_top)}}


def _ev_powertrain(veh, T):
    import fsae_ev as ev
    ep = T.ep
    idx = _pick(len(T.v), 300)
    code = np.asarray(T.code)[idx]
    names = [ev.CODES[int(c)] if np.issubdtype(code.dtype, np.integer) and 0 <= int(c) < len(ev.CODES) else str(c) for c in code]
    m = ep.motor
    rg = np.linspace(100, ep.evc.motor.max_rpm, 40)
    tg = np.linspace(1, float(np.nanmax(T.T)), 30)
    return {"envelope": {"v": _j(T.v[idx]), "rpm": _j(T.rpm[idx]), "torque": _j(T.T[idx]), "force": _j(T.F[idx]),
                         "P_acc_kW": _j(np.asarray(T.P_acc)[idx] / 1e3), "regen_torque": _j(T.T_regen[idx]),
                         "regen_force": _j(T.F_regen[idx]), "P_regen_kW": _j(np.asarray(T.P_regen)[idx] / 1e3),
                         "resistance": _j(T.R[idx]), "limit": names, "v_top": _num(T.v_top)},
            "efficiency_map": {"rpm": _j(rg), "torque": _j(tg), "eta": _j(100 * m.efficiency_map(rg, tg), 4)},
            "overall_ratio": _num(ep.G), "power_limit_kW": _num(_val(ep.power_limit) / 1e3),
            "pack_energy_kWh": _num(_val(ep.pack.nominal_energy) / 3.6e6)}


# --------------------------------------------------------------------------- run
def run(car, overrides=None, events=None, detail=True):
    """Run the events of one car. ``overrides`` = {'<object>:<path>': value}."""
    t_start = time.time()
    car = car.lower()
    events = tuple(events or EVENTS)
    objs = _load(car, overrides)
    cfg = objs["vehicle"]
    if car == "ic":
        import fsae_pt as pt
        res = pt.run_events_pt(cfg, objs["pt"], events=events)
        limits, colors = list(pt.LIMITS), dict(pt.LIMIT_COLORS)
    else:
        import fsae_ev as ev
        res = ev.run_events_ev(cfg, objs["ev"], objs["pack"], events=events)
        limits, colors = list(ev.LIMITS), dict(ev.LIMIT_COLORS)
    veh = res.vehicle
    out = {"car": car, "name": str(getattr(cfg, "name", car)),
           "events": {e: {"time_s": _num(res.times.get(e), 6), "points": _num(res.points.get(e), 5)}
                      for e in (*EVENTS, "efficiency") if e in res.times or e in res.points},
           "notes": [str(n) for n in res.notes], "limits": limits, "limit_colors": colors,
           "mass_kg": _num(veh.m), "overrides": _j(overrides or {})}
    laps = {}
    for e in EVENTS:
        if e in res.laps:
            _safe(laps, e, lambda e=e: _lap(res.laps[e]))
    out["laps"] = laps
    if res.endurance is not None:
        out["endurance"] = _table(res.endurance)
        out["endurance_flags"] = _j(dict(res.endurance.attrs))
        cols = (["v", "coolant_C", "oil_C", "rpm", "fuel_power_kW", "P_engine_kW", "fan"] if car == "ic" else
                ["v", "soc", "V_pack", "I_pack", "cell_temp_C", "motor_temp_C", "P_acc_kW", "E_acc_kWh", "V_cell"])
        _safe(out, "endurance_trace", lambda: _endurance_trace(res.laps["endurance_all"], cols))
    if detail:
        sub = {}
        _safe(sub, "ggv", lambda: _ggv(res.tggv))
        _safe(sub, "aero", lambda: _aero(veh))
        _safe(sub, "suspension", lambda: _suspension(veh))
        _safe(sub, "brakes", lambda: _brakes(veh))
        _safe(sub, "frame", lambda: frame_eval((overrides or {}).get("geom:frame"), cfg))
        for e in ("autocross", "endurance"):
            if e in res.laps:
                n = len(res.endurance) if (e == "endurance" and res.endurance is not None) else 1
                _safe(sub, f"brake_temps_{e}", lambda e=e, n=n: _brake_temps(veh, res.laps[e], n))
        _safe(sub, "powertrain", lambda: _ic_powertrain(veh, res.tables) if car == "ic" else _ev_powertrain(veh, res.tables))
        if car == "ev":
            import fsae_ev as ev
            for e in ("autocross", "endurance"):
                if e in res.laps:
                    _safe(sub, f"energy_{e}", lambda e=e: _j(ev.energy_breakdown(res.laps[e])))
            st = objs["_status"]
            sub["status"] = _table(st[~st["status"].str.startswith("ASSUMED")], index=False)
            sub["status_counts"] = _j(ev.status_counts(st))
            sub["limit_help"] = _j(ev.LIMIT_HELP)
        out["subsystems"] = sub
    out["solve_s"] = _num(time.time() - t_start, 3)
    return out


def sweep(car, key, values, events=("acceleration", "skidpad", "autocross"), overrides=None):
    """One input over a list of values: event times and points for each."""
    rows = []
    for v in values:
        ov = dict(overrides or {})
        ov[key] = v
        try:
            r = run(car, ov, events=events, detail=False)
            row = {"value": _j(v)}
            for e, d in r["events"].items():
                row[f"{e}_s"] = d["time_s"]
                row[f"{e}_pts"] = d["points"]
            rows.append(row)
        except Exception as ex:  # noqa: BLE001
            rows.append({"value": _j(v), "error": f"{type(ex).__name__}: {ex}"})
    return {"car": car, "key": key, "rows": rows}


def ev_studies(overrides=None, power_limits=(80, 70, 60, 50, 40, 30)):
    """The two EV studies: endurance against the accumulator power limit, and events against the reduction ratio."""
    import fsae_ev as ev
    objs = _load("ev", overrides)
    cfg, evc, bcfg = objs["vehicle"], objs["ev"], objs["pack"]
    veh = ev.EVVehicle(cfg, evc, bcfg)
    tggv = veh.traction_ggv()
    tracks = ev.make_tracks(cfg, cfg.sim.ds)
    out = {}
    if "endurance" in tracks:
        _safe(out, "energy_sweep", lambda: _table(ev.energy_sweep(veh, tggv, tracks["endurance"], power_limits,
                                                                    cfg.events.endurance_distance, cfg.events.tmin_endurance)))
    _safe(out, "gearing_sweep", lambda: _table(ev.gearing_sweep(cfg, evc, bcfg, tggv, tracks)))
    return out


def tracks(car="ic"):
    """Centre lines of the event tracks (for the track page)."""
    from fsae_vd.track import load_track
    cfg = _load(car)["vehicle"]
    out = {}
    for e in ("autocross", "endurance"):
        f = getattr(cfg.events, f"{e}_track")
        if f:
            t = load_track(f, 1.0)
            idx = _pick(len(t.s), 1200)
            out[e] = {"file": Path(f).name, "length_m": _num(t.length), "s": _j(t.s[idx]), "x": _j(t.x[idx]),
                      "y": _j(t.y[idx]), "kappa": _j(t.kappa[idx])}
    return out


# --------------------------------------------------------------------------- tracks
TRACK_DIRS = ("data/tracks/ic_tracks2026", "data/tracks")


def track_files():
    """Track centre lines in the bundle: {file name: path}."""
    out = {}
    for d in TRACK_DIRS:
        for f in sorted((IC / d).glob("*.csv")):
            out.setdefault(f.name, f)
    return out


def _apply_tracks(cfg, choice):
    """choice = {'autocross': 'file.csv', 'endurance': 'file.csv'}"""
    files = track_files()
    for e in ("autocross", "endurance"):
        f = (choice or {}).get(e)
        if f:
            if f not in files:
                raise ValueError(f"track file {f!r} is not in the bundle")
            setattr(cfg.events, f"{e}_track", str(files[f]))


# --------------------------------------------------------------------------- suspension hard points
# how the points are joined, for drawing (names as in the OptimumK export, left corner)
HP_LINKS = [("CHAS_LowFor", "UPRI_LowPnt", "arm"), ("CHAS_LowAft", "UPRI_LowPnt", "arm"),
            ("CHAS_UppFor", "UPRI_UppPnt", "arm"), ("CHAS_UppAft", "UPRI_UppPnt", "arm"),
            ("UPRI_LowPnt", "UPRI_UppPnt", "upright"), ("UPRI_UppPnt", "UPRI_TiePnt", "upright"), ("UPRI_LowPnt", "UPRI_TiePnt", "upright"),
            ("CHAS_TiePnt", "UPRI_TiePnt", "tie"), ("NSMA_PPAttPnt", "ROCK_RodPnt", "rod"),
            ("ROCK_RodPnt", "CHAS_RocPiv", "rocker"), ("CHAS_RocPiv", "ROCK_CoiPnt", "rocker"), ("ROCK_RodPnt", "ROCK_CoiPnt", "rocker"),
            ("ROCK_CoiPnt", "CHAS_AttPnt", "damper"), ("NSMA_UBarAttPnt", "UBAR_AttPnt", "arb"), ("UBAR_AttPnt", "CHAS_PivPnt", "arb")]
HP_NAMES = {"CHAS_LowFor": "lower arm, front chassis pivot", "CHAS_LowAft": "lower arm, rear chassis pivot",
            "CHAS_UppFor": "upper arm, front chassis pivot", "CHAS_UppAft": "upper arm, rear chassis pivot",
            "UPRI_LowPnt": "lower ball joint", "UPRI_UppPnt": "upper ball joint", "CHAS_TiePnt": "tie rod, chassis end",
            "UPRI_TiePnt": "tie rod, upright end", "NSMA_PPAttPnt": "pull rod, outboard end", "CHAS_AttPnt": "damper, chassis end",
            "CHAS_RocAxi": "rocker axis point", "CHAS_RocPiv": "rocker pivot", "ROCK_RodPnt": "rocker, rod end",
            "ROCK_CoiPnt": "rocker, damper end", "NSMA_UBarAttPnt": "ARB drop link, rocker end", "UBAR_AttPnt": "ARB blade tip",
            "CHAS_PivPnt": "ARB pivot"}
_BASE_HP = {}


def _hp_file(which):
    """The baseline hard-point file of an axle (never an edited copy)."""
    if which not in _BASE_HP:
        import fsae_vd as vd
        cfg = vd.load_config(str(IC / "configs" / "ct17_baseline.yaml"))
        _BASE_HP[which] = getattr(cfg, which).hardpoint_file
    return _BASE_HP[which]


def hardpoints():
    """Baseline hard points of both axles (left corner; export axes: x forward, y left, z up; metres)."""
    from fsae_vd import hardpoints as hp
    out = {"links": [list(l) for l in HP_LINKS], "names": HP_NAMES}
    for which in ("front", "rear"):
        f = _hp_file(which)
        if not f:
            continue
        d = hp.load_optimumk_csv(f)
        out[which] = {"points": {k: _j(v, 7) for k, v in d["points"].items()}, "wheel": _j(d["wheel"], 6), "title": d["name"]}
    return out


def _apply_hardpoints(cfg, edits):
    """edits = {'front': {point name: [x, y, z] in metres}, 'rear': {...}}. The edited export is written next to
    the temp files and the car config pointed at it, so every model that reads the hard points sees the change."""
    import hashlib
    import tempfile
    for which in ("front", "rear"):
        ed = {k: v for k, v in ((edits or {}).get(which) or {}).items() if v is not None}
        if not ed:
            continue
        src = Path(_hp_file(which))
        out = []
        for ln in src.read_text(encoding="utf-8-sig", errors="replace").splitlines():
            c = ln.split(",")
            name = c[1].strip() if len(c) > 8 else ""
            key = name[:-2] if name.endswith("_L") else name
            if key in ed:
                x, y, z = (1e3 * float(v) for v in ed[key])
                c[2:5] = [f"{x:.4f}", f"{y:.4f}", f"{z:.4f}"]
                c[6:9] = [f"{x:.4f}", f"{-y:.4f}", f"{z:.4f}"]
                ln = ",".join(c)
            out.append(ln)
        text = "\n".join(out) + "\n"
        d = Path(tempfile.gettempdir()) / "lapsim_hardpoints"
        d.mkdir(parents=True, exist_ok=True)
        f = d / f"{which}_{hashlib.sha1(text.encode()).hexdigest()[:16]}.csv"
        if not f.exists():
            f.write_text(text)
        getattr(cfg, which).hardpoint_file = str(f)


# --------------------------------------------------------------------------- frame
MATERIALS = {
    "4130 steel": {"E": 205e9, "G": 80e9, "density": 7850.0},
    "1020 mild steel": {"E": 200e9, "G": 77e9, "density": 7870.0},
    "6061-T6 aluminium": {"E": 68.9e9, "G": 26e9, "density": 2700.0},
    "Ti-3Al-2.5V titanium": {"E": 100e9, "G": 40e9, "density": 4480.0},
}
FRAME_TOL = 0.022          # ends this close are one welded node (as in fsae_frame.FrameModel)
_FRAME = {}


def _frame_tubes():
    if "tubes" not in _FRAME:
        from fsae_frame import model, tubes
        _FRAME["tubes"] = model.to_car(tubes.load_tubes(IC / "data" / "frame" / "ct17_spaceframe_tubes.csv"))
    return _FRAME["tubes"]


def frame_geometry():
    """The frame as an editable structure (car axes: x rearward from the front axle, y right, z up; metres).

    nodes   welded joints: tube ends closer than FRAME_TOL are one node
    tubes   [node0, node1, size index] + the small offset of each end from its node, so the baseline is exact.
            An end coped onto the side of another tube has node -1 and a 'host' [tube, fraction along it]: it
            follows that tube when its nodes move.
    sizes   [{shape, od, wall, material}] (metres)
    """
    if "geometry" in _FRAME:
        return _FRAME["geometry"]
    tb = _frame_tubes()
    n = len(tb)
    ends = np.array([p for t in tb for p in (t.p0, t.p1)])
    host = {}
    for i, t in enumerate(tb):
        for k, e in enumerate((t.p0, t.p1)):
            best = None
            for j, u in enumerate(tb):
                if j == i:
                    continue
                d = u.p1 - u.p0
                sp = float(np.clip(((e - u.p0) @ d) / (d @ d), 0.0, 1.0))
                dist = float(np.linalg.norm(u.p0 + sp * d - e))
                if dist < FRAME_TOL and FRAME_TOL < sp * u.length < u.length - FRAME_TOL and (best is None or dist < best[0]):
                    best = (dist, j, sp)
            if best:
                host[2 * i + k] = best[1:]
    parent = list(range(2 * n))

    def find(a):
        while parent[a] != a:
            parent[a] = parent[parent[a]]
            a = parent[a]
        return a

    free = [a for a in range(2 * n) if a not in host]
    for a in free:
        d = np.linalg.norm(ends - ends[a], axis=1)
        for b in np.flatnonzero(d < FRAME_TOL):
            if b > a and b not in host and b // 2 != a // 2:
                parent[find(int(b))] = find(a)
    roots = sorted({find(a) for a in free})
    idx = {r: k for k, r in enumerate(roots)}
    members = {k: [] for k in range(len(roots))}
    for a in free:
        members[idx[find(a)]].append(a)
    nodes = np.array([ends[m].mean(0) for m in members.values()])
    sizes, size_of = [], {}
    tubes = []
    for i, t in enumerate(tb):
        key = (t.shape, round(t.od / 0.0254, 3), round(t.wall / 0.0254, 3))      # stock sizes, as tubes.summary groups them
        if key not in size_of:
            size_of[key] = len(sizes)
            sizes.append({"shape": t.shape, "od": _num(key[1] * 0.0254, 6), "wall": _num(key[2] * 0.0254, 6), "material": "4130 steel"})
        row = {"n": [-1, -1], "off": [[0, 0, 0], [0, 0, 0]], "host": [None, None], "size": size_of[key], "bend": bool(t.bend)}
        for k in (0, 1):
            a = 2 * i + k
            if a in host:
                j, sp = host[a]
                u = tb[j]
                row["host"][k] = [int(j), _num(sp, 7)]
                row["off"][k] = _j(ends[a] - (u.p0 + sp * (u.p1 - u.p0)), 6)
            else:
                nd = idx[find(a)]
                row["n"][k] = int(nd)
                row["off"][k] = _j(ends[a] - nodes[nd], 6)
        tubes.append(row)
    _FRAME["geometry"] = {"nodes": _j(nodes, 7), "tubes": tubes, "sizes": sizes, "materials": MATERIALS, "use_stiffness": False}
    return _FRAME["geometry"]


def _frame_build(state):
    """Tube list + per-tube material from an editable frame state (frame_geometry() format)."""
    from fsae_frame.tubes import Tube
    nodes = np.array(state["nodes"], float)
    rows = state["tubes"]
    P = [[None, None] for _ in rows]
    for _ in range(8):                                    # coped ends follow their host tube: resolve in passes
        todo = False
        for i, r in enumerate(rows):
            for k in (0, 1):
                if P[i][k] is not None:
                    continue
                off = np.array(r["off"][k], float)
                if r["n"][k] is not None and r["n"][k] >= 0:
                    P[i][k] = nodes[r["n"][k]] + off
                else:
                    j, sp = r["host"][k]
                    if j < len(rows) and P[j][0] is not None and P[j][1] is not None:
                        P[i][k] = P[j][0] + sp * (P[j][1] - P[j][0]) + off
                    else:
                        todo = True
        if not todo:
            break
    tubes, mats = [], []
    for i, r in enumerate(rows):
        if P[i][0] is None or P[i][1] is None:
            raise ValueError(f"tube {i + 1}: an end is joined to a tube that no longer exists")
        z = state["sizes"][r["size"]]
        if not (z["od"] > 0 and 0 < z["wall"] <= z["od"] / 2):
            raise ValueError(f"tube size {r['size'] + 1}: wall must be between 0 and half the outer size")
        if np.linalg.norm(P[i][1] - P[i][0]) < 1e-4:
            raise ValueError(f"tube {i + 1} has zero length")
        tubes.append(Tube(P[i][0], P[i][1], float(z["od"]), float(z["wall"]), z.get("shape", "round"), bool(r.get("bend")), -1))
        mats.append(z.get("material", "4130 steel"))
    return tubes, mats


def _frame_props(state, pickups):
    """Mass, CG, torsional stiffness and per-tube strain-energy share of a frame state."""
    from fsae_frame import model
    tubes, mats = _frame_build(state)
    lib = dict(MATERIALS)
    lib.update(state.get("materials") or {})
    ref = model.STEEL_4130
    L = np.array([t.length for t in tubes])
    m = np.array([lib[k]["density"] * t.area * t.length for t, k in zip(tubes, mats)])
    mid = np.array([0.5 * (t.p0 + t.p1) for t in tubes])
    out = {"count": len(tubes), "length": float(L.sum()), "mass": float(m.sum()), "cg": (m[:, None] * mid).sum(0) / m.sum(),
           "tube_length": L, "tube_mass": m}
    by = {}
    for r, l, mm in zip(state["tubes"], L, m):
        b = by.setdefault(r["size"], [0, 0.0, 0.0])
        b[0] += 1
        b[1] += l
        b[2] += mm
    out["sizes"] = [{"count": by.get(i, [0, 0, 0])[0], "length": by.get(i, [0, 0, 0])[1], "mass": by.get(i, [0, 0, 0])[2]}
                    for i in range(len(state["sizes"]))]
    try:
        fm = model.FrameModel(tubes, pickups, ref, FRAME_TOL)
        for e in fm.elements:                              # each tube's own material: scale EA, EI, GJ
            if e.tube >= 0:
                mt = lib[mats[e.tube]]
                e.A *= mt["E"] / ref["E"]
                e.I *= mt["E"] / ref["E"]
                e.J *= mt["G"] / ref["G"]
        pieces, loose = fm.connectivity()
        out["pieces"], out["free_ends"] = int(pieces), int(len(loose))
        tt = fm.torsion_test()
        out["k_per_deg"], out["k_per_rad"] = float(tt["stiffness_per_deg"]), float(tt["stiffness"])
        out["energy"] = [float(tt["energy_by_tube"].get(i, 0.0)) for i in range(len(tubes))]
    except Exception as e:  # noqa: BLE001
        out["stiffness_error"] = f"{type(e).__name__}: {e}"
    return out


def _pickups(cfg):
    from fsae_frame import model
    return model.load_pickups(cfg.front.hardpoint_file, cfg.rear.hardpoint_file)


def _frame_base(cfg=None):
    if "base" not in _FRAME:
        import fsae_vd as vd
        base_cfg = vd.load_config(str(IC / "configs" / "ct17_baseline.yaml"))
        _FRAME["base"] = _frame_props(frame_geometry(), _pickups(base_cfg))
    return _FRAME["base"]


def _apply_frame(cfg, state):
    """The car with the edited frame in place of the baseline one: mass, CG and (when asked) torsional stiffness."""
    from fsae_frame import lapsim as FL
    base, new = _frame_base(), _frame_props(state, _pickups(cfg))
    c = FL.swap_frame(cfg, base, new)
    if state.get("use_stiffness") and "k_per_rad" in new:
        c.mass.chassis_torsional_stiffness = new["k_per_rad"]
    return c


def frame_eval(state=None, cfg=None):
    """Numbers and drawing data for a frame state (None = the baseline frame)."""
    import fsae_vd as vd
    from fsae_frame import lapsim as FL
    cfg = cfg or vd.load_config(str(IC / "configs" / "ct17_baseline.yaml"))
    base = _frame_base()
    st = state or frame_geometry()
    new = _frame_props(st, _pickups(cfg)) if state else base
    tubes, _ = _frame_build(st)
    out = {k: _j(new.get(k)) for k in ("count", "length", "mass", "cg", "pieces", "free_ends", "k_per_deg", "stiffness_error", "sizes")}
    out["energy"] = _j(np.array(new["energy"]), 4) if "energy" in new else None
    out["tube_length"] = _j(new["tube_length"], 5)
    out["tube_mass"] = _j(new["tube_mass"], 4)
    out["tubes"] = [[_num(v, 5) for v in (*t.p0, *t.p1)] + [_num(t.od * 1e3, 4), _num(t.wall * 1e3, 3)] for t in tubes]
    out["base"] = {"mass": _num(base["mass"]), "cg": _j(base["cg"]), "k_per_deg": _num(base.get("k_per_deg"))}
    out["edited"] = bool(state)
    out["use_stiffness"] = bool(state and state.get("use_stiffness"))
    out["car"] = {"mass_kg": _num(cfg.mass.total), "cg_height_mm": _num(cfg.mass.cg_height * 1e3),
                  "front_weight_%": _num(100 * cfg.mass.front_weight_fraction),
                  "frame_twist_in_sim": cfg.mass.chassis_torsional_stiffness is not None}
    return out


# --------------------------------------------------------------------------- geometry + live preview
def geometry(car="ic"):
    """Everything the interactive figures start from: hard points, the frame, the track files, the car's layout."""
    out = {}
    _safe(out, "hardpoints", hardpoints)
    _safe(out, "frame", frame_geometry)
    _safe(out, "tracks", lambda: sorted(track_files()))
    cfg = _load(car)["vehicle"]
    out["track_choice"] = {e: Path(getattr(cfg.events, f"{e}_track")).name for e in ("autocross", "endurance")
                           if getattr(cfg.events, f"{e}_track")}
    rho = float(cfg.aero.density())

    def _elements():
        rows = wing_elements()
        for r in rows:                                     # each element alone, at its own angle, 15 m/s
            r["result"] = airfoil(r["section"], r["chord"], r["angle_deg"], 15.0, rho, points=r.get("unit"))
        return rows
    _safe(out, "wing_elements", _elements)
    _safe(out, "tunnel", tunnel_shapes)

    def _stacks():
        if not wing_sections():
            return None
        zones = wing_zones()
        for z in zones:
            z["result"] = wing_stack(z["elements"], 15.0, rho)
        return {"zones": zones, "endplates": (wing_sections() or {}).get("endplate_boxes", {})}
    _safe(out, "wing_stacks", _stacks)
    out["air_density"] = _num(rho, 4)
    return out


PREVIEW_BLOCKS = ("aero", "suspension", "brakes", "frame", "powertrain", "ggv")


def preview(car, overrides=None, blocks=PREVIEW_BLOCKS):
    """The subsystem data for edited inputs WITHOUT running the events: fast enough to follow a slider or a drag."""
    t0 = time.time()
    car = car.lower()
    objs = _load(car, overrides)
    cfg = objs["vehicle"]
    if car == "ic":
        import fsae_pt as pt
        veh = pt.PTVehicle(cfg, objs["pt"])
    else:
        import fsae_ev as ev
        veh = ev.EVVehicle(cfg, objs["ev"], objs["pack"])
    out = {}
    for b in blocks:
        if b == "aero":
            _safe(out, b, lambda: _aero(veh))
        elif b == "suspension":
            _safe(out, b, lambda: _suspension(veh))
        elif b == "brakes":
            _safe(out, b, lambda: _brakes(veh))
        elif b == "frame":
            _safe(out, b, lambda: frame_eval((overrides or {}).get("geom:frame"), cfg))
        elif b == "ggv":
            _safe(out, b, lambda: _ggv(veh.traction_ggv()))
        elif b == "powertrain":
            if car == "ic":
                import fsae_pt as pt
                _safe(out, b, lambda: _ic_powertrain(veh, pt.PTTables(veh)))
            else:
                _safe(out, b, lambda: _ev_powertrain(veh, veh.tables0))
    return {"car": car, "blocks": out, "mass_kg": _num(veh.m), "notes": [str(n) for n in veh.warnings], "solve_s": _num(time.time() - t0, 3)}


# --------------------------------------------------------------------------- brakes: rotor temperature
def _brake_powers(veh, v, ax, braking):
    """Friction power at ONE front and ONE rear brake along a lap [W]: the line pressure that gives each
    braking point's deceleration (drag helps), times wheel speed."""
    from fsae_vd import brakes
    v = np.asarray(v, float)
    g = np.where(braking, np.clip(-np.asarray(ax, float) / 9.80665, 0.0, None), 0.0)
    gg, vg = np.linspace(0.0, max(2.5, float(g.max()) + 0.1), 14), np.linspace(0.0, max(40.0, float(v.max())), 7)
    tab = np.array([[brakes.line_pressure_for(veh, float(a), float(u))[0] for u in vg] for a in gg])
    from scipy.interpolate import RegularGridInterpolator
    p_f = RegularGridInterpolator((gg, vg), tab)(np.column_stack([g, np.clip(v, 0, vg[-1])]))
    p_f = np.where(g > 1e-3, p_f, 0.0)
    return brakes.wheel_brake_power(veh.cfg, v, p_f)


def _brake_temps(veh, lr, laps=1, T_air=25.0):
    """Rotor temperatures over ``laps`` repeats of one simulated lap (lumped rotor model, fsae_vd.brakes)."""
    from fsae_vd import brakes
    front, rear = brakes.rotors(veh.cfg)
    lim = np.asarray(lr.limit)
    braking = (lim == "braking") if lim.dtype.kind in "OUS" else (np.asarray(lr.ax) < -0.5)
    ch = getattr(lr, "pt", None) or {}
    if "P_brake_front_kW" in ch and "P_brake_rear_kW" in ch:          # EV: the friction brakes' share after regen, per axle
        Pf, Pr = 500.0 * np.asarray(ch["P_brake_front_kW"], float), 500.0 * np.asarray(ch["P_brake_rear_kW"], float)
    else:
        Pf, Pr = _brake_powers(veh, lr.v, lr.ax, braking)
    t1, n = np.asarray(lr.t, float), int(max(1, laps))
    t = np.concatenate([t1 + k * float(t1[-1]) for k in range(n)])
    v = np.tile(np.asarray(lr.v, float), n)
    Tf = brakes.rotor_temperature(t, np.tile(Pf, n), v, front, T_air, T_air)
    Tr = brakes.rotor_temperature(t, np.tile(Pr, n), v, rear, T_air, T_air)
    idx = _pick(len(t), 1400)
    dt = np.diff(t1, append=t1[-1])
    return {"t": _j(t[idx]), "T_front_C": _j(Tf[idx], 4), "T_rear_C": _j(Tr[idx], 4), "laps": n,
            "peak_front_C": _num(Tf.max(), 4), "peak_rear_C": _num(Tr.max(), 4), "end_front_C": _num(Tf[-1], 4), "end_rear_C": _num(Tr[-1], 4),
            "energy_front_kJ_per_lap": _num(float((Pf * dt).sum()) / 1e3, 4), "energy_rear_kJ_per_lap": _num(float((Pr * dt).sum()) / 1e3, 4),
            "peak_power_front_kW": _num(Pf.max() / 1e3, 4), "peak_power_rear_kW": _num(Pr.max() / 1e3, 4),
            "rear_rotor_assumed_equal_to_front": bool(getattr(rear, "rear_assumed", False)), "T_air_C": T_air}


def brake_stops(car, overrides=None, v_hi_kph=100.0, v_lo_kph=30.0, n=15, decel_g=1.4, accel_g=0.6):
    """Rotor temperatures over repeated identical stops (fade / sizing check)."""
    from fsae_vd import brakes
    objs = _load(car, overrides)
    if car == "ic":
        import fsae_pt as pt
        veh = pt.PTVehicle(objs["vehicle"], objs["pt"])
    else:
        import fsae_ev as ev
        veh = ev.EVVehicle(objs["vehicle"], objs["ev"], objs["pack"])
    df, peaks = brakes.repeated_stops(veh, v_hi_kph / 3.6, v_lo_kph / 3.6, int(n), float(decel_g), float(accel_g))
    idx = _pick(len(df), 1400)
    ef, er = brakes.stop_energy(veh, v_hi_kph / 3.6, v_lo_kph / 3.6)
    return {"t": _j(df["t"].to_numpy()[idx]), "T_front_C": _j(df["T_front_C"].to_numpy()[idx], 4), "T_rear_C": _j(df["T_rear_C"].to_numpy()[idx], 4),
            "v_kph": _j(3.6 * df["v"].to_numpy()[idx], 4), "peaks_front_C": _j(peaks["T_front_C"].to_numpy(), 4), "peaks_rear_C": _j(peaks["T_rear_C"].to_numpy(), 4),
            "energy_per_stop_front_kJ": _num(ef / 1e3, 4), "energy_per_stop_rear_kJ": _num(er / 1e3, 4), "p_front_peak_bar": _num(df["p_front_bar"].max(), 4)}


# --------------------------------------------------------------------------- one airfoil on its own
def _naca4(code, n=90):
    """Unit-chord NACA 4-digit section, upper trailing edge -> leading edge -> lower trailing edge."""
    code = str(code).lower().replace("naca", "").strip().zfill(4)
    if not (code.isdigit() and len(code) == 4):
        raise ValueError(f"'{code}' is not a NACA 4-digit code (for example 6412)")
    m, p, t = int(code[0]) / 100.0, int(code[1]) / 10.0, int(code[2:]) / 100.0
    x = 0.5 * (1 - np.cos(np.linspace(0, np.pi, n)))
    yt = 5 * t * (0.2969 * np.sqrt(x) - 0.1260 * x - 0.3516 * x ** 2 + 0.2843 * x ** 3 - 0.1036 * x ** 4)
    if m > 0 and p > 0:
        yc = np.where(x < p, m / p ** 2 * (2 * p * x - x ** 2), m / (1 - p) ** 2 * ((1 - 2 * p) + 2 * p * x - x ** 2))
        dyc = np.where(x < p, 2 * m / p ** 2 * (p - x), 2 * m / (1 - p) ** 2 * (p - x))
    else:
        yc, dyc = np.zeros_like(x), np.zeros_like(x)
    th = np.arctan(dyc)
    up = np.stack([(x - yt * np.sin(th))[::-1], (yc + yt * np.cos(th))[::-1]], axis=1)
    lo = np.stack([(x + yt * np.sin(th))[1:], (yc - yt * np.cos(th))[1:]], axis=1)
    return np.vstack([up, lo])


def _section_from_text(text, n=90):
    """Coordinates pasted from a Selig .dat file (x y per line, trailing edge -> upper -> leading edge -> lower),
    scaled to unit chord and re-spaced."""
    pts = []
    for ln in str(text).splitlines():
        c = ln.replace(",", " ").split()
        try:
            pts.append([float(c[0]), float(c[1])])
        except (ValueError, IndexError):
            continue
    a = np.array(pts)
    if len(a) < 12:
        raise ValueError("need at least 12 coordinate rows (x y per line, Selig order)")
    if a[0, 0] > 1.5:                         # Lednicer: counts line, then upper LE->TE and lower LE->TE
        nu = int(a[0, 0])
        a = np.vstack([a[1:1 + nu][::-1], a[1 + nu + 1:]])
    a = a - a[np.argmin(a[:, 0])]
    a = a / (a[:, 0].max() or 1.0)
    d = np.r_[0, np.cumsum(np.hypot(*np.diff(a, axis=0).T))]
    le = int(np.argmin(a[:, 0]))
    beta = np.linspace(0, np.pi, n)
    s_up = d[le] * 0.5 * (1 - np.cos(beta))                      # cosine spacing: dense at both ends of each surface
    s_lo = d[le] + (d[-1] - d[le]) * 0.5 * (1 - np.cos(beta))[1:]
    ss = np.r_[s_up, s_lo]
    return np.stack([np.interp(ss, d, a[:, 0]), np.interp(ss, d, a[:, 1])], axis=1)


def _panel(poly, alpha_deg):
    """Hess-Smith panel method (sources + one vortex strength, Kutta condition) for one closed section in
    uniform flow. Inviscid, incompressible: no stall and no profile drag. Returns (cl, cm about c/4, xm, cp)."""
    P = np.asarray(poly, float)[::-1]                 # lower TE -> LE -> upper TE (clockwise)
    if np.hypot(*(P[0] - P[-1])) > 1e-9:
        P = np.vstack([P, P[0]])
    x, y = P[:, 0], P[:, 1]
    N = len(x) - 1
    xm, ym = 0.5 * (x[:-1] + x[1:]), 0.5 * (y[:-1] + y[1:])
    dx, dy = np.diff(x), np.diff(y)
    S, th = np.hypot(dx, dy), np.arctan2(dy, dx)
    al = np.radians(alpha_deg)
    X1, Y1 = xm[:, None] - x[None, :-1], ym[:, None] - y[None, :-1]
    X2, Y2 = xm[:, None] - x[None, 1:], ym[:, None] - y[None, 1:]
    F = np.log(np.hypot(X2, Y2) / np.hypot(X1, Y1))
    B = np.arctan2(Y2 * X1 - X2 * Y1, X2 * X1 + Y2 * Y1)
    np.fill_diagonal(F, 0.0)
    np.fill_diagonal(B, np.pi)
    D = th[:, None] - th[None, :]
    sn, cs = np.sin(D), np.cos(D)
    A = np.zeros((N + 1, N + 1))
    A[:N, :N] = (sn * F + cs * B) / (2 * np.pi)
    A[:N, N] = ((cs * F - sn * B) / (2 * np.pi)).sum(1)
    b = np.zeros(N + 1)
    b[:N] = np.sin(th - al)
    Tq = (sn * B - cs * F) / (2 * np.pi)              # tangential velocity from each source panel
    Tg = (sn * F + cs * B) / (2 * np.pi)              # ... and from the vortex on each panel
    for i in (0, N - 1):
        A[N, :N] += Tq[i]
        A[N, N] += Tg[i].sum()
    b[N] = -np.cos(th[0] - al) - np.cos(th[N - 1] - al)
    sol = np.linalg.solve(A, b)
    q, gam = sol[:N], sol[N]
    vt = np.cos(th - al) + Tq @ q + gam * Tg.sum(1)
    cp = 1 - vt ** 2
    c = x.max() - x.min()
    fx, fy = (cp * dy).sum() / c, -(cp * dx).sum() / c                 # pressure force / (q c), body axes
    cl = fy * np.cos(al) - fx * np.sin(al)
    cm = ((cp * dx) * (xm - x.min() - 0.25 * c) + (cp * dy) * ym).sum() / c ** 2
    return float(cl), float(cm), xm, ym, cp


def airfoil(section="naca4412", chord=0.35, angle_deg=5.0, speed=15.0, rho=1.12, coordinates=None, points=None):
    """One section alone in free air: shape, pressure distribution, lift and moment at ``angle_deg``, and the lift
    curve. Loads are per metre of span. For a wing element the angle is its chord angle to the airflow."""
    if points is not None:                              # a section already as unit-chord points (the CAD cut)
        poly = np.asarray(points, float)
    else:
        poly = _section_from_text(coordinates) if coordinates else _naca4(section)
    cl, cm, xm, ym, cp = _panel(poly, angle_deg)
    al = np.arange(-6.0, 20.1, 2.0)
    curve = [_panel(poly, a)[0] for a in al]
    qd = 0.5 * rho * speed ** 2
    le = int(np.argmin(poly[:, 0]))
    thick = float(np.max(np.interp(np.linspace(0, 1, 60), poly[:le + 1][::-1, 0], poly[:le + 1][::-1, 1])
                         - np.interp(np.linspace(0, 1, 60), poly[le:, 0], poly[le:, 1])))
    return {"x": _j(poly[:, 0]), "y": _j(poly[:, 1]), "cp_x": _j(xm), "cp_y": _j(ym), "cp": _j(cp, 4),
            "cl": _num(cl, 4), "cm_quarter_chord": _num(cm, 4), "alpha": _j(al), "cl_curve": _j(np.array(curve), 4),
            "cl_per_deg": _num((curve[4] - curve[3]) / 2.0, 4), "alpha_zero_lift_deg": _num(float(np.interp(0.0, curve, al)), 3),
            "lift_N_per_m": _num(qd * chord * cl, 4), "moment_Nm_per_m": _num(qd * chord ** 2 * cm, 4), "thickness_%": _num(100 * thick, 3),
            "reynolds": _num(rho * speed * chord / 1.81e-5, 3), "q_Pa": _num(qd, 4), "min_cp": _num(float(cp.min()), 4)}


_WINGS = {}
# The wings on the car (team, 2026-11-15): front F1 + F2 + F3 inboard of the middle endplate and F1 + F2 outboard of
# it; rear R1 to R4. The CAD assembly's fourth front flap (611-11-17IC F4) is not on the car and is never used here.
DESCRIBED = ("F1", "F2", "F3", "R1", "R2", "R3", "R4")


def wing_sections():
    """The wing sections cut from the CAD (web/export_wings.py -> bridge/wing_sections.json), or None."""
    if "cad" not in _WINGS:
        f = Path(__file__).resolve().parent / "wing_sections.json"
        _WINGS["cad"] = __import__("json").loads(f.read_text()) if f.exists() else None
    return _WINGS["cad"]


def wing_elements():
    """Every wing element: [{name, wing, source, section, chord, angle_deg, thickness_pct, camber_pct, y, span, unit}].
    From the CAD cut when it is in the bundle (the real sections), else the NACA stand-ins in configs/ct17_cfd.yaml."""
    cad = wing_sections()
    if cad:
        return [{"name": e["name"], "wing": e["wing"], "source": "cad", "section": "section cut from the CAD", "chord": e["chord"], "angle_deg": e["angle_deg"],
                 "thickness_pct": e["thickness_pct"], "camber_pct": e["camber_pct"], "y": e["y"], "span": e["span"], "unit": e["unit"],
                 "inverted": e.get("inverted", True), "le": _j(np.array(e["poly"])[np.argmax(np.linalg.norm(np.array(e["poly"]) - np.array(e["poly"][0]), axis=1))], 6),
                 "described": True} for e in cad["elements"] if e["name"] in DESCRIBED]
    import yaml
    f = IC / "configs" / "ct17_cfd.yaml"
    y = yaml.safe_load(f.read_text()) or {}
    out = []
    for wing in ("front_wing", "rear_wing"):
        for k, e in enumerate((y.get(wing) or {}).get("elements") or []):
            name = f"{'F' if wing == 'front_wing' else 'R'}{k + 1}"
            out.append({"name": name, "wing": wing, "source": "naca", "section": str(e.get("section", "")), "chord": _num(e.get("chord")),
                        "angle_deg": _num(e.get("angle_deg")), "span": _num((y.get(wing) or {}).get("span")), "described": name in DESCRIBED})
    return out


# --------------------------------------------------------------------------- several elements together
def _clockwise_from_te(poly):
    """A section as the panel method wants it: closed, clockwise, starting and ending at the trailing edge."""
    P = np.asarray(poly, float)
    if np.hypot(*(P[0] - P[-1])) < 1e-9:
        P = P[:-1]
    if 0.5 * (np.dot(P[:, 0], np.roll(P[:, 1], -1)) - np.dot(P[:, 1], np.roll(P[:, 0], -1))) > 0:      # counter-clockwise
        P = np.vstack([P[:1], P[1:][::-1]])
    return np.vstack([P, P[:1]])


def _influence(xm, ym, th, x, y, th_j, self_term):
    """Normal and tangential velocity at panel mid-points (xm, ym, angle th) from unit-strength source and vortex
    panels with nodes (x, y) and angles th_j: (An, At, Vn, Vt). Hess-Smith."""
    X1, Y1 = xm[:, None] - x[None, :-1], ym[:, None] - y[None, :-1]
    X2, Y2 = xm[:, None] - x[None, 1:], ym[:, None] - y[None, 1:]
    F = np.log(np.hypot(X2, Y2) / np.hypot(X1, Y1))
    B = np.arctan2(Y2 * X1 - X2 * Y1, X2 * X1 + Y2 * Y1)
    if self_term:
        np.fill_diagonal(F, 0.0)
        np.fill_diagonal(B, np.pi)
    D = th[:, None] - th_j[None, :]
    sn, cs = np.sin(D), np.cos(D)
    k = 1 / (2 * np.pi)
    return k * (sn * F + cs * B), k * (sn * B - cs * F), k * (cs * F - sn * B), k * (sn * F + cs * B)


def _panel_multi(bodies, ground=None):
    """Hess-Smith panel method for several closed sections in a uniform flow along +x (speed 1): sources on every
    panel, one vortex strength per section, a Kutta condition at every trailing edge. ``ground`` = height of a flat
    ground (the flow's mirror image below it makes the ground a streamline, as a road moving with the air does).
    Inviscid: no stall, no profile drag. Returns per section (fx, fz) per unit dynamic pressure per metre of span,
    and (x_mid, cp) along its surface."""
    P = [_clockwise_from_te(b) for b in bodies]
    xs, ys, own, first, last, n0 = [], [], [], [], [], 0
    for b, p in enumerate(P):
        n = len(p) - 1
        xs.append(p[:, 0]); ys.append(p[:, 1]); own += [b] * n
        first.append(n0); last.append(n0 + n - 1); n0 += n
    N, M, own = n0, len(P), np.array(own)
    xm = np.concatenate([0.5 * (x[:-1] + x[1:]) for x in xs]); ym = np.concatenate([0.5 * (y[:-1] + y[1:]) for y in ys])
    dx = np.concatenate([np.diff(x) for x in xs]); dy = np.concatenate([np.diff(y) for y in ys])
    th = np.arctan2(dy, dx)
    An, At, Vn, Vt = (np.zeros((N, N)) for _ in range(4))
    c0 = 0
    for b, (x, y) in enumerate(zip(xs, ys)):                              # each section's panels acting on every panel
        n = len(x) - 1
        sl = slice(c0, c0 + n)
        an, at, vn, vt = _influence(xm, ym, th, x, y, th[sl], False)
        # the section's own panels: the singular self term
        a2, t2, v2, w2 = _influence(xm[sl], ym[sl], th[sl], x, y, th[sl], True)
        an[sl], at[sl], vn[sl], vt[sl] = a2, t2, v2, w2
        An[:, sl], At[:, sl], Vn[:, sl], Vt[:, sl] = an, at, vn, vt
        if ground is not None:                                           # mirror image below the ground: same sources, opposite vortices
            yi = 2 * ground - y
            thi = np.arctan2(np.diff(yi), np.diff(x))
            an, at, vn, vt = _influence(xm, ym, th, x, yi, thi, False)
            An[:, sl] += an; At[:, sl] += at; Vn[:, sl] -= vn; Vt[:, sl] -= vt
        c0 += n
    A = np.zeros((N + M, N + M)); rhs = np.zeros(N + M)
    A[:N, :N] = An
    for b in range(M):
        A[:N, N + b] = Vn[:, own == b].sum(1)
    rhs[:N] = np.sin(th)                                                 # flow along +x: no flow through any panel
    for b in range(M):                                                    # Kutta: equal and opposite speeds either side of each trailing edge
        for i in (first[b], last[b]):
            A[N + b, :N] += At[i]
            for c in range(M):
                A[N + b, N + c] += Vt[i, own == c].sum()
        rhs[N + b] = -np.cos(th[first[b]]) - np.cos(th[last[b]])
    sol = np.linalg.solve(A, rhs)
    q, gam = sol[:N], sol[N:]
    vt = np.cos(th) + At @ q + sum(gam[b] * Vt[:, own == b].sum(1) for b in range(M))
    cp = 1 - vt ** 2
    out = []
    for b in range(M):
        k = own == b
        out.append({"fx": float((cp[k] * dy[k]).sum()), "fz": float(-(cp[k] * dx[k]).sum()), "x": xm[k], "z": ym[k], "cp": cp[k]})
    return out


def _stack_polys(names, d_angle=None, dz=0.0):
    """The named elements as they sit on the car (CAD cut), each turned by its d_angle [deg] about its own leading
    edge (trailing edge up = more wing) and the whole stack raised by dz [m]."""
    cad = wing_sections()
    if not cad:
        raise ValueError("the CAD sections are not in this build: run  python web/export_wings.py  then  python web/build.py")
    by = {e["name"]: e for e in cad["elements"]}
    out = []
    for n in names:
        if n not in by:
            raise ValueError(f"no wing element named {n}")
        p = np.array(by[n]["poly"], float)
        le = p[np.argmax(np.linalg.norm(p - p[0], axis=1))]
        a = np.radians(float((d_angle or {}).get(n, 0.0)))
        r = p - le
        p = np.stack([le[0] + r[:, 0] * np.cos(a) - r[:, 1] * np.sin(a), le[1] + r[:, 0] * np.sin(a) + r[:, 1] * np.cos(a) + dz], axis=1)
        out.append(p)
    return out


def wing_stack(names, speed=15.0, rho=1.12, d_angle=None, dz=0.0, ground=True, polys=None, chords=None):
    """Several wing elements solved together as they sit on the car, in ground effect: each element's pressure
    distribution and load per metre of span, and the stack's totals. 2-D and inviscid (see _panel_multi)."""
    by = {e["name"]: dict(e) for e in (wing_sections() or {"elements": []})["elements"]}
    if polys is not None:                              # sections as the wing designer drew them (car axes, metres)
        polys = [np.asarray(q, float) for q in polys]
        for k, n in enumerate(names):
            c = float(chords[k]) if chords else float(np.max(np.linalg.norm(polys[k] - polys[k][0], axis=1)))
            te, le = polys[k][0], polys[k][np.argmax(np.linalg.norm(polys[k] - polys[k][0], axis=1))]
            by[n] = {"chord": c, "angle_deg": float(np.degrees(np.arctan2(te[1] - le[1], te[0] - le[0])))}
        d_angle = None
    else:
        polys = _stack_polys(names, d_angle, dz)
    if ground and min(float(p[:, 1].min()) for p in polys) <= 0.004:
        raise ValueError("the wing is on the ground: raise it")
    res = _panel_multi(polys, 0.0 if ground else None)
    qd = 0.5 * rho * speed ** 2
    rows, tot_down, tot_x = [], 0.0, 0.0
    for n, p, r in zip(names, polys, res):
        c = by[n]["chord"]
        rows.append({"name": n, "chord_mm": _num(1e3 * c), "angle_deg": _num(by[n]["angle_deg"] + float((d_angle or {}).get(n, 0.0)), 4),
                     "cl": _num(-r["fz"] / c, 4), "downforce_N_per_m": _num(-r["fz"] * qd, 4), "fx_N_per_m": _num(r["fx"] * qd, 4),
                     "min_cp": _num(float(r["cp"].min()), 4), "x": _j(p[:, 0], 5), "z": _j(p[:, 1], 5), "cp_x": _j(r["x"], 5), "cp": _j(r["cp"], 4)})
        tot_down += -r["fz"] * qd
        tot_x += r["fx"] * qd
    ref = sum(by[n]["chord"] for n in names)
    return {"names": list(names), "elements": rows, "downforce_N_per_m": _num(tot_down, 4), "fx_N_per_m": _num(tot_x, 4),
            "cl_on_total_chord": _num(tot_down / qd / ref, 4), "total_chord_mm": _num(1e3 * ref), "speed": speed, "rho": rho,
            "ground": bool(ground), "lowest_point_mm": _num(1e3 * min(float(p[:, 1].min()) for p in polys))}


def wing_zones():
    """The span zones of both wings: [{wing, y, width (both sides), elements}]."""
    cad = wing_sections()
    out = []
    for wing in ("front_wing", "rear_wing"):
        for z in (cad or {}).get("zones", {}).get(wing, []):
            names = [n for n in z["elements"] if n in DESCRIBED]
            if names:
                out.append({"wing": wing, "y": z["y"], "width": _num(2 * (z["y"][1] - z["y"][0])), "elements": names})
    return out


# --------------------------------------------------------------------------- side-view outlines for the 2-D wind tunnel
def _place(section, chord, angle_deg, le):
    """A unit section as it sits on the car (x rearward, z up): inverted (suction side down), scaled, pitched
    trailing-edge-up by ``angle_deg`` and moved to its leading edge ``le``. As fsae_cfd.geometry.airfoils.place_element."""
    p = _naca4(section, 48).copy()
    p[:, 1] *= -1.0
    p *= chord
    a = np.radians(angle_deg)
    return np.stack([p[:, 0] * np.cos(a) - p[:, 1] * np.sin(a) + le[0], p[:, 0] * np.sin(a) + p[:, 1] * np.cos(a) + le[1]], axis=1)


def tunnel_shapes():
    """Side-view outlines from configs/ct17_cfd.yaml, in metres (x rearward from the front axle, z up from the ground):
    each wing's elements, the body's centre-line silhouette, the driver's helmet and the wheels."""
    import yaml
    y = yaml.safe_load((IC / "configs" / "ct17_cfd.yaml").read_text()) or {}
    out = {"elements": {}, "le": {}, "names": {}}
    cad = wing_sections()
    for wing in ("front_wing", "rear_wing"):
        if cad:                                                # the real sections, where the CAD puts them
            els = [e for e in cad["elements"] if e["wing"] == wing and e["name"] in DESCRIBED]
            polys = [np.array(e["poly"], float) for e in els]
            out["elements"][wing] = [_j(q, 5) for q in polys]
            out["le"][wing] = [_j(q[np.argmax(np.linalg.norm(q - q[0], axis=1))], 5) for q in polys]
            out["names"][wing] = [e["name"] for e in els]
            continue
        w, polys, les, prev = y.get(wing) or {}, [], [], None
        for k, e in enumerate(w.get("elements") or []):
            if prev is None:
                le = np.array(w["main_le"], float) if "main_le" in w else np.array([float(w.get("main_le_x", 0.0)), 0.0])
            else:
                le = prev[0] + np.array([-float(e.get("overlap", 0.01)), float(e.get("gap", 0.012))])
            prev = _place(e["section"], float(e["chord"]), float(e["angle_deg"]), le)
            polys.append(prev)
            les.append(le)
        if polys and "ground_clearance" in w:                 # the main plane's lowest point sits at the measured clearance
            dz = float(w["ground_clearance"]) - float(polys[0][:, 1].min())
            polys = [q + np.array([0.0, dz]) for q in polys]
            les = [q + np.array([0.0, dz]) for q in les]
        out["elements"][wing] = [_j(q, 5) for q in polys]
        out["le"][wing] = [_j(q, 5) for q in les]
        out["names"][wing] = [f"{'F' if wing == 'front_wing' else 'R'}{k + 1}" for k in range(len(polys))]
    st = np.array((y.get("body") or {}).get("stations") or [], float)
    if len(st):
        out["body"] = _j(np.vstack([st[:, [0, 3]], st[::-1][:, [0, 2]]]), 5)        # over the top, back along the floor
        out["body_half_width"] = _j(st[:, [0, 1]], 5)
    d = y.get("driver") or {}
    if "helmet_centre" in d:
        out["helmet"] = [_num(d["helmet_centre"][0]), _num(d["helmet_centre"][2]), _num(d.get("helmet_radius", 0.135))]
    wh = y.get("wheels") or {}
    cfg = _load("ic")["vehicle"]
    out["wheels"] = {"radius": _num(wh.get("radius", cfg.tires.tire_radius)), "wheelbase": _num(cfg.geometry.wheelbase),
                     "inner_face_y": _num(cfg.geometry.track_front / 2 - float(wh.get("width", 0.19)) / 2)}
    fw = y.get("front_wing") or {}
    out["front_sections"] = fw.get("sections")
    out["rear_half_span"] = _num(float((y.get("rear_wing") or {}).get("span", 0.86)) / 2)
    return out


# --------------------------------------------------------------------------- export: CAD and config files
def _car_and_runner(car, overrides):
    """(objs, vehicle, run(cfg, events) -> event results) for a car with the overrides applied."""
    objs = _load(car, overrides)
    cfg = objs["vehicle"]
    if car == "ic":
        import fsae_pt as pt
        return objs, pt.PTVehicle(cfg, objs["pt"]), (lambda c, events: pt.run_events_pt(c, objs["pt"], events=tuple(events)))
    import fsae_ev as ev
    return objs, ev.EVVehicle(cfg, objs["ev"], objs["pack"]), (lambda c, events: ev.run_events_ev(c, objs["ev"], objs["pack"], events=tuple(events)))


def export_files(car, overrides=None):
    """Text files for the changed car: {file name: text}.

    hardpoints_<axle>_optimumk.csv   the OptimumK export with the edited points (the format the model reads)
    hardpoints_xyz.csv               every suspension point, both sides, in mm (name, side, x, y, z)
    solidworks_equations.txt         the same points as SolidWorks global variables ("FL_UPRI_UppPnt_X"= 12.3mm)
    frame_tubes.csv                  the frame's tubes in the frame CAD's axes (the tube-list format the model reads)
    frame_joints.csv                 the frame's joints in car axes, mm
    inputs_changed.yaml              the changed inputs, grouped by config file, ready to paste into the YAML files
    """
    from fsae_vd import hardpoints as hp
    ov = dict(overrides or {})
    objs = _load(car, ov)
    cfg = objs["vehicle"]
    out = {}
    rows, eqs = ["point,description,side,x_mm,y_mm,z_mm"], ['\' Suspension hard points from the lap-simulation website. Export axes: x forward from the front axle, y to the left, z up.',
                                                             '\' In SolidWorks: Tools > Equations > Import, or link this file. Left side shown positive in y.']
    for which in ("front", "rear"):
        f = getattr(cfg, which).hardpoint_file
        if not f:
            continue
        out[f"hardpoints_{which}_optimumk.csv"] = Path(f).read_text(encoding="utf-8-sig", errors="replace")
        d = hp.load_optimumk_csv(f)
        tag = "F" if which == "front" else "R"
        for name, p in d["points"].items():
            x, y, z = (1e3 * float(v) for v in p)
            for side, sy in (("L", 1.0), ("R", -1.0)):
                rows.append(f"{which}_{name},{HP_NAMES.get(name, name)},{side},{x:.3f},{sy * y:.3f},{z:.3f}")
                for ax, v in (("X", x), ("Y", sy * y), ("Z", z)):
                    eqs.append(f'"{tag}{side}_{name}_{ax}"= {v:.3f}mm')
    out["hardpoints_xyz.csv"] = "\n".join(rows) + "\n"
    out["solidworks_equations.txt"] = "\n".join(eqs) + "\n"
    try:
        from fsae_frame import model
        st = ov.get("geom:frame") or frame_geometry()
        tubes, _ = _frame_build(st)
        R = model.CAD_AXES                                   # car <- frame CAD; its transpose goes back
        lines = ["# frame from the lap-simulation website, part axes (x across, y up, z forward; origin front axle centre on the ground), metres",
                 "x0,y0,z0,x1,y1,z1,od,wall,shape,bend,member"]
        for t in tubes:
            a, b = R.T @ t.p0, R.T @ t.p1
            lines.append(",".join(f"{v:.6f}" for v in (*a, *b, t.od, t.wall)) + f",{t.shape},{int(t.bend)},{t.member}")
        out["frame_tubes.csv"] = "\n".join(lines) + "\n"
        out["frame_joints.csv"] = "joint,x_mm,y_mm,z_mm\n" + "".join(f"{i + 1},{1e3 * p[0]:.2f},{1e3 * p[1]:.2f},{1e3 * p[2]:.2f}\n" for i, p in enumerate(st["nodes"]))
    except Exception as e:  # noqa: BLE001
        out["frame_tubes.csv"] = f"# frame export failed: {e}\n"
    files = {"vehicle": "ct17_baseline.yaml" if car == "ic" else "ct17ev_vehicle.yaml", "pt": "ct17_powertrain.yaml",
             "ev": "ct17ev_powertrain.yaml", "pack": "ev_accumulator.yaml"}
    by = {}
    for k, v in ov.items():
        which, _, path = k.partition(":")
        if which in files:
            by.setdefault(files[which], []).append((path, v))
    y = ["# Inputs changed on the lap-simulation website (values in SI unless a unit is written).",
         "# Paste each value at its dotted path in the named config file."]
    for f, items in by.items():
        y.append(f"\n{f}:")
        for path, v in sorted(items):
            y.append(f"  {path}: {__import__('json').dumps(v)}")
    for g in ("geom:hardpoints", "geom:frame", "geom:tracks", "geom:wings"):
        if g in ov:
            y.append(f"\n# {g.split(':')[1]}: edited on the site (see the exported geometry files)")
    out["inputs_changed.yaml"] = "\n".join(y) + "\n"
    return out


# --------------------------------------------------------------------------- import: CAD
def import_hardpoints(text):
    """Suspension points from an OptimumK export, or from this site's hardpoints_xyz.csv -> {axle: {point: [x, y, z] m}}.
    An OptimumK file holds one axle: it is reported under 'axle' = None and the page asks which one it is."""
    import tempfile
    from fsae_vd import hardpoints as hp
    lines = [ln for ln in str(text).splitlines() if ln.strip()]
    if lines and lines[0].lower().replace(" ", "").startswith("point,description,side"):
        out = {"front": {}, "rear": {}}
        for ln in lines[1:]:
            c = ln.split(",")
            if len(c) >= 6 and c[2].strip().upper() == "L":
                which, _, name = c[0].partition("_")
                if which in out:
                    out[which][name] = [float(c[3]) / 1e3, float(c[4]) / 1e3, float(c[5]) / 1e3]
        return {"format": "xyz", "points": {k: v for k, v in out.items() if v}}
    f = Path(tempfile.gettempdir()) / "lapsim_import_hp.csv"
    f.write_text(str(text))
    d = hp.load_optimumk_csv(f)
    if not d["points"]:
        raise ValueError("no suspension points found: expected an OptimumK export (.csv) or this site's hardpoints_xyz.csv")
    hp.Corner(d, True)                                   # raises if a point the solver needs is missing
    xs = [p[0] for p in d["points"].values()]
    return {"format": "optimumk", "title": d["name"], "axle_guess": "rear" if float(np.mean(xs)) < -0.6 else "front",
            "points": {None: {k: _j(v, 7) for k, v in d["points"].items()}}}


def import_frame(text):
    """A frame tube list (x0,y0,z0,x1,y1,z1,od,wall,shape,bend,member in the frame CAD's axes, metres) -> an
    editable frame state for the frame editor."""
    import tempfile
    from fsae_frame import model, tubes
    f = Path(tempfile.gettempdir()) / "lapsim_import_frame.csv"
    f.write_text(str(text))
    tb = model.to_car(tubes.load_tubes(f))
    if len(tb) < 3:
        raise ValueError("fewer than three tubes found: expected the columns x0,y0,z0,x1,y1,z1,od,wall,shape,bend,member")
    keep = dict(_FRAME)
    try:
        _FRAME.clear()
        _FRAME["tubes"] = tb
        st = __import__("copy").deepcopy(frame_geometry())
    finally:
        _FRAME.clear()
        _FRAME.update(keep)
    return st


# --------------------------------------------------------------------------- import: logged runs, and the comparison
def log_import(car, text, overrides=None):
    """A logged run (RaceStudio / AiM .csv export, or a plain .csv with a time column) cleaned with the model's
    own pipeline and set against the prediction: the car simulated on the course the log was driven on."""
    import tempfile
    import fsae_vd as vd
    from fsae_vd import identification as idn
    from fsae_vd import track as trk
    f = Path(tempfile.gettempdir()) / "lapsim_import_log.csv"
    f.write_text(str(text))
    log = vd.load_log(str(f))
    pl = vd.process_log(log)
    d = pl.df
    out = {"meta": {k: str(v) for k, v in (log.meta or {}).items() if k.lower() in ("session", "vehicle", "racer", "track", "date", "comment", "duration")},
           "channels": sorted(log.channels), "duration_s": _num(float(d["t"].iloc[-1] - d["t"].iloc[0])),
           "laps": [{"lap": i + 1, "start_s": _num(a), "time_s": _num(b - a, 6)} for i, (a, b) in enumerate(pl.laps or [])],
           "rejected_laps": len(pl.rejected_laps or [])}
    if "v" not in d:
        raise ValueError("no speed channel could be built from this log (it needs GPS speed or wheel speeds)")
    laps = pl.laps or [(float(d["t"].iloc[0]), float(d["t"].iloc[-1]))]
    best = min(range(len(laps)), key=lambda i: laps[i][1] - laps[i][0])
    a, b = laps[best]
    L = d[(d["t"] >= a) & (d["t"] < b)]
    idx = _pick(len(L), 1200)
    s0 = float(L["s"].iloc[0]) if "s" in L else 0.0
    meas = {"lap": best + 1, "time_s": _num(b - a, 6), "s": _j((L["s"].to_numpy() - s0)[idx]) if "s" in L else None,
            "t": _j((L["t"].to_numpy() - a)[idx])}
    for c in ("v", "ax", "ay", "x", "y", "throttle", "brake_f", "steer"):
        if c in L:
            meas[c] = _j(L[c].to_numpy(float)[idx], 4)
    out["measured"] = meas
    g = idn.gg_points(pl)
    gi = _pick(len(g), 1500)
    out["gg"] = {"ax": _j(g["ax"].to_numpy()[gi] / 9.80665, 3), "ay": _j(g["ay"].to_numpy()[gi] / 9.80665, 3)}
    objs, veh, _ = _car_and_runner(car, overrides)
    cfg = objs["vehicle"]
    pred = {}
    try:                                                   # the car on the logged course
        tr = trk.from_processed_log(pl)
        flying = float(L["v"].iloc[0]) > 3.0
        tg = veh.traction_ggv()
        if car == "ic":
            import fsae_pt as pt
            r = pt.solve_pt(tr, veh, tg, pt.PTTables(veh), v0=None if flying else 0.0, flying=flying)
        else:
            import fsae_ev as ev
            r = ev.solve_ev(tr, veh, tg, veh.tables0, v0=None if flying else 0.0, flying=flying)
        pi = _pick(len(r.s), 1200)
        pred = {"time_s": _num(r.time, 6), "s": _j(np.asarray(r.s)[pi]), "v": _j(np.asarray(r.v)[pi], 4), "ax": _j(np.asarray(r.ax)[pi], 4),
                "ay": _j(np.asarray(r.ay)[pi], 4), "x": _j(np.asarray(tr.x)[pi]) if tr.x is not None else None,
                "y": _j(np.asarray(tr.y)[pi]) if tr.y is not None else None, "track_length_m": _num(tr.length), "start": "flying" if flying else "standing",
                "limit": [str(v) for v in np.asarray(r.limit)[pi]]}
        pred["delta_s"] = _num(r.time - (b - a), 4)
        pred["delta_pct"] = _num(100 * (r.time - (b - a)) / (b - a), 3)
    except Exception as e:  # noqa: BLE001
        pred = {"error": f"the course could not be rebuilt from this log ({type(e).__name__}: {e}); it needs GPS position and at least one lap"}
    out["predicted"] = pred
    cal = {}
    try:                                                   # grip: scale the tires so the model's lateral limit matches the log's
        env = idn.lateral_envelope(pl)
        if len(env) >= 2 and float(np.median(env["ay_env"])) < 0.6 * 9.80665:
            cal["grip"] = {"skipped": "this run has no hard cornering (lateral acceleration stays under 0.6 g), so it says nothing about grip"}
        elif len(env) >= 2:
            veh.pt.unlimited = True
            try:
                model = np.asarray(veh.ay_max(env["v"].to_numpy()), float)
            finally:
                veh.pt.unlimited = False
            w = np.sqrt(env["n"].to_numpy())
            ratio = float(np.sum(w * env["ay_env"].to_numpy() / model) / np.sum(w))
            cal["grip"] = {"v": _j(env["v"].to_numpy()), "measured_g": _j(env["ay_env"].to_numpy() / 9.80665, 4), "model_g": _j(model / 9.80665, 4),
                           "ratio": _num(ratio, 4), "lambda_mu_now": _num(cfg.tires.lambda_mu), "lambda_mu_suggested": _num(cfg.tires.lambda_mu * ratio, 4),
                           "samples": int(env["n"].sum())}
    except Exception as e:  # noqa: BLE001
        cal["grip"] = {"error": f"{type(e).__name__}: {e}"}
    try:                                                   # drag and rolling resistance from coasting
        mask = idn.coastdown_mask(pl)
        if int(mask.sum()) >= 150:
            c = cfg.copy()
            if c.aero.rho is None:
                c.aero.rho = float(c.aero.density())
            fit = idn.fit_coastdown(pl, c, mask)
            cal["coastdown"] = {k: _num(v, 4) for k, v in fit.items()}
    except Exception as e:  # noqa: BLE001
        cal["coastdown"] = {"error": f"{type(e).__name__}: {e}"}
    out["calibration"] = cal
    qc = vd.qc_table(pl)
    out["qc"] = _table(qc.head(40), index=True) if qc is not None and len(qc) else None
    return out


# --------------------------------------------------------------------------- optimisation (fsae_vd.optimize / adjust)
def _progress(text):
    """Tell the page what a long job is doing (in the browser's worker; a no-op elsewhere)."""
    try:
        import json as _json
        import js
        js.postMessage(js.JSON.parse(_json.dumps({"status": text})))
    except Exception:  # noqa: BLE001
        pass


class _full_car_events:
    """Make fsae_vd.optimize / adjust score cars with the full powertrain model (fsae_pt or fsae_ev) instead of the
    vehicle-dynamics-only runner they import, so their sensitivity, optimiser and trackside tools work on this car."""

    def __init__(self, runner, label=None):
        self.runner, self.label, self.n = runner, label, 0

    def __enter__(self):
        from fsae_vd import adjust, optimize
        self.mods, self.old = (optimize, adjust), (optimize.run_events, adjust.run_events)

        def run_events(cfg, tracks=None, ggv=None, events=EVENTS, **kw):
            self.n += 1
            if self.label and self.n % 4 == 0:
                _progress(f"{self.label}: {self.n} simulations so far…")
            return self.runner(cfg, events)
        for m in self.mods:
            m.run_events = run_events
        return self

    def __exit__(self, *a):
        for m, old in zip(self.mods, self.old):
            m.run_events = old


def _params(cfg, keys, span=0.2):
    from fsae_vd.optimize import Param
    out = []
    for k in keys:
        path = k.split(":", 1)[1] if ":" in k else k
        v = cfg.get(path)
        if not isinstance(v, (int, float)) or isinstance(v, bool):
            raise ValueError(f"{path} is not a number in the car config, so it cannot be varied")
        lo, hi = sorted((v * (1 - span), v * (1 + span))) if v else (-span, span)
        out.append(Param(path, bounds=(lo, hi), label=path))
    return out


def sensitivity(car, keys, overrides=None, events=("acceleration", "skidpad", "autocross")):
    """fsae_vd.optimize.sensitivity on the full car: what a +/-5 % step in each input does to every event and to the
    score. Vehicle inputs only ('vehicle:<path>')."""
    from fsae_vd import optimize
    objs, veh, runner = _car_and_runner(car, overrides)
    cfg = objs["vehicle"]
    P = _params(cfg, keys)
    with _full_car_events(runner, "Sensitivity") as fc:
        base, tab = optimize.sensitivity(cfg, P, events=tuple(events))
    return {"base": {"score": _num(base["score"]), "score_kind": base["score_kind"], "times": _j(base["times"])},
            "table": _table(tab, index=False), "simulations": fc.n, "events": list(events)}


def optimise(car, keys, overrides=None, events=("acceleration", "skidpad", "autocross"), span=0.15, maxiter=4, popsize=4, robust=True):
    """fsae_vd.optimize.optimise (differential evolution) + robustness + recommend on the full car."""
    from fsae_vd import optimize
    objs, veh, runner = _car_and_runner(car, overrides)
    cfg = objs["vehicle"]
    P = _params(cfg, keys, float(span))
    with _full_car_events(runner, "Optimising") as fc:
        opt = optimize.optimise(cfg, P, events=tuple(events), maxiter=int(maxiter), popsize=int(popsize))
        rob = optimize.robustness(cfg, opt["best_cfg"], events=tuple(events)) if robust else None
        table, ev, verdict = optimize.recommend(opt, P, rob)
    hist = [h[1] - h[2] for h in opt["history"]]
    best_so_far = list(np.maximum.accumulate(hist)) if hist else []
    return {"parameters": _table(table, index=False), "events": _table(ev.rename_axis("event")), "verdict": [str(v) for v in verdict],
            "robustness": _table(rob, index=False) if rob is not None else None,
            "best": {f"vehicle:{p.path}": _num(float(x), 6) for p, x in zip(P, opt["x"])},
            "score": {"base": _num(opt["base"]["score"]), "best": _num(opt["best"]["score"]), "kind": opt["base"]["score_kind"]},
            "history": _j(np.array(best_so_far), 5), "simulations": fc.n, "reverted": list(opt.get("reverted_no_effect", []))}


def trackside(car, overrides=None, events=("acceleration", "skidpad", "autocross")):
    """fsae_vd.adjust.optimise_all_events on the full car: the best reachable setting of the adjustments the car
    has (springs, anti-roll bar blades, camber shims ...) for each event."""
    from fsae_vd import adjust
    objs, veh, runner = _car_and_runner(car, overrides)
    cfg = objs["vehicle"]
    with _full_car_events(runner, "Trackside adjustment") as fc:
        r = adjust.optimise_all_events(cfg, events=tuple(events), max_exhaustive=80, restarts=1)
    return {"summary": _table(r["summary"]), "settings": _table(r["settings_by_event"]) if len(r["settings_by_event"]) else None,
            "not_optimised": _table(r["not_optimised"], index=False), "features": [f.name for f in r["features"]], "simulations": fc.n}
