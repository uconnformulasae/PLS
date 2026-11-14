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
    for k, v in (overrides or {}).items():
        _set(objs, k, v)
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
    axle, car = reports.design_report(veh)
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


def _frame():
    from fsae_frame import model, tubes
    f = IC / "data" / "frame" / "ct17_spaceframe_tubes.csv"
    tb = model.to_car(tubes.load_tubes(f))
    seg = [[_num(v, 4) for v in (*t.p0, *t.p1)] + [_num(t.od * 1e3, 4), _num(t.wall * 1e3, 3)] for t in tb]
    s = tubes.summary(tb)
    return {"tubes": seg, "summary": _j(s) if not isinstance(s, pd.DataFrame) else _table(s)}


def _ic_powertrain(res):
    veh, T = res.vehicle, res.tables
    rpm = np.arange(3000, veh.ptc.engine.rev_limit + 1, 250.0) if hasattr(veh.ptc.engine, "rev_limit") else np.arange(3000, 13001, 250.0)
    w = veh.engine.wot(rpm)
    idx = _pick(len(T.v), 300)
    return {"engine": {k: _j(np.asarray(w[k], float)) for k in ("rpm", "torque", "power", "map", "bsfc_g_kwh", "ve") if k in w},
            "engine_summary": _j(veh.engine.summary()),
            "gears": _j(veh.dt.summary(veh.m)),
            "traction": {"v": _j(T.v[idx]), "force_by_gear": [_j(f[idx]) for f in T.F], "resistance": _j(T.R[idx]),
                         "best_gear": _j(T.best[idx]), "v_shift": _j(T.v_shift), "shift_rpm": _j(T.shift_rpm),
                         "v_top": _num(T.v_top)}}


def _ev_powertrain(res):
    import fsae_ev as ev
    veh, T = res.vehicle, res.tables
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
        _safe(sub, "frame", _frame)
        _safe(sub, "powertrain", lambda: _ic_powertrain(res) if car == "ic" else _ev_powertrain(res))
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
