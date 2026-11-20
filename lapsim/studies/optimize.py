"""Sensitivity analysis, setup optimisation and recommendations.

Every function takes ``runner(cfg, events) -> EventResults``: how a configuration is simulated (the car's
own QSS solver; see ``runner_for(car)`` in lapsim/studies/__init__.py)."""
from __future__ import annotations

import dataclasses as dc

import numpy as np
import pandas as pd
from scipy import optimize as so

from lapsim.config.schema import CarConfig
from lapsim.core.events import SCORING

EVENT_MAX = {e: v[0] + v[1] for e, v in SCORING.items()}


@dc.dataclass
class Param:
    path: str                       # config path, e.g. "front.spring_rate"
    bounds: tuple | None = None     # continuous (lo, hi) in SI
    choices: list | None = None     # discrete settings (e.g. ARB blades, shim steps)
    label: str = ""
    step: float | None = None       # sensitivity step (default 5% of value)

    def name(self):
        return self.label or self.path


ALL_EVENTS = ("acceleration", "skidpad", "autocross", "endurance")


def evaluate(cfg: CarConfig, runner=None, events=None, baseline_times: dict | None = None):
    """Objective and constraint metrics for one configuration."""
    events = events or ALL_EVENTS
    r = runner(cfg, events)
    times = r.times
    if len(r.points) == len(times) and times:
        score = sum(r.points.values())
        kind = "points"
    else:   # no Tmin: weight relative time change by each event's maximum points
        base = baseline_times or times
        score = -sum(EVENT_MAX[e] * (times[e] / base[e] - 1) for e in times)
        kind = "weighted relative time (higher is better)"
    # constraint metrics over every simulated lap
    clear_f = clear_r = np.inf
    lift = False
    for k, lap in r.laps.items():
        if not hasattr(lap, "state"):
            continue
        st = lap.state
        if cfg.front.ride_height is not None:
            clear_f = min(clear_f, cfg.front.ride_height - float(np.max(st["heave_front"])))
        if cfg.rear.ride_height is not None:
            clear_r = min(clear_r, cfg.rear.ride_height - float(np.max(st["heave_rear"])))
        lift |= bool(np.any(st["wheel_lift"]))
    return {"score": score, "score_kind": kind, "times": times, "points": r.points,
            "clearance_front": clear_f, "clearance_rear": clear_r, "wheel_lift": lift, "result": r}


def _apply(cfg, params, x):
    c = cfg.copy()
    for p, xi in zip(params, x):
        c.set(p.path, p.choices[int(round(xi))] if p.choices is not None else float(xi))
    return c


def sensitivity(cfg: CarConfig, params: list[Param], runner=None, events=None):
    """Central finite differences of each event time and the score."""
    events = events or ("acceleration", "skidpad", "autocross", "endurance")
    base = evaluate(cfg, runner, events)
    rows = []
    for p in params:
        x0 = cfg.get(p.path)
        if p.choices is not None:
            vals = p.choices
            res = [evaluate(cfg.copy().set(p.path, v), runner, events, base["times"]) for v in vals]
            for v, rr in zip(vals, res):
                rows.append({"param": p.name(), "value": v, "d_score": rr["score"] - base["score"],
                             **{f"d_t_{e}": rr["times"][e] - base["times"][e] for e in base["times"]}})
            continue
        h = p.step or 0.05 * abs(x0 if x0 else 1.0)
        up = evaluate(cfg.copy().set(p.path, x0 + h), runner, events, base["times"])
        dn = evaluate(cfg.copy().set(p.path, x0 - h), runner, events, base["times"])
        rows.append({"param": p.name(), "value": x0, "step": h,
                     "d_score_per_step": (up["score"] - dn["score"]) / 2,
                     **{f"dt_{e}_per_step": (up["times"][e] - dn["times"][e]) / 2 for e in base["times"]}})
    return base, pd.DataFrame(rows)


def optimise(cfg: CarConfig, params: list[Param], runner=None, events=None, maxiter=25, popsize=8,
             seed=0, min_clearance=0.005, penalty=50.0, workers=1, freq_bounds=(None, None),
             penalise_wheel_lift=False):
    """Global search (differential evolution) over the chosen parameters.

    Discrete parameters use integer indices (scipy 'integrality'). Setups that
    bottom out (clearance < min_clearance) or lift a wheel are penalised."""
    events = events or ("acceleration", "skidpad", "autocross", "endurance")
    base = evaluate(cfg, runner, events)
    bounds, integ = [], []
    for p in params:
        if p.choices is not None:
            bounds.append((0, len(p.choices) - 1)); integ.append(True)
        else:
            bounds.append(p.bounds); integ.append(False)
    history = []

    def f(x):
        try:
            ev = evaluate(_apply(cfg, params, x), runner, events, base["times"])
        except Exception:
            return 1e6
        pen = 0.0
        for k in ("clearance_front", "clearance_rear"):
            if np.isfinite(ev[k]) and ev[k] < min_clearance:
                pen += penalty * (min_clearance - ev[k]) / min_clearance
        pen += penalty if (penalise_wheel_lift and ev["wheel_lift"]) else 0.0
        veh = ev["result"].vehicle
        for R in (veh.Rf, veh.Rr):          # optional ride-frequency window (Hz)
            lo, hi = freq_bounds
            if lo is not None and R.ride_frequency < lo:
                pen += penalty * (lo - R.ride_frequency) / lo
            if hi is not None and R.ride_frequency > hi:
                pen += penalty * (R.ride_frequency - hi) / hi
        history.append((list(x), ev["score"], pen))
        return -(ev["score"] - pen)

    x0 = []
    for p in params:
        v = cfg.get(p.path)
        x0.append(p.choices.index(v) if p.choices is not None and v in p.choices else
                  (np.clip(v, *p.bounds) if v is not None and p.bounds else np.mean(bounds[len(x0)])))
    r = so.differential_evolution(f, bounds, integrality=integ, maxiter=maxiter, popsize=popsize,
                                  seed=seed, x0=x0, polish=False, tol=1e-4, workers=workers,
                                  updating="deferred" if workers != 1 else "immediate")
    best_cfg = _apply(cfg, params, r.x)
    best = evaluate(best_cfg, runner, events, base["times"])
    if best["score"] < base["score"]:          # never recommend something worse than baseline
        x0i = np.array(x0, float)
        best_cfg, best, r.x = cfg.copy(), base, x0i
    # parsimony: put back any parameter whose change does not help (only change what matters)
    xb = np.array(r.x, float)
    reverted = []
    for i, p in enumerate(params):
        if np.isclose(xb[i], x0[i]):
            continue
        trial = xb.copy(); trial[i] = x0[i]
        ev = evaluate(_apply(cfg, params, trial), runner, events, base["times"])
        if ev["score"] >= best["score"] - 1e-3:
            xb, best = trial, ev
            reverted.append(p.name())
    best_cfg = _apply(cfg, params, xb)
    r.x = xb
    return {"base": base, "best": best, "best_cfg": best_cfg, "x": r.x, "history": history, "result": r,
            "reverted_no_effect": reverted}


def robustness(cfg_base, cfg_best, runner=None, events=None,
               perturbations=(("tires.lambda_mu", 0.95), ("tires.lambda_mu", 1.05),
                              ("aero.rho", 0.93), ("mass.driver_mass", 1.1))):
    """Does the optimum still beat the baseline when uncertain inputs move?"""
    rows = []
    for path, factor in perturbations:
        cb, co = cfg_base.copy(), cfg_best.copy()
        cb.set(path, cb.get(path) * factor); co.set(path, co.get(path) * factor)
        eb = evaluate(cb, runner, events)
        eo = evaluate(co, runner, events, eb["times"])
        rows.append({"perturbation": f"{path} x{factor}", "base_score": eb["score"],
                     "opt_score": eo["score"] if eo["score_kind"] == eb["score_kind"] else np.nan,
                     "opt_better": eo["score"] > eb["score"]})
    return pd.DataFrame(rows)


def recommend(opt: dict, params: list[Param], robust: pd.DataFrame | None = None, units: dict | None = None):
    """Recommendation table: baseline vs optimum for every parameter, event deltas, and a verdict."""
    from lapsim.config.units import to_unit
    units = units or {}
    base_cfg = opt["best"]["result"].vehicle.cfg
    rows = []
    for p, xi in zip(params, opt["x"]):
        b = opt["base"]["result"].vehicle.cfg.get(p.path)
        n = p.choices[int(round(xi))] if p.choices is not None else float(xi)
        u = units.get(p.path) or ("deg" if p.path.endswith("_deg") else None)
        rows.append({"parameter": p.name(), "baseline": to_unit(b, u) if u and b is not None else b,
                     "recommended": to_unit(n, u) if u else n, "unit": u or "SI / -"})
    table = pd.DataFrame(rows)
    ev = pd.DataFrame({"baseline_s": opt["base"]["times"], "optimised_s": opt["best"]["times"]})
    ev["delta_s"] = ev["optimised_s"] - ev["baseline_s"]
    gain = opt["best"]["score"] - opt["base"]["score"]
    verdict = []
    if opt.get("reverted_no_effect"):
        verdict.append("Kept at baseline (no measurable effect): " + ", ".join(opt["reverted_no_effect"]) + ".")
    verdict.append(f"Objective ({opt['base']['score_kind']}): {opt['base']['score']:.2f} -> "
                   f"{opt['best']['score']:.2f} ({gain:+.2f}).")
    if robust is not None and len(robust):
        frac = robust["opt_better"].mean()
        verdict.append(f"The optimum beats the baseline in {frac:.0%} of the robustness cases.")
        verdict.append("RECOMMEND" if frac >= 0.75 and gain > 0 else
                       "DO NOT ADOPT without testing: the gain is not robust to input uncertainty.")
    return table, ev, verdict
