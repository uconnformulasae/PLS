"""Trackside-adjustment optimisation: the best reachable setting of the car's adjustments for each event.

Builds the set of settings the car can physically reach (from
``cfg.adjustability``), searches it separately for each Michigan event, and
reports the best adaptation with its predicted time and points.

Search: exhaustive when the discrete space is small, otherwise coordinate
descent (try every value of one adjustment with the others fixed, cycle until
nothing improves) with random restarts, which suits a space of a few discrete
knobs and never returns a setup worse than the baseline.
"""
from __future__ import annotations

import dataclasses as dc
import itertools

import numpy as np
import pandas as pd

from lapsim.config.schema import CarConfig, MissingDataError
from lapsim.config.units import to_unit
from lapsim.studies.optimize import evaluate

EVENTS = ("acceleration", "skidpad", "autocross", "endurance")


@dc.dataclass
class Feature:
    name: str
    path: str
    values: list
    unit: str = ""

    def show(self, v):
        if isinstance(v, str) or v is None:
            return v
        return round(to_unit(v, self.unit), 4) if self.unit else v


def build_space(cfg: CarConfig):
    """Features that can be optimised now, and the ones left out (with the reason)."""
    a = cfg.adjustability
    feats, skipped = [], []
    if a.spring_rates:
        feats += [Feature("front spring", "front.spring_rate", list(a.spring_rates), "lbf/in"),
                  Feature("rear spring", "rear.spring_rate", list(a.spring_rates), "lbf/in")]
    else:
        skipped.append(("springs", "adjustability.spring_rates empty"))
    if a.arb_blades:
        blades = list(a.arb_blades) + ([0.0] if a.arb_can_disconnect else [])
        for nm, ax in (("front", cfg.front), ("rear", cfg.rear)):
            if ax.hardpoint_file or ax.arb_motion_ratio is not None:
                feats.append(Feature(f"{nm} ARB blade", f"{nm}.arb_blade_length", blades, "in"))
            else:
                skipped.append((f"{nm} ARB", "needs hardpoints or arb_motion_ratio"))
    if a.camber_shim_range and a.shim_thicknesses:
        steps = _shim_totals(a.shim_thicknesses, *a.camber_shim_range)
        feats += [Feature("front camber shims", "adjust.camber_shim_front", steps, "in"),
                  Feature("rear camber shims", "adjust.camber_shim_rear", steps, "in")]
    else:
        skipped.append(("camber shims", "adjustability.camber_shim_range not recorded"))
    if a.tie_rod_range and a.tie_rod_step:
        vals = list(np.arange(a.tie_rod_range[0], a.tie_rod_range[1] + 1e-9, a.tie_rod_step))
        feats += [Feature("front toe (tie-rod length)", "adjust.tie_rod_front", vals, "mm"),
                  Feature("rear toe (toe-link length)", "adjust.tie_rod_rear", vals, "mm")]
    else:
        skipped.append(("toe", "adjustability.tie_rod_range / tie_rod_step not recorded"))
    if a.pull_rod_range and a.pull_rod_step:
        vals = list(np.arange(a.pull_rod_range[0], a.pull_rod_range[1] + 1e-9, a.pull_rod_step))
        feats += [Feature("front ride height (pull-rod length)", "adjust.pull_rod_front", vals, "mm"),
                  Feature("rear ride height (pull-rod length)", "adjust.pull_rod_rear", vals, "mm")]
    else:
        skipped.append(("ride height", "adjustability.pull_rod_range / pull_rod_step not recorded"))
    ws = [w for w in a.wing_settings if all(cfg.aero.settings.get(w, {}).get(k) is not None
                                            for k in ("ClA_front", "ClA_rear", "CdA"))]
    if ws:
        feats.append(Feature("wing setting", "adjust.wing_setting", ws))
    else:
        skipped.append(("wing angle", "aero.settings has no ClA/CdA values per setting"))
    if a.pressure_range and a.pressure_step:
        skipped.append(("tire pressure", "the tire model has no pressure sensitivity"))
    else:
        skipped.append(("tire pressure", "adjustability.pressure_range / pressure_step not recorded"))
    skipped.append(("dampers", "quasi-steady model: damping does not change steady-state lap time"))
    return feats, skipped


def _shim_totals(thicknesses, lo, hi, n_max=8):
    tot = set()
    for counts in itertools.product(range(-n_max, n_max + 1), repeat=len(thicknesses)):
        t = sum(c * th for c, th in zip(counts, thicknesses))
        if lo - 1e-9 <= t <= hi + 1e-9:
            tot.add(round(t, 7))
    return sorted(tot)


def _apply(cfg, feats, idx):
    c = cfg.copy()
    for f, i in zip(feats, idx):
        c.set(f.path, f.values[i])
    return c


def _event_time(cfg, event, runner, min_clearance):
    ev = evaluate(cfg, runner, (event,))
    t = ev["times"].get(event, np.inf)
    pen = 0.0
    for k in ("clearance_front", "clearance_rear"):
        if np.isfinite(ev[k]) and ev[k] < min_clearance:
            pen += 1e3 * (min_clearance - ev[k])
    return t * (1 + pen), ev


def optimise_event(cfg: CarConfig, event: str, feats=None, runner=None, max_exhaustive=600, restarts=4,
                   seed=0, min_clearance=0.005):
    """Best reachable setup for one event. Returns dict with baseline/best time,
    settings table and the EventResults of the best setup."""
    if feats is None:
        feats, _ = build_space(cfg)
    if not feats:
        raise MissingDataError("no adjustable features with recorded ranges")
    # baseline index: nearest available value to the current setting
    base_idx = []
    for f in feats:
        cur = cfg.get(f.path)
        if isinstance(f.values[0], str):
            base_idx.append(f.values.index(cur) if cur in f.values else 0)
        else:
            cur = cur if cur is not None else f.values[len(f.values) // 2]
            base_idx.append(int(np.argmin([abs(v - cur) for v in f.values])))
    cache = {}

    def J(idx):
        idx = tuple(idx)
        if idx not in cache:
            try:
                cache[idx] = _event_time(_apply(cfg, feats, idx), event, runner, min_clearance)[0]
            except MissingDataError:
                raise
            except Exception:
                cache[idx] = np.inf
        return cache[idx]

    base_t = J(base_idx)
    sizes = [len(f.values) for f in feats]
    if np.prod(sizes) <= max_exhaustive:
        best = min(itertools.product(*[range(n) for n in sizes]), key=J)
    else:
        rng = np.random.default_rng(seed)
        starts = [list(base_idx)] + [[int(rng.integers(n)) for n in sizes] for _ in range(restarts)]
        best = list(base_idx)
        for s in starts:
            cur = list(s)
            improved = True
            while improved:
                improved = False
                for i, n in enumerate(sizes):
                    cand = min(range(n), key=lambda v: J(cur[:i] + [v] + cur[i + 1:]))
                    if J(cur[:i] + [cand] + cur[i + 1:]) < J(cur) - 1e-9:
                        cur[i] = cand
                        improved = True
            if J(cur) < J(best):
                best = cur
    best = list(best)
    # parsimony: keep baseline value where changing it gains < 1 ms
    for i in range(len(best)):
        trial = best[:i] + [base_idx[i]] + best[i + 1:]
        if J(trial) <= J(best) + 1e-3:
            best = trial
    best_cfg = _apply(cfg, feats, best)
    res = runner(best_cfg, (event,))
    base_res = runner(_apply(cfg, feats, base_idx), (event,))
    table = pd.DataFrame([{"adjustment": f.name, "baseline": f.show(f.values[b]), "best": f.show(f.values[k]),
                           "unit": f.unit, "changed": b != k}
                          for f, b, k in zip(feats, base_idx, best)])
    return {"event": event, "baseline_time": base_res.times[event], "best_time": res.times[event],
            "baseline_points": base_res.points.get(event), "best_points": res.points.get(event),
            "settings": table, "best_cfg": best_cfg, "result": res, "baseline_result": base_res,
            "evaluations": len(cache)}


def optimise_all_events(cfg: CarConfig, runner=None, events=EVENTS, **kw):
    feats, skipped = build_space(cfg)
    out, rows = {}, []
    for e in events:
        if e in ("autocross", "endurance") and not getattr(cfg.events, f"{e}_track"):
            rows.append({"event": e, "status": "no track supplied"})
            continue
        r = optimise_event(cfg, e, feats, runner, **kw)
        out[e] = r
        rows.append({"event": e, "baseline_s": r["baseline_time"], "adapted_s": r["best_time"],
                     "gain_s": r["baseline_time"] - r["best_time"],
                     "baseline_pts": r["baseline_points"], "adapted_pts": r["best_points"],
                     "changes": "; ".join(f"{a}: {b} -> {c} {u}" for a, b, c, u, ch in
                                          r["settings"].itertuples(index=False) if ch) or "keep baseline"})
    summary = pd.DataFrame(rows).set_index("event")
    settings = pd.concat({e: r["settings"].set_index("adjustment")["best"] for e, r in out.items()}, axis=1) \
        if out else pd.DataFrame()
    return {"summary": summary, "settings_by_event": settings, "per_event": out,
            "features": feats, "not_optimised": pd.DataFrame(skipped, columns=["feature", "reason"])}
