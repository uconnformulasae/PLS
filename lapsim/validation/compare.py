"""Compare two runs: event times, and every channel along each lap.

Each run is a folder written by ``core.results.write_run`` (or the dict of laps of an ``EventResults``).

    d = compare("results/before", "results/after")
    d["events"]                       # time and points of each event, both runs and the difference
    d["channels"]["autocross"]        # largest difference of each channel along the autocross lap
"""
from __future__ import annotations

from pathlib import Path

import numpy as np
import pandas as pd


def _laps(run) -> dict:
    if isinstance(run, (str, Path)):
        return {f.stem: pd.read_csv(f) for f in sorted((Path(run) / "laps").glob("*.csv"))}
    return {k: lr.frame() for k, lr in run.laps.items() if hasattr(lr, "frame")}


def _events(run) -> pd.DataFrame:
    if isinstance(run, (str, Path)):
        return pd.read_csv(Path(run) / "events.csv", index_col="event")
    return run.table()


def channel_differences(a: pd.DataFrame, b: pd.DataFrame) -> pd.DataFrame:
    """Largest absolute and relative difference of every numeric channel the two laps share
    (b interpolated onto a's distance grid when the grids differ)."""
    rows = []
    s_a = a["s"].to_numpy(float)
    same_grid = len(a) == len(b) and np.allclose(s_a, b["s"].to_numpy(float))
    for c in a.columns:
        if c not in b.columns or c == "s" or a[c].dtype.kind not in "fiub" or b[c].dtype.kind not in "fiub":
            continue
        xa = a[c].to_numpy(float)
        xb = b[c].to_numpy(float) if same_grid else np.interp(s_a, b["s"].to_numpy(float), b[c].to_numpy(float))
        ok = np.isfinite(xa) & np.isfinite(xb)
        if not ok.any():
            continue
        d = np.abs(xa[ok] - xb[ok])
        scale = max(float(np.max(np.abs(xa[ok]))), 1e-12)
        rows.append({"channel": c, "max_abs": float(d.max()), "max_rel": float(d.max() / scale)})
    return pd.DataFrame(rows).set_index("channel").sort_values("max_rel", ascending=False) if rows else pd.DataFrame()


def compare(run_a, run_b) -> dict:
    ea, eb = _events(run_a), _events(run_b)
    ev = ea.join(eb, lsuffix="_a", rsuffix="_b", how="outer")
    ev["time_diff_s"] = ev["time_s_b"] - ev["time_s_a"]
    la, lb = _laps(run_a), _laps(run_b)
    ch = {k: channel_differences(la[k], lb[k]) for k in la if k in lb}
    return {"events": ev, "channels": ch}


def max_relative_difference(run_a, run_b) -> float:
    """One number: the largest relative difference over every event time and every lap channel."""
    d = compare(run_a, run_b)
    t = d["events"]
    worst = float(np.nanmax(np.abs(t["time_diff_s"] / t["time_s_a"]))) if len(t) else 0.0
    for df in d["channels"].values():
        if len(df):
            worst = max(worst, float(df["max_rel"].max()))
    return worst
