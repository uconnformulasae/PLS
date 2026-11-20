"""Event geometry: the acceleration straight, the skidpad circle, and tracks from files or logged runs.

A ``Track`` is a centre line: distance ``s`` [m], curvature ``kappa`` [1/m, + left] and, when known,
the plan view ``x``, ``y`` [m]. Track files (tracks/<set>/*.csv) have the columns ``s_m, x_m, y_m,
curvature_1pm`` (see tracks/README.md); ``x, y`` points alone (or ``lat, lon``) are smoothed into a
centre line first. A logged run (a RaceStudio .csv export) becomes the median line of its laps.
"""
from __future__ import annotations

import dataclasses as dc
from pathlib import Path

import numpy as np
import pandas as pd


@dc.dataclass
class Track:
    name: str
    s: np.ndarray          # m
    kappa: np.ndarray      # 1/m (sign: + left)
    closed: bool = False
    x: np.ndarray | None = None
    y: np.ndarray | None = None

    @property
    def length(self):
        return float(self.s[-1])


# =========================================================================== simple shapes
def straight(length: float, ds: float = 0.5, name="acceleration") -> Track:
    s = np.arange(0, length + ds / 2, ds)
    return Track(name, s, np.zeros_like(s), False, s.copy(), np.zeros_like(s))


def circle(radius: float, ds: float = 0.25, name="skidpad") -> Track:
    L = 2 * np.pi * radius
    s = np.arange(0, L + ds / 2, ds)
    th = s / radius
    return Track(name, s, np.full_like(s, 1 / radius), True, radius * np.sin(th), radius * (1 - np.cos(th)))


# =========================================================================== from points
def from_xy(x, y, ds=0.5, pos_sigma=0.05, name="track", closed=None) -> Track:
    """Track from centre-line points. ``pos_sigma`` is the position uncertainty of the points
    (0.05 m for a survey / CAD line, ~0.5 m for GPS). ``closed``: None = closed when the ends are
    within 5 m of each other."""
    from lapsim.validation.logs import smooth_path_curvature
    x, y = np.asarray(x, float), np.asarray(y, float)
    seg = np.hypot(np.diff(x), np.diff(y))
    keep = np.r_[True, seg > 1e-6]
    x, y = x[keep], y[keep]
    s_raw = np.r_[0, np.cumsum(np.hypot(np.diff(x), np.diff(y)))]
    closed = bool(np.hypot(x[0] - x[-1], y[0] - y[-1]) < 5.0) if closed is None else bool(closed)
    pad = min(20.0, 0.25 * s_raw[-1]) if closed and x.size > 8 else 0.0
    if pad > 0:      # closed: smooth with the lap wrapped round both ends, so the curvature has no end effects
        lap = s_raw[-1] + float(np.hypot(x[0] - x[-1], y[0] - y[-1]))
        a, b = s_raw > s_raw[-1] - pad, s_raw < pad
        xs, ys, k = smooth_path_curvature(np.r_[s_raw[a] - lap, s_raw, s_raw[b] + lap], np.r_[x[a], x, x[b]],
                                          np.r_[y[a], y, y[b]], pos_sigma)
        m = slice(int(a.sum()), int(a.sum()) + x.size)
        xs, ys, k = xs[m], ys[m], k[m]
    else:
        xs, ys, k = smooth_path_curvature(s_raw, x, y, pos_sigma)
    s = np.arange(0, s_raw[-1], ds)
    return Track(name, s, np.interp(s, s_raw, k), closed, np.interp(s, s_raw, xs), np.interp(s, s_raw, ys))


# =========================================================================== files
_UNITS = ("_m", "_1pm", "_1/m", "_deg", "_rad")
_ALIASES = {"curvature": "kappa"}


def _columns(d):
    """{canonical name: column} - accepts unit-suffixed names (s_m, x_m, curvature_1pm)."""
    c = {}
    for col in d.columns:
        key = str(col).lower().strip().lstrip("#").strip()
        for suf in _UNITS:
            if key.endswith(suf) and len(key) > len(suf):
                key = key[: -len(suf)]
                break
        c.setdefault(_ALIASES.get(key, key), col)
    return c


def from_csv(path, ds=0.5, pos_sigma=0.05, name=None, closed=None) -> Track:
    """Track .csv: columns s, kappa (curvature) [m, 1/m] with optional x, y; or x, y [m]; or lat, lon [deg]."""
    d = pd.read_csv(path)
    c = _columns(d)
    get = lambda k: d[c[k]].to_numpy(float)
    name = name or Path(path).stem
    if "s" in c and "kappa" in c:
        s0, k0 = get("s"), get("kappa")
        s = np.arange(0, s0[-1], ds)
        x = np.interp(s, s0, get("x")) if "x" in c else None
        y = np.interp(s, s0, get("y")) if "y" in c else None
        if closed is None:
            closed = bool(x is not None and np.hypot(x[0] - x[-1], y[0] - y[-1]) < 5.0)
        return Track(name, s, np.interp(s, s0, k0), closed, x, y)
    if "x" in c and "y" in c:
        return from_xy(get("x"), get("y"), ds, pos_sigma, name, closed)
    if "lat" in c and "lon" in c:
        from lapsim.validation.logs import latlon_to_xy
        x, y = latlon_to_xy(get("lat"), get("lon"))
        return from_xy(x, y, ds, max(pos_sigma, 0.3), name, closed)
    raise ValueError(f"{path}: need columns s,kappa or x,y or lat,lon")


def from_processed_log(plog, ds=0.5, name="logged track") -> Track:
    """Track from the lap ensemble (median curvature across all good laps)."""
    from lapsim.validation.logs import distance_ensemble
    if not plog.laps:
        raise ValueError("no laps found in the log - set gate_latlon or check the beacon channel")
    ens = distance_ensemble(plog.df, plog.laps, ["kappa", "x", "y", "v"])
    s = np.arange(0, ens["s"].iloc[-1], ds)
    k = np.interp(s, ens["s"], ens["kappa_med"])
    x = np.interp(s, ens["s"], ens["x_med"]) if "x_med" in ens else None
    y = np.interp(s, ens["s"], ens["y_med"]) if "y_med" in ens else None
    return Track(name, s, k, True, x, y)


def load_track(path, ds=0.5) -> Track:
    """A track .csv, or a logged run (a RaceStudio .csv export with a Time column)."""
    p = Path(path)
    if not p.exists():
        raise FileNotFoundError(f"track file not found: {p}")
    head = p.read_text(errors="replace")[:4000].lower()
    if "time" in head.split("\n")[0] or '"time"' in head:
        from lapsim.validation.logs import load_log, process_log
        return from_processed_log(process_log(load_log(p)), ds, p.stem)
    return from_csv(p, ds)
