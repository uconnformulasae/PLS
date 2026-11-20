"""Quasi-steady-state (QSS) lap solver: the parts shared by the IC and the EV car.

At every point of the track the car is assumed to be in a steady state on its performance envelope:

    1. lateral speed limit   the highest speed whose v^2 kappa fits inside the tire + differential
                             envelope (``Vehicle.traction_ggv``);
    2. backward pass         from the end of the lap, the fastest speed the car can still brake down from;
    3. forward pass          a = min(a_tire(v, ay), a_power(v)) from the start, capped by 1. and 2.
                             (the power module's solver: ``qss_ic.py`` gear by gear, ``qss_ev.py`` for the
                             accumulator state);
    4. every point is flagged with what limits it (braking, cornering, traction, power, ...).

Flying laps iterate the start speed until it equals the end speed. Time follows from the speed
profile on the distance grid; the vehicle state (loads, roll, pitch, camber, damper travel) is
evaluated along the solved profile.
"""
from __future__ import annotations

import numpy as np

from lapsim.core.track import load_track


def lateral_speed_limit(kappa, ggv, v_top):
    """Highest speed per station with v^2 |kappa| inside the lateral limit (searched on a 600-point grid)."""
    vv = np.linspace(0.5, v_top, 600)
    A = ggv.ay_lim(vv)
    ak = np.abs(kappa)[:, None]
    ok = vv[None, :] ** 2 * ak <= A[None, :]
    idx = np.where(ok.any(1), ok.shape[1] - 1 - np.argmax(ok[:, ::-1], axis=1), 0)
    return vv[idx]


def backward(s, k, vlim, tggv, v_end):
    """Braking pass from the end of the lap: the fastest speed from which the car can still slow down."""
    n = s.size
    ds = np.diff(s)
    v = np.empty(n)
    v[-1] = min(v_end, vlim[-1])
    for i in range(n - 1, 0, -1):
        d = float(tggv.brk(v[i], v[i] ** 2 * k[i]))
        v[i - 1] = min(np.sqrt(max(v[i] ** 2 + 2 * d * ds[i - 1], 0.01)), vlim[i - 1])
    return v


def skidpad_time(veh, tggv, radius):
    """Steady circle: the highest speed with v^2/R inside the lateral limit (robust to diff
    types whose limit is not monotonic in speed). Returns (time per lap, speed)."""
    v = np.linspace(float(tggv.v[0]), float(tggv.v[-1]), 2000)
    ok = tggv.ay_lim(v) >= v ** 2 / radius
    if not ok.any():
        return np.nan, np.nan
    i = np.flatnonzero(ok)[-1]
    if i + 1 < v.size:
        from scipy.optimize import brentq
        f = lambda x: float(tggv.ay_lim(x)) - x ** 2 / radius
        vs = brentq(f, v[i], v[i + 1])
    else:
        vs = v[i]
    return 2 * np.pi * radius / vs, vs


def make_tracks(cfg, ds=0.5):
    """The autocross and endurance tracks named in ``cfg.events``."""
    tr = {}
    for e, f in (("autocross", cfg.events.autocross_track), ("endurance", cfg.events.endurance_track)):
        if f:
            tr[e] = load_track(f, ds)
    return tr
