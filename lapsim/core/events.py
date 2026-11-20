"""The FSAE dynamic events: names, scoring, and the result of running them.

Scoring follows the FSAE rules (Tmax = k * Tmin). The constants change between rule years: check
SCORING against the rules for the competition you simulate. The Michigan 2026 maxima (accel 100,
skidpad 75, autocross 125, endurance 275) are consistent with these constants.
"""
from __future__ import annotations

import dataclasses as dc

import pandas as pd

EVENTS = ("acceleration", "skidpad", "autocross", "endurance")
ROLL_IN = 0.30     # m: the car is staged 0.30 m behind the acceleration start line; timing starts at the line

SCORING = {
    #  event:       (max_points_variable, min_points, Tmax/Tmin, exponent)
    "acceleration": (95.5, 4.5, 1.5, 1),
    "skidpad":      (71.5, 3.5, 1.25, 2),
    "autocross":    (118.5, 6.5, 1.45, 1),
    "endurance":    (250.0, 25.0, 1.45, 1),
}


def points(event: str, t: float, tmin: float) -> float:
    A, B, k, p = SCORING[event]
    tmax = k * tmin
    if t >= tmax:
        return B                                   # finishing points
    t = max(t, tmin)
    return A * ((tmax / t) ** p - 1) / (k ** p - 1) + B


def event_points(events_cfg, times: dict) -> dict:
    """Points for every event whose best time (``events.tmin_*``) is set."""
    pts = {}
    for e, t in times.items():
        tmin = getattr(events_cfg, f"tmin_{'accel' if e == 'acceleration' else e}", None)
        if tmin is not None:
            pts[e] = points(e, t, tmin)
    return pts


@dc.dataclass
class EventResults:
    """What ``run_events`` returns for either car."""
    times: dict              # event -> s
    points: dict             # event -> points (events with a Tmin; plus "efficiency")
    laps: dict               # event -> lap result; "endurance_all" -> every endurance lap
    endurance: pd.DataFrame | None    # one row per endurance lap
    tggv: object             # tire + differential envelope the laps were solved on
    vehicle: object          # ICVehicle / EVVehicle
    tables: object           # drive-force tables of the power module
    notes: list

    def table(self) -> pd.DataFrame:
        rows = [{"event": e, "time_s": self.times.get(e), "points": self.points.get(e)}
                for e in (*EVENTS, "efficiency") if e in self.times or e in self.points]
        return pd.DataFrame(rows).set_index("event")
