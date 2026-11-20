"""Results: one solved lap, and writing a run to disk so it can be reproduced and compared later.

``write_run(res, car, folder)`` writes

    events.csv            time and points per event
    endurance.csv         one row per endurance lap (when run)
    laps/<event>.csv      every channel along each lap
    metadata.json         the car file, its hash, the git commit, the package version and the date

``validation.compare`` reads two such folders back.
"""
from __future__ import annotations

import dataclasses as dc
import datetime as dt
import hashlib
import json
import subprocess
from pathlib import Path

import numpy as np
import pandas as pd

from lapsim.core.track import Track


@dc.dataclass
class LapResult:
    track: Track
    s: np.ndarray
    v: np.ndarray
    ax: np.ndarray          # dv/dt along the path
    ay: np.ndarray          # v^2 kappa
    t: np.ndarray
    time: float
    state: dict             # vehicle state along the lap (core.vehicle.Vehicle.state)
    energy_J: float

    def frame(self) -> pd.DataFrame:
        d = {"s": self.s, "t": self.t, "v": self.v, "ax": self.ax, "ay": self.ay,
             "roll_deg": self.state["roll_deg"], "pitch_deg": self.state["pitch_deg"]}
        for k in ("fo", "fi", "ro", "ri"):
            d[f"Fz_{k}"] = self.state["Fz"][k]
            d[f"camber_{k}"] = self.state["camber"][k]
            d[f"damper_{k}"] = self.state["damper"][k]
        return pd.DataFrame(d)


def _git_commit(path: Path) -> str:
    try:
        return subprocess.run(["git", "rev-parse", "--short", "HEAD"], cwd=path, capture_output=True,
                              text=True, timeout=5).stdout.strip()
    except (OSError, subprocess.SubprocessError):
        return ""


def write_run(res, car, folder) -> Path:
    """Write an ``EventResults`` of ``car`` (config.loader.Car) to ``folder``; returns the folder."""
    from lapsim import __version__
    out = Path(folder)
    (out / "laps").mkdir(parents=True, exist_ok=True)
    res.table().to_csv(out / "events.csv")
    if res.endurance is not None:
        res.endurance.to_csv(out / "endurance.csv")
    for name, lr in res.laps.items():
        if hasattr(lr, "frame"):
            lr.frame().to_csv(out / "laps" / f"{name}.csv", index=False)
    src = Path(car.source)
    meta = {"car": car.name, "car_file": str(src), "car_file_sha1": hashlib.sha1(src.read_bytes()).hexdigest(),
            "overrides": {k: v for k, v in car.overrides.items()}, "git_commit": _git_commit(src.parent),
            "lapsim_version": __version__, "written": dt.datetime.now().isoformat(timespec="seconds"),
            "times": {k: float(v) for k, v in res.times.items()}, "notes": [str(n) for n in res.notes]}
    (out / "metadata.json").write_text(json.dumps(meta, indent=1, default=str))
    return out
