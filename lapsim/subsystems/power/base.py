"""What every power module must provide.

A power module (``ic.py``, ``ev.py``) turns the chassis (``core.vehicle.Vehicle``) into a car by
subclassing it and setting, after ``Vehicle.__init__``:

    pt              a ``DriveLimit``: the largest tractive force at the rear contact patches
    dt              the driveline; the chassis reads ``dt.r`` (rolling radius used by the differential)
    diff            ``subsystems.driveline.Differential`` (rear drive split, locking, coast ramp)
    ptc             the module's configuration (the car file's `powertrain:` section)
    coast_torque(v) axle torque on the coast side of the differential while braking
                    (engine braking for the IC car, regen for the EV)

and by giving its QSS solver (``solvers/qss_ic.py``, ``solvers/qss_ev.py``) a table of the drive force
on a speed grid (``ICTables`` / ``EVTables``: ``v``, ``F``, ``R``, ``A``, ``v_top``, ``idx(v)``).

The chassis builds its tire-and-differential envelope with ``traction_ggv()`` (drive limit switched
off); the solver then applies the power module's limit point by point, gear by gear for the IC car and
for the current state of charge and temperatures for the EV.
"""
from __future__ import annotations

from typing import Protocol

import numpy as np


class DriveLimit(Protocol):
    unlimited: bool          # True while the chassis builds the traction-only envelope

    def force(self, v) -> np.ndarray:
        """Largest tractive force [N] at speed v [m/s] (inf when ``unlimited``)."""
        ...
