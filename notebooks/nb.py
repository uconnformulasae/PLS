"""Helpers shared by the notebooks: car loading, sliders bound to car-file values, a run button.

Every slider is bound to one value of the car file by the same key the website and the ``lapsim`` command
use ('<section>:<dotted.path>', section vehicle, pt, ev or pack) and shows it in a readable unit.
"""
from __future__ import annotations

import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:                 # lapsim works without `pip install -e .`
    sys.path.insert(0, str(ROOT))

import warnings  # noqa: E402

import ipywidgets as W  # noqa: E402
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
import pandas as pd  # noqa: E402
from IPython.display import display  # noqa: E402

import lapsim  # noqa: E402
import lapsim.core.events  # noqa: E402,F401
import lapsim.solvers.qss_ic  # noqa: E402,F401
import lapsim.solvers.transient  # noqa: E402,F401
import lapsim.subsystems.brakes  # noqa: E402,F401
import lapsim.subsystems.frame  # noqa: E402,F401
import lapsim.subsystems.power.ev  # noqa: E402,F401
import lapsim.subsystems.suspension  # noqa: E402,F401
import lapsim.subsystems.tires  # noqa: E402,F401

warnings.filterwarnings("ignore")
plt.rcParams.update({"figure.dpi": 100, "axes.grid": True, "grid.alpha": 0.3, "figure.autolayout": True})
CARS = {"CT-17 IC": "ct17_ic", "CT-17 EV": "ct17_ev"}


def get(car, key):
    which, _, path = key.partition(":")
    return car.objects()[which].get(path)


def slider(car, key, lo, hi, step, label, scale=1.0, unit=""):
    """A FloatSlider for car-file value ``key`` shown as value * scale [unit] (a value that is not set
    starts at ``hi``)."""
    v = get(car, key)
    x = float(v) * scale if v is not None else hi
    lo, hi = min(lo, x), max(hi, x)                 # never clamp (and so change) the car's own value
    s = W.FloatSlider(value=x, min=lo, max=hi, step=step, description=label,
                      continuous_update=False, readout_format=".3g", style={"description_width": "190px"},
                      layout=W.Layout(width="520px"))
    s.key, s.scale, s.unit = key, scale, unit
    if unit:
        s.description = f"{label} [{unit}]"
    return s


def with_values(car, sliders):
    """A copy of ``car`` with every slider's value set."""
    c = car.copy()
    for s in sliders:
        c.set(s.key, s.value / s.scale)
    return c


def run_button(label, fn, out):
    """A button that clears ``out`` and calls ``fn()`` inside it."""
    b = W.Button(description=label, button_style="primary", layout=W.Layout(width="220px"))

    def go(_):
        with out:
            out.clear_output(wait=True)
            print("running ...")
            out.clear_output(wait=True)
            fn()
    b.on_click(go)
    return b


def panel(sliders, *extra):
    return W.VBox([*sliders, *extra])
