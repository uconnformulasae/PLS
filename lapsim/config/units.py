"""Unit handling.

Every value in a configuration file may be written either as a plain number
(interpreted in the SI unit of that field) or as a string with a unit, e.g.
"11 in", "350 lbf/in", "62 N/mm", "66.54 deg", "238 N*m/deg".
Internally everything is SI (m, kg, N, s, rad) except angles stored in fields
whose names end in ``_deg``, which stay in degrees for readability.
"""
from __future__ import annotations

import re

G0 = 9.80665          # standard gravity, m/s^2
LBF = 4.4482216152605  # N
LBM = 0.45359237       # kg
INCH = 0.0254          # m

# factor to SI for each recognised unit token
_FACTORS = {
    # length
    "m": 1.0, "mm": 1e-3, "cm": 1e-2, "km": 1e3, "in": INCH, "ft": 0.3048, "mi": 1609.344,
    # mass
    "kg": 1.0, "g": 1e-3, "lb": LBM, "lbm": LBM,
    # force
    "N": 1.0, "kN": 1e3, "lbf": LBF,
    # time
    "s": 1.0, "ms": 1e-3, "min": 60.0, "h": 3600.0,
    # angle (to degrees for *_deg fields, handled separately)
    "deg": 1.0, "rad": 57.29577951308232,
    # power / energy
    "W": 1.0, "kW": 1e3, "hp": 745.6998715822702, "J": 1.0, "kJ": 1e3, "kWh": 3.6e6,
    # pressure
    "Pa": 1.0, "kPa": 1e3, "bar": 1e5, "psi": 6894.757293168,
    # volume (powertrain: displacement, fuel)
    "cc": 1e-6, "L": 1e-3, "l": 1e-3, "gal": 3.785411784e-3,
    # electrical (battery model)
    "V": 1.0, "mV": 1e-3, "A": 1.0, "mA": 1e-3, "Ah": 3600.0, "mAh": 3.6, "Wh": 3600.0,
    "ohm": 1.0, "Ohm": 1.0, "mohm": 1e-3, "mOhm": 1e-3, "F": 1.0, "kF": 1e3,
    # dimensionless
    "%": 0.01, "": 1.0,
}
_SPEED = {"mph": 0.44704, "kph": 1 / 3.6, "km/h": 1 / 3.6, "m/s": 1.0}
_ACC = {"g": G0}


def _parse_unit_expr(expr: str) -> float:
    """Convert a compound unit like 'lbf/in', 'N*m/deg', 'kg*m^2' into its SI factor."""
    expr = expr.strip()
    if expr in _SPEED:
        return _SPEED[expr]
    num, _, den = expr.partition("/")
    def prod(part: str) -> float:
        f = 1.0
        for tok in re.split(r"[*·]", part):
            tok = tok.strip()
            if not tok:
                continue
            m = re.fullmatch(r"([A-Za-z%]+)\^?(-?\d+)?", tok)
            if not m or m.group(1) not in _FACTORS:
                raise ValueError(f"unknown unit '{tok}' in '{expr}'")
            f *= _FACTORS[m.group(1)] ** (int(m.group(2)) if m.group(2) else 1)
        return f
    f = prod(num)
    for d in den.split("/") if den else []:
        f /= prod(d)
    return f


def parse_quantity(value, *, angle_field: bool = False, accel_field: bool = False):
    """Return ``value`` converted to SI (or degrees for angle fields).

    Numbers, None, bools, lists and dicts pass through unchanged (lists are
    converted element-wise).
    """
    if value is None or isinstance(value, (bool, int, float)):
        return value
    if isinstance(value, (list, tuple)):
        return [parse_quantity(v, angle_field=angle_field, accel_field=accel_field) for v in value]
    if not isinstance(value, str):
        return value
    s = value.strip()
    m = re.fullmatch(r"([-+]?(?:\d+\.?\d*|\.\d+)(?:[eE][-+]?\d+)?)\s*(.*)", s)
    if not m:
        return value  # not a quantity (e.g. a file path or a keyword)
    number, unit = float(m.group(1)), m.group(2).strip()
    if unit == "":
        return number
    if accel_field and unit == "g":
        return number * G0
    if angle_field:
        if unit not in ("deg", "rad"):
            raise ValueError(f"angle field given unit '{unit}'")
        return number * _FACTORS[unit]
    return number * _parse_unit_expr(unit)


def to_unit(value_si: float, unit: str) -> float:
    """Convert an SI value to the given display unit (inverse of parse_quantity)."""
    if unit == "g":
        return value_si / G0
    return value_si / _parse_unit_expr(unit)
