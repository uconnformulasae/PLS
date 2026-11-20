"""Load a car file (cars/<car>/vehicle.yaml) into config objects, and build and run the car.

A car file has these top-level keys:

    name:        the car's name
    kind:        ic | ev
    base:        (optional) another car file whose `vehicle:` section this one starts from; then the
                 `vehicle:` section here lists only what is different (the CT-17EV uses the CT-17IC chassis)
    confirmed:   (optional, with base) dotted keys of `vehicle:` values that are real measurements;
                 every other value listed in the overlay is reported as a PLACEHOLDER
    vehicle:     chassis, tires, suspension, aero, brakes, events, solver settings  (config.schema)
    powertrain:  ic: engine ... cooling (config.schema_ic);  ev: motor ... strategy (config.schema_ev)
    pack:        ev only: accumulator (config.schema_ev)

File paths inside a section are relative to the car file.

    car = load_car("ct17_ev")                         # or a path to any car file
    car.set("vehicle:mass.cg_height", "11 in")         # change any value: '<section>:<dotted.path>'
    res = car.run()                                    # all four events
"""
from __future__ import annotations

import copy
import dataclasses as dc
from pathlib import Path

import yaml

from . import schema, schema_ev, schema_ic
from .register import input_status

REPO = Path(__file__).resolve().parents[2]
CARS = REPO / "cars"
TRACKS = REPO / "tracks"
# override prefixes ('<prefix>:<dotted.path>') -> object, as the website and the CLI use them
PREFIXES = {"vehicle": "vehicle", "pt": "power", "ev": "power", "pack": "pack"}


def car_file(name_or_path) -> Path:
    """cars/<name>/vehicle.yaml, or the path itself."""
    p = Path(name_or_path)
    if p.is_file():
        return p.resolve()
    q = CARS / str(name_or_path) / "vehicle.yaml"
    if q.is_file():
        return q
    known = sorted(d.name for d in CARS.iterdir() if (d / "vehicle.yaml").is_file()) if CARS.is_dir() else []
    raise FileNotFoundError(f"no car file {name_or_path!r} (cars available: {', '.join(known) or 'none'})")


def _read(path: Path) -> dict:
    data = yaml.safe_load(path.read_text()) or {}
    unknown = set(data) - {"name", "kind", "base", "confirmed", "vehicle", "powertrain", "pack"}
    if unknown:
        raise KeyError(f"{path.name}: unknown top-level key(s) {sorted(unknown)}")
    return data


def _flatten(d, prefix=""):
    for k, v in (d or {}).items():
        if isinstance(v, dict) and prefix.count(".") < 1 and k not in ("components", "settings", "drs", "hardpoints"):
            yield from _flatten(v, f"{prefix}{k}.")
        else:
            yield f"{prefix}{k}", v


def _vehicle(path: Path, data: dict):
    """(CarConfig, overlay) for a car file; overlay = {'listed', 'confirmed'} when it has a `base`."""
    if not data.get("base"):
        return schema.from_dict(data.get("vehicle"), path.parent, data.get("name", path.parent.name)), None
    base = (path.parent / data["base"]).resolve()
    cfg = schema.from_dict(_read(base).get("vehicle"), base.parent)
    cfg.name = data.get("name", path.parent.name)
    listed = []
    for key, val in _flatten(data.get("vehicle")):
        leaf = key.rsplit(".", 1)[-1]
        if isinstance(val, str) and (leaf.endswith("_file") or leaf.endswith("_track")) and not Path(val).is_absolute():
            val = str((path.parent / val).resolve())
        cfg.set(key, val)
        listed.append(key)
    if "aero.rho" not in listed and any(k.startswith("aero.ambient") for k in listed):
        cfg.aero.rho = cfg.aero.density()
    confirmed = list(data.get("confirmed") or [])
    schema_ev.check_confirmed(confirmed, listed, "vehicle")
    return cfg, {"listed": listed, "confirmed": confirmed}


@dc.dataclass
class Car:
    name: str
    kind: str                       # "ic" | "ev"
    vehicle: schema.CarConfig
    power: object                   # schema_ic.PTConfig | schema_ev.EVConfig
    pack: schema_ev.PackConfig | None
    source: Path
    overlay: dict | None = None
    overrides: dict = dc.field(default_factory=dict)

    # ------------------------------------------------------------------ objects by override prefix
    def objects(self) -> dict:
        """{'vehicle': ..., 'pt' | 'ev': ..., 'pack': ...}: the keys of '<prefix>:<path>' overrides."""
        out = {"vehicle": self.vehicle, "pt" if self.kind == "ic" else "ev": self.power}
        if self.pack is not None:
            out["pack"] = self.pack
        return out

    def set(self, key: str, value):
        """Change one value: key = '<prefix>:<dotted.path>' (prefix vehicle, pt / ev, pack), value in SI
        or a string with units ("350 lbf/in")."""
        which, _, path = key.partition(":")
        obj = self.objects().get(which)
        if obj is None or not path:
            raise KeyError(f"{key!r}: expected '<section>:<path>' with section one of {sorted(self.objects())}")
        obj.set(path, value)
        self.overrides[key] = value
        return self

    def copy(self) -> "Car":
        return copy.deepcopy(self)

    def status(self):
        """Which values are confirmed and which are placeholders (EV car; None for a car without a base)."""
        if self.overlay is None and self.kind == "ic":
            return None
        return input_status(self.vehicle, self.overlay or {"listed": [], "confirmed": []}, self.power, self.pack)

    # ------------------------------------------------------------------ the car
    def build(self):
        """The vehicle model: ICVehicle or EVVehicle."""
        if self.kind == "ic":
            from lapsim.subsystems.power.ic import ICVehicle
            return ICVehicle(self.vehicle, self.power)
        from lapsim.subsystems.power.ev import EVVehicle
        return EVVehicle(self.vehicle, self.power, self.pack)

    def run(self, events=None, **kw):
        """Run the dynamic events (default: all four) with the QSS solver of the car's power module."""
        from lapsim.core.events import EVENTS
        events = tuple(events or EVENTS)
        if self.kind == "ic":
            from lapsim.solvers import qss_ic
            return qss_ic.run_events(self.vehicle, self.power, events=events, **kw)
        from lapsim.solvers import qss_ev
        return qss_ev.run_events(self.vehicle, self.power, self.pack, events=events, **kw)


def load_car(name_or_path, overrides: dict | None = None) -> Car:
    path = car_file(name_or_path)
    data = _read(path)
    kind = str(data.get("kind", "")).lower()
    if kind not in ("ic", "ev"):
        raise KeyError(f"{path.name}: kind must be 'ic' or 'ev', not {data.get('kind')!r}")
    cfg, overlay = _vehicle(path, data)
    name = str(data.get("name", path.parent.name))
    if kind == "ic":
        power, pack = schema_ic.from_dict(data.get("powertrain"), path.parent, f"{name}_powertrain"), None
        if data.get("pack"):
            raise KeyError(f"{path.name}: `pack:` is for an EV car file")
    else:
        power = schema_ev.ev_from_dict(data.get("powertrain"), path.parent, str(path), f"{name}_powertrain")
        pack = schema_ev.pack_from_dict(data.get("pack"), str(path))
    car = Car(name, kind, cfg, power, pack, path, overlay)
    for k, v in (overrides or {}).items():
        car.set(k, v)
    return car
