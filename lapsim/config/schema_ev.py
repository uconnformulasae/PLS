"""EV configuration: the `powertrain:` section (motor, inverter, reduction, regen, strategy) and the
`pack:` section (accumulator topology, cell model, thermal model, limits) of an EV car file.

Change anything with ``evc.set("drivetrain.rear_sprocket", 40)`` or ``pack.set("cell.r0", "14 mOhm")``.
Values may be SI numbers or strings with units ("230 N*m", "80 kW", "12 mOhm", "4.2 Ah").
Temperatures are degC, motor speeds rpm.
"""
from __future__ import annotations

import copy
import dataclasses as dc
from pathlib import Path
from typing import Optional

from .schema_ic import Differential as DifferentialCfg, Wheels as WheelsCfg
from .units import parse_quantity

_TEXT = {"name", "type", "model", "sag_branches", "torque_curve_file", "chemistry"}


class _Settable:
    """get / set by dotted path, deep copy."""

    def get(self, path):
        obj = self
        for p in path.split("."):
            obj = getattr(obj, p)
        return obj

    def set(self, path, value):
        *parents, leaf = path.split(".")
        obj = self
        for p in parents:
            obj = getattr(obj, p)
        if not hasattr(obj, leaf):
            raise AttributeError(f"no field '{path}'")
        setattr(obj, leaf, _conv(leaf, value))
        return self

    def copy(self):
        return copy.deepcopy(self)


def _conv(key, v):
    if key in _TEXT:
        return None if v is None else str(v)
    if key == "capacity_ah" and isinstance(v, str):
        return parse_quantity(v) / 3600.0
    return parse_quantity(v, angle_field=key.endswith("_deg"))


def _fill(obj, data, where, listed):
    names = {f.name for f in dc.fields(obj)}
    for k, v in (data or {}).items():
        if k not in names:
            raise KeyError(f"unknown key '{where}{k}'")
        cur = getattr(obj, k)
        if dc.is_dataclass(cur):
            _fill(cur, v, f"{where}{k}.", listed)
        else:
            setattr(obj, k, _conv(k, v))
            listed.append(f"{where}{k}")


# --------------------------------------------------------------------------- powertrain
@dc.dataclass
class Motor:
    name: str = "motor"
    peak_torque: float = 230.0               # N m
    continuous_torque: float = 120.0         # N m
    peak_power: float = 100e3                # W mechanical
    max_rpm: float = 6500.0
    corner_rpm: float = 4500.0               # at rated_voltage
    rated_voltage: float = 520.0             # V dc
    kt: float = 0.60                         # N m / A rms
    phase_resistance: float = 0.012          # ohm at 20 degC
    iron_loss_1: float = 1.2                 # W / (rad/s)
    iron_loss_2: float = 0.0035              # W / (rad/s)^2
    rotor_inertia: float = 0.042             # kg m^2
    torque_curve_file: Optional[str] = None
    thermal_mass: float = 6000.0             # J/K
    ua_coolant: float = 60.0                 # W/K
    coolant_temp: float = 40.0               # degC
    start_temp: float = 40.0
    derate_start_temp: float = 110.0
    derate_end_temp: float = 130.0


@dc.dataclass
class Inverter:
    name: str = "inverter"
    max_phase_current: float = 400.0         # A rms
    max_dc_current: float = 250.0            # A
    min_dc_voltage: float = 300.0            # V
    loss_const: float = 150.0                # W
    loss_per_amp: float = 2.2                # W / A rms


@dc.dataclass
class Accumulator:
    """How the pack is used in the events (the pack itself is the `pack:` section)."""
    start_soc: float = 0.98
    soc_min: float = 0.05
    start_temp: float = 25.0
    sag_branches: str = "r0+r1"


@dc.dataclass
class Drivetrain:
    type: str = "chain"
    front_sprocket: Optional[float] = 12
    rear_sprocket: Optional[float] = 42
    ratio: Optional[float] = None
    efficiency: float = 0.96
    inertia: float = 0.03                    # kg m^2 at the axle
    chain_pitch: float = 0.015875            # m (520 chain), for the chain tension channel
    ratio_options: list = dc.field(default_factory=lambda: [3.0, 3.25, 3.5, 3.75, 4.0, 4.25])

    @property
    def overall(self) -> float:
        if self.ratio:
            return float(self.ratio)
        if not (self.front_sprocket and self.rear_sprocket):
            raise ValueError("drivetrain: give ratio, or front_sprocket and rear_sprocket")
        return float(self.rear_sprocket) / float(self.front_sprocket)


@dc.dataclass
class Regen:
    enabled: bool = True
    max_torque: float = 80.0                 # N m
    max_power: float = 30e3                  # W into the accumulator
    min_speed: float = 5 / 3.6               # m/s
    fade_speed: float = 15 / 3.6             # m/s
    rear_brake_share: float = 1.0
    max_soc: float = 0.97


@dc.dataclass
class LV:
    aux_power: float = 350.0                 # W
    dcdc_efficiency: float = 0.90


@dc.dataclass
class Strategy:
    power_limit: Optional[float] = None      # W at the accumulator
    endurance_power_limit: Optional[float] = None
    torque_limit: Optional[float] = None     # N m
    max_speed: Optional[float] = None        # m/s


@dc.dataclass
class Endurance:
    driver_change_s: float = 180.0


@dc.dataclass
class Scoring:
    energy_min_per_lap: Optional[float] = None   # J (give it as "0.25 kWh")
    t_min_lap: Optional[float] = None
    ef_min: float = 0.10
    ef_max: float = 0.85


@dc.dataclass
class EVConfig(_Settable):
    name: str = "ev_powertrain"
    confirmed: list = dc.field(default_factory=list)
    motor: Motor = dc.field(default_factory=Motor)
    inverter: Inverter = dc.field(default_factory=Inverter)
    accumulator: Accumulator = dc.field(default_factory=Accumulator)
    drivetrain: Drivetrain = dc.field(default_factory=Drivetrain)
    differential: DifferentialCfg = dc.field(default_factory=DifferentialCfg)
    wheels: WheelsCfg = dc.field(default_factory=WheelsCfg)
    regen: Regen = dc.field(default_factory=Regen)
    lv: LV = dc.field(default_factory=LV)
    strategy: Strategy = dc.field(default_factory=Strategy)
    endurance: Endurance = dc.field(default_factory=Endurance)
    scoring: Scoring = dc.field(default_factory=Scoring)
    source: Optional[str] = None
    listed: list = dc.field(default_factory=list)      # keys present in the car file


def ev_from_dict(data: dict, base: Path, source: str = "", name: str = "ev_powertrain") -> EVConfig:
    data = dict(data or {})
    evc = EVConfig(name=data.pop("name", name), confirmed=list(data.pop("confirmed", None) or []), source=source)
    _fill(evc, data, "", evc.listed)
    f = evc.motor.torque_curve_file
    if f and not Path(f).is_absolute():
        evc.motor.torque_curve_file = str((Path(base) / f).resolve())
    check_confirmed(evc.confirmed, evc.listed, "powertrain")
    return evc


def check_confirmed(confirmed, listed, where):
    bad = [k for k in confirmed if k not in listed]
    if bad:
        raise KeyError(f"{where}: `confirmed` names {bad}, which are not values in this section")


# --------------------------------------------------------------------------- accumulator
# OCV curves (average of the charge and discharge rest curves) and default cell parameters.  ASSUMED
_LFP_SOC = [0.00, 0.02, 0.05, 0.10, 0.15, 0.20, 0.30, 0.40, 0.50, 0.60, 0.65, 0.70, 0.80, 0.90, 0.95, 0.98, 1.00]
_LFP_OCV = [2.500, 2.900, 3.100, 3.195, 3.225, 3.248, 3.272, 3.287, 3.297, 3.305, 3.315, 3.326, 3.332, 3.338, 3.355, 3.400, 3.550]
_NMC_SOC = [0.00, 0.02, 0.05, 0.10, 0.15, 0.20, 0.30, 0.40, 0.50, 0.60, 0.70, 0.80, 0.90, 0.95, 1.00]
_NMC_OCV = [2.750, 3.080, 3.300, 3.440, 3.510, 3.560, 3.625, 3.672, 3.725, 3.802, 3.893, 3.985, 4.075, 4.120, 4.180]
PRESETS = {
    "lfp": dict(ocv_soc=_LFP_SOC, ocv_v=_LFP_OCV, capacity_ah=2.5, r0=8.0e-3, r1=3.0e-3, tau1=8.0, r2=4.0e-3,
                tau2=120.0, eta_charge=0.995, ea_r=25e3, v_min=2.5, v_max=3.65, docv_dt=-0.0001),
    "nmc": dict(ocv_soc=_NMC_SOC, ocv_v=_NMC_OCV, capacity_ah=4.2, r0=14.0e-3, r1=6.0e-3, tau1=5.0, r2=8.0e-3,
                tau2=80.0, eta_charge=0.998, ea_r=28e3, v_min=2.5, v_max=4.2, docv_dt=0.0),
}


@dc.dataclass
class Pack:
    series: int = 140                    # cells (parallel groups) in series
    parallel: int = 4                    # physical cells per group
    segments: int = 7                    # accumulator segments
    interconnect_resistance: float = 0.0  # ohm: busbars, fuses, contactors, MSD


@dc.dataclass
class Cell:
    """ONE physical cell; None = the chemistry preset's value."""
    chemistry: str = "nmc"
    capacity_ah: Optional[float] = None
    r0: Optional[float] = None           # ohm at t_ref, ohmic
    r1: Optional[float] = None           # ohm, fast RC
    tau1: Optional[float] = None         # s
    r2: Optional[float] = None           # ohm, slow RC; 0 -> 1-RC model
    tau2: Optional[float] = None         # s
    eta_charge: Optional[float] = None   # coulombic efficiency on charge
    ea_r: Optional[float] = None         # J/mol, Arrhenius activation energy of the resistances
    t_ref: float = 25.0                  # degC at which r0/r1/r2 are given
    v_min: Optional[float] = None
    v_max: Optional[float] = None
    docv_dt: Optional[float] = None      # V/K entropic OCV coefficient
    ocv_soc: Optional[list] = None       # override the preset OCV table
    ocv_v: Optional[list] = None

    def resolved(self) -> dict:
        """Preset values with every explicit entry in this section overriding them."""
        if self.chemistry not in PRESETS:
            raise ValueError(f"unknown chemistry '{self.chemistry}' (choose from {list(PRESETS)})")
        out = dict(PRESETS[self.chemistry])
        for f in dc.fields(self):
            v = getattr(self, f.name)
            if v is not None:
                out[f.name] = v
        return out


@dc.dataclass
class Thermal:
    """Lumped cell temperature: one heat capacity per physical cell, cooled to ``ambient``."""
    ambient: float = 25.0                # degC, air / coolant temperature
    cell_mass: float = 0.068             # kg per physical cell (21700)
    cell_cp: float = 1000.0              # J/(kg K)
    r_core_surface: float = 2.0          # K/W per physical cell, core -> can
    r_surface_ambient: float = 15.0      # K/W per physical cell, can -> air / cold plate


@dc.dataclass
class Limits:
    """Rule and cell limits - CHECK THE CURRENT RULEBOOK AND YOUR CELL DATASHEET."""
    max_power: float = 80e3              # W at the accumulator
    cell_t_max: float = 60.0             # degC, rule / shutdown limit
    cell_t_derate: float = 50.0          # degC, start of linear current derating (to zero at cell_t_max)
    cell_t_min_charge: float = 0.0       # degC, no charging (regen) below this
    i_dis_peak: float = 45.0             # A per physical cell, peak discharge
    i_chg_peak: float = 8.4              # A per physical cell, regen / charge


@dc.dataclass
class PackConfig(_Settable):
    name: str = "accumulator"
    pack: Pack = dc.field(default_factory=Pack)
    cell: Cell = dc.field(default_factory=Cell)
    thermal: Thermal = dc.field(default_factory=Thermal)
    limits: Limits = dc.field(default_factory=Limits)
    source: str = ""


def pack_from_dict(data: dict, source: str = "") -> PackConfig:
    data = dict(data or {})
    cfg = PackConfig(name=data.pop("name", "accumulator"), source=source)
    _fill(cfg, data, "pack.", [])
    return cfg
