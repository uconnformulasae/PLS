"""IC powertrain configuration: the `powertrain:` section of an IC car file.

Engine, intake, fuel, clutch, gearbox, final drive, differential, half-shafts, cooling and
electrics. Change anything with ``pt.set("final_drive.rear_sprocket", 31)``. Units follow ``lapsim.config.units``
(SI internally; strings with units are converted). Temperatures are degC and
engine speeds rpm, as the team reads them.
"""
from __future__ import annotations

import copy
import dataclasses as dc
from pathlib import Path
from typing import Optional

from .units import parse_quantity

_TEXT = {"type", "model", "cv_type", "shifter", "chain", "name", "injectors", "pump_type", "radiator", "dyno_file"}


def F(default=None, note: str = "", source: str = "", unit: str = ""):
    return dc.field(default=default, metadata={"note": note, "source": source, "unit": unit})


def FL(factory, note="", source="", unit=""):
    return dc.field(default_factory=factory, metadata={"note": note, "source": source, "unit": unit})


@dc.dataclass
class Ambient:
    pressure: float = F(95.9e3, "barometric pressure (engine-off MAP)", "LOG", "Pa")
    temperature: float = F(25.0, "ambient air temperature", "ASSUMED", "degC")
    intake_temp_rise: float = F(5.0, "intake air temperature above ambient", "LOG", "K")


@dc.dataclass
class Engine:
    displacement: float = F(599e-6, "swept volume", "R6", "m^3")
    cylinders: int = F(4, "", "R6", "")
    bore: float = F(0.067, "", "R6", "m")
    stroke: float = F(0.0425, "", "R6", "m")
    compression_ratio: float = F(13.1, "", "R6", "")
    dyno_file: Optional[str] = F(None, "motor dyno csv (rpm, torque_ftlb or torque_Nm)", "PT61", "")
    dyno_pressure: float = F(101325.0, "dyno correction pressure", "ASSUMED", "Pa")
    dyno_temperature: float = F(25.0, "dyno correction temperature", "ASSUMED", "degC")
    hub_dyno_power: Optional[float] = F(None, "power at the hubs", "PT63", "W")
    hub_dyno_rpm: Optional[float] = F(None, "", "PT63", "rpm")
    torque_scale: float = F(1.0, "tune factor on brake torque (endurance tune ~0.973)", "PT64", "-")
    exhaust_event_duration: float = F(232.0, "ED in P = 850 ED / rpm - 3", "ASSUMED", "deg")
    rev_limit: float = F(13000.0, "", "LOG", "rpm")
    idle_rpm: float = F(3000.0, "", "ASSUMED", "rpm")
    launch_rpm: float = F(12200.0, "launch-control rpm", "LOG", "rpm")
    min_drive_rpm: float = F(4500.0, "lowest usable rpm (downshift below)", "ASSUMED", "rpm")
    ve_rpm: list = FL(list, "VE table rpm", "", "rpm")
    ve: list = FL(list, "volumetric efficiency table (plenum density)", "fit", "-")
    eta_indicated: float = F(0.37, "gross indicated efficiency", "ASSUMED/fit", "-")
    lambda_wot: float = F(0.92, "", "LOG", "-")
    lambda_part: float = F(1.0, "", "LOG", "-")
    fmep_a: float = F(30.0, "FMEP constant", "ASSUMED", "kPa")
    fmep_b: float = F(5.0, "FMEP per piston speed", "ASSUMED", "kPa/(m/s)")
    fmep_c: float = F(0.3, "FMEP per piston speed^2", "ASSUMED", "kPa/(m/s)^2")
    exhaust_back_pressure: float = F(105e3, "", "ASSUMED", "Pa")
    crank_inertia: float = F(0.010, "", "ASSUMED", "kg m^2")
    derate_start_temp: float = F(105.0, "", "ASSUMED", "degC")
    derate_per_degC: float = F(0.008, "", "ASSUMED", "1/K")
    overrun_fuel_cut: bool = F(True, "", "ASSUMED", "")
    fuel_scale: float = F(1.0, "injected fuel / modelled fuel (fitted from the ECU fuel-flow channel)", "fit", "-")
    intake_runner_length: float = F(0.1044, "", "PT19", "m")
    exhaust_primary_length: float = F(0.38, "", "ASSUMED", "m")
    intake_runner_ref: float = F(0.1044, "", "PT19", "m")
    exhaust_primary_ref: float = F(0.38, "", "", "m")
    tuning_amplitude: float = F(0.05, "", "ASSUMED", "-")


@dc.dataclass
class Intake:
    restrictor_diameter: float = F(0.020, "", "RULE", "m")
    restrictor_cd: float = F(0.95, "", "ASSUMED", "-")
    diffuser_recovery: float = F(0.70, "fraction of throat dynamic pressure recovered by the diffuser", "ASSUMED", "-")
    plenum_volume: float = F(2.28e-3, "", "PT19", "m^3")
    loss_coef: float = F(0.0, "air filter + throttle body loss: dp = K * m_air^2", "fit", "Pa/(kg/s)^2")


@dc.dataclass
class Fuel:
    type: str = F("gasoline", "'gasoline' or 'e85'", "PT8", "")
    tank_volume: float = F(5.2e-3, "", "PT41", "m^3")
    start_fill: float = F(5.2e-3, "", "PT41", "m^3")
    endurance_fuel_normal: Optional[float] = F(None, "binder endurance fuel, normal tune", "PT64", "m^3")
    endurance_fuel_endurance_tune: Optional[float] = F(None, "binder endurance fuel, endurance tune", "PT64", "m^3")
    endurance_tune_fuel_scale: float = F(0.930, "", "PT64", "-")
    injectors: str = F("", "", "PT41", "")


@dc.dataclass
class Clutch:
    torque_capacity: float = F(140.0, "", "ASSUMED", "N m")
    inertia: float = F(0.012, "", "ASSUMED", "kg m^2")


@dc.dataclass
class Gearbox:
    primary_ratio: float = F(2.073, "", "R6", "-")
    gear_ratios: list = FL(lambda: [2.583, 2.0, 1.667, 1.444, 1.286, 1.15], "", "R6", "-")
    primary_efficiency: float = F(0.985, "", "ASSUMED", "-")
    mesh_efficiency: float = F(0.98, "", "ASSUMED", "-")
    loss_scale: float = F(1.0, "multiplies every stage loss (calibrated to the hub dyno)", "PT63 fit", "-")
    spin_loss_a: float = F(0.4, "", "ASSUMED", "N m")
    spin_loss_b: float = F(0.0008, "", "ASSUMED", "N m s/rad")
    output_inertia: float = F(0.004, "", "ASSUMED", "kg m^2")
    upshift_time: float = F(0.09, "", "LOG", "s")
    downshift_time: float = F(0.15, "", "LOG", "s")
    shift_rpm: Optional[float] = F(None, "None = optimal", "", "rpm")
    lowest_running_gear: int = F(1, "lowest gear used after the launch (drivers often hold 2nd in hairpins)", "", "")
    gear_channel_offset: int = F(2, "logged gear - real gear", "LOG", "")
    shifter: str = F("electro-pneumatic paddle", "", "LOG", "")


@dc.dataclass
class FinalDrive:
    front_sprocket: int = F(11, "", "PT55", "teeth")
    rear_sprocket: int = F(32, "", "PT55", "teeth")
    rear_sprocket_options: list = FL(lambda: [30, 31, 32], "", "team", "teeth")
    front_sprocket_options: list = FL(lambda: [10, 11, 12], "", "", "teeth")
    chain: str = F("520", "", "ASSUMED", "")
    chain_pitch: float = F(0.015875, "", "", "m")
    chain_tensile_strength: Optional[float] = F(None, "", "PT77", "N")
    chain_mass_per_m: float = F(0.85, "", "ASSUMED", "kg/m")
    chain_artic_coef: float = F(0.205, "", "ASSUMED", "-")
    centre_distance: float = F(0.42, "", "ASSUMED", "m")
    sprocket_inertia: float = F(0.010, "", "ASSUMED", "kg m^2")


@dc.dataclass
class Differential:
    type: str = F("drexler", "'drexler' | 'open' | 'spool' | 'ideal'", "", "")
    model: str = F("Drexler FSAE V3 adjustable", "", "team", "")
    preload: float = F(43.0, "", "PT45", "N m")
    drive_ramp_deg: float = F(60.0, "", "PT45", "deg")
    coast_ramp_deg: float = F(45.0, "", "PT45", "deg")
    lockup_at_45deg: Optional[float] = F(0.51, "Drexler theoretical lock-up at 45 deg; L = value / tan(ramp)", "PT45", "-")
    friction_faces: int = F(6, "", "ASSUMED", "")
    plate_mu: float = F(0.12, "", "ASSUMED", "-")
    friction_radius: float = F(0.035, "", "ASSUMED", "m")
    ramp_radius: float = F(0.032, "", "ASSUMED", "m")
    efficiency: float = F(0.985, "", "ASSUMED", "-")
    inertia: float = F(0.015, "", "ASSUMED", "kg m^2")
    preload_options: list = FL(lambda: [5, 10, 20, 30, 45, 60], "", "", "N m")
    ramp_options: list = FL(lambda: [30, 40, 45, 50, 60], "", "", "deg")


@dc.dataclass
class Halfshaft:
    cv_type: str = F("tripod", "", "", "")
    cv_efficiency: float = F(0.995, "", "ASSUMED", "-")
    cv_loss_per_deg: float = F(0.0006, "", "ASSUMED", "1/deg")
    cv_angle_deg: float = F(6.0, "", "ASSUMED", "deg")
    torsional_stiffness: float = F(6000.0, "", "ASSUMED", "N m/rad")
    damping: float = F(4.0, "", "ASSUMED", "N m s/rad")
    od: float = F(0.0254, "", "ASSUMED", "m")
    id: float = F(0.01905, "", "ASSUMED", "m")
    inertia: float = F(0.002, "", "ASSUMED", "kg m^2")


@dc.dataclass
class Wheels:
    rolling_radius_factor: float = F(0.975, "", "LOG/ASSUMED", "-")
    rear_corner_inertia: float = F(0.20, "", "derived", "kg m^2")
    front_corner_inertia: float = F(0.18, "", "derived", "kg m^2")
    bearing_drag: float = F(0.3, "", "ASSUMED", "N m")
    slip_stiffness_per_load: float = F(25.0, "longitudinal slip stiffness / Fz (per unit slip ratio)", "ASSUMED", "-")


@dc.dataclass
class Cooling:
    core_width: float = F(0.30, "", "ASSUMED", "m")
    core_height: float = F(0.25, "", "ASSUMED", "m")
    ua_ref: float = F(900.0, "", "ASSUMED/fit", "W/K")
    air_flow_ref: float = F(0.40, "", "ASSUMED", "m^3/s")
    duct_ratio: float = F(0.30, "", "ASSUMED", "-")
    fan_flow: float = F(0.18, "", "ASSUMED", "m^3/s")
    fan_power: float = F(92.0, "", "PT71", "W")
    fan_on_temp: float = F(98.0, "", "ASSUMED", "degC")
    fan_off_temp: float = F(92.0, "", "ASSUMED", "degC")
    thermostat_open: float = F(80.0, "", "ASSUMED", "degC")
    thermostat_full: float = F(90.0, "", "ASSUMED", "degC")
    pump_type: str = F("electric", "'electric' (EWP80) or 'mechanical'", "PT66", "")
    pump_flow: float = F(50.0, "electric pump flow", "PT67", "L/min")
    pump_flow_per_krpm: float = F(0.06, "", "ASSUMED", "kg/s")
    radiator: str = F("", "", "PT66", "")
    heat_to_coolant: float = F(0.27, "", "ASSUMED/fit", "-")
    heat_to_oil: float = F(0.06, "", "ASSUMED", "-")
    thermal_mass_coolant: float = F(18000.0, "", "ASSUMED/fit", "J/K")
    thermal_mass_oil: float = F(9000.0, "", "PT33", "J/K")
    oil_to_coolant_ua: float = F(400.0, "", "ASSUMED", "W/K")
    start_temp: float = F(75.0, "", "LOG", "degC")


@dc.dataclass
class Electrical:
    alternator_max_power: float = F(380.0, "", "ASSUMED", "W")
    alternator_full_rpm: float = F(5000.0, "", "ASSUMED", "rpm")
    alternator_efficiency: float = F(0.55, "", "ASSUMED", "-")
    alternator_drag: float = F(0.25, "", "ASSUMED", "N m")
    battery_capacity_ah: float = F(4.6, "", "ASSUMED", "Ah")
    battery_voltage: float = F(13.2, "", "LOG", "V")
    start_soc: float = F(0.95, "", "", "-")
    loads: dict = FL(lambda: {"ecu": 20, "fuel_pump": 60, "injectors_ignition": 55, "logger_dash": 18,
                              "shifter": 12, "water_pump_electric": 90, "misc": 15}, "", "ASSUMED / PT66", "W")


@dc.dataclass
class Resistance:
    assumed_CdA: float = F(1.22, "", "PT4 (41.2 lb, speed ASSUMED 35 mph)", "m^2")
    assumed_Crr: float = F(0.020, "", "ASSUMED", "-")


@dc.dataclass
class Scoring:
    co2_min_per_lap: float = F(0.065, "", "ASSUMED", "kg")
    t_min_lap: Optional[float] = F(None, "", "", "s")
    ef_min: float = F(0.10, "", "ASSUMED", "-")
    ef_max: float = F(0.85, "", "ASSUMED", "-")


@dc.dataclass
class PTConfig:
    name: str = "powertrain"
    ambient: Ambient = dc.field(default_factory=Ambient)
    engine: Engine = dc.field(default_factory=Engine)
    intake: Intake = dc.field(default_factory=Intake)
    fuel: Fuel = dc.field(default_factory=Fuel)
    clutch: Clutch = dc.field(default_factory=Clutch)
    gearbox: Gearbox = dc.field(default_factory=Gearbox)
    final_drive: FinalDrive = dc.field(default_factory=FinalDrive)
    differential: Differential = dc.field(default_factory=Differential)
    halfshaft: Halfshaft = dc.field(default_factory=Halfshaft)
    wheels: Wheels = dc.field(default_factory=Wheels)
    cooling: Cooling = dc.field(default_factory=Cooling)
    electrical: Electrical = dc.field(default_factory=Electrical)
    resistance: Resistance = dc.field(default_factory=Resistance)
    scoring: Scoring = dc.field(default_factory=Scoring)

    def get(self, path):
        obj = self
        for p in path.split("."):
            obj = getattr(obj, p)
        return obj

    def set(self, path, value):
        """e.g. pt.set('final_drive.rear_sprocket', 31); pt.set('intake.restrictor_diameter', '19 mm')."""
        *parents, leaf = path.split(".")
        obj = self
        for p in parents:
            obj = getattr(obj, p)
        if not hasattr(obj, leaf):
            raise AttributeError(f"no powertrain field '{path}'")
        setattr(obj, leaf, _conv(leaf, value))
        return self

    def copy(self):
        return copy.deepcopy(self)


def _conv(key, v):
    if key in _TEXT:
        return None if v is None else str(v)
    if isinstance(v, dict):
        return {k: parse_quantity(x) for k, x in v.items()}
    return parse_quantity(v, angle_field=key.endswith("_deg"))


def _fill(obj, data, where):
    names = {f.name for f in dc.fields(obj)}
    for k, v in (data or {}).items():
        if k not in names:
            raise KeyError(f"unknown powertrain key '{where}.{k}'")
        cur = getattr(obj, k)
        if dc.is_dataclass(cur):
            _fill(cur, v, f"{where}.{k}")
        else:
            setattr(obj, k, _conv(k, v))


def from_dict(data: dict, base: Path, name: str = "powertrain") -> PTConfig:
    """A PTConfig from the parsed IC `powertrain:` section; the dyno file is resolved against ``base``."""
    data = dict(data or {})
    cfg = PTConfig()
    cfg.name = data.pop("name", name)
    _fill(cfg, data, "powertrain")
    f = cfg.engine.dyno_file
    if f and not Path(f).is_absolute():
        cfg.engine.dyno_file = str((Path(base) / f).resolve())
    return cfg
