"""Vehicle configuration: the `vehicle:` section of a car file (cars/<car>/vehicle.yaml).

Chassis, tires, suspension, aero, brakes, events and solver settings, grouped by subsystem.
Every field carries its unit, a note and the source of its value (field metadata). Edit the
car file, or change a value for one run with ``cfg.set("front.spring_rate", "300 lbf/in")``.

A value of ``None`` means "not recorded". Each analysis checks the values it
needs and raises :class:`MissingDataError` listing exactly what to supply,
instead of silently assuming a number.
"""
from __future__ import annotations

import copy
import dataclasses as dc
from pathlib import Path
from typing import Optional

from .units import parse_quantity


class MissingDataError(RuntimeError):
    """Raised when an analysis needs a value that has not been supplied."""


def F(default=None, note: str = "", source: str = "", unit: str = ""):
    """Config field with documentation metadata."""
    return dc.field(default=default, metadata={"note": note, "source": source, "unit": unit})


def FL(default_factory, note="", source="", unit=""):
    return dc.field(default_factory=default_factory,
                    metadata={"note": note, "source": source, "unit": unit})


# ---------------------------------------------------------------------------
@dc.dataclass
class Mass:
    mass_no_driver: Optional[float] = F(None, "vehicle mass without driver", "B67", "kg")
    driver_mass: Optional[float] = F(None, "driver mass", "B67", "kg")
    front_weight_fraction: Optional[float] = F(None, "static front weight fraction (with driver)", "B67", "-")
    left_weight_fraction: float = F(0.5, "static left weight fraction", "B67", "-")
    cg_height: Optional[float] = F(None, "CG height", "N, B67", "m")
    unsprung_mass_front: Optional[float] = F(None, "unsprung mass per front corner", "MISSING", "kg")
    unsprung_mass_rear: Optional[float] = F(None, "unsprung mass per rear corner", "MISSING", "kg")
    unsprung_cg_height: Optional[float] = F(None, "unsprung CG height (default: tire radius)", "", "m")
    yaw_inertia: Optional[float] = F(None, "yaw inertia Iz (transient only)", "MISSING", "kg*m^2")
    roll_inertia: Optional[float] = F(None, "sprung-mass roll inertia Ixx about its CG (transient only)", "MISSING", "kg*m^2")
    pitch_inertia: Optional[float] = F(None, "sprung-mass pitch inertia Iyy about its CG (transient only)", "MISSING", "kg*m^2")
    chassis_torsional_stiffness: Optional[float] = F(None, "frame torsional stiffness between the axles (None = rigid)", "frame beam model (subsystems.frame)", "N*m/rad")

    @property
    def total(self) -> float:
        return self.mass_no_driver + self.driver_mass


@dc.dataclass
class Geometry:
    wheelbase: Optional[float] = F(None, "wheelbase", "N, B7", "m")
    track_front: Optional[float] = F(None, "front track", "N, B7", "m")
    track_rear: Optional[float] = F(None, "rear track", "N, B7", "m")


@dc.dataclass
class Tires:
    tire_radius: Optional[float] = F(None, "loaded/unloaded radius", "B22 (8 in)", "m")
    width: Optional[float] = F(None, "tread width", "B10", "m")
    # peak friction model  mu(Fz, gamma) = lambda_mu * (mu0 + mu_fz*(Fz-Fz0)) * (1 - k_gamma*(gamma-gamma_opt)^2)
    mu0: Optional[float] = F(None, "peak lateral mu at Fz0 (from TTC fit or logged g-g)", "TTC / log", "-")
    mu_fz: Optional[float] = F(None, "load sensitivity d(mu)/dFz", "TTC / log", "1/N")
    fz0: float = F(1000.0, "reference load for mu0", "", "N")
    mux_over_muy: float = F(1.0, "longitudinal/lateral peak mu ratio (friction ellipse)", "TTC", "-")
    gamma_opt_deg: Optional[float] = F(None, "inclination angle of peak grip", "TTC", "deg")
    k_gamma: Optional[float] = F(None, "grip loss per (deg)^2 from gamma_opt", "TTC", "1/deg^2")
    cornering_stiffness_per_load: Optional[float] = F(None, "C_alpha/Fz (for understeer gradient)", "TTC", "1/rad")
    pneumatic_trail: Optional[float] = F(None, "pneumatic trail", "MISSING", "m")
    lambda_mu: float = F(1.0, "track/rig grip scaling, calibrated from logged data", "calibration", "-")
    rolling_resistance: Optional[float] = F(None, "Crr (from coast-down)", "log", "-")
    pressure_front: Optional[float] = F(None, "hot pressure front", "setup sheet", "Pa")
    pressure_rear: Optional[float] = F(None, "hot pressure rear", "setup sheet", "Pa")
    # vertical stiffness table: pressures (Pa), loads (N), rates (N/m)
    kt_pressures: list = FL(list, "tire stiffness table pressures", "B13", "Pa")
    kt_loads: list = FL(list, "tire stiffness table loads", "B13", "N")
    kt_rates: list = FL(list, "tire stiffness table rates [pressure][load]", "B13", "N/m")


@dc.dataclass
class Axle:
    """One axle's suspension. Everything here can be changed independently front/rear."""
    rod_type: str = F("pull", "'pull' or 'push'", "B67", "")
    rod_angle_deg: Optional[float] = F(None, "pull/push-rod angle from vertical (rocker plane)", "B59/62 derived", "deg")
    rocker_force_ratio: Optional[float] = F(None, "F_damper/F_rod = l_p sin(beta)/(l_d sin(alpha))", "B59 derived", "-")
    rocker_lp: Optional[float] = F(None, "rocker pull-rod-side arm", "MISSING", "m")
    rocker_ld: Optional[float] = F(None, "rocker damper-side arm", "MISSING", "m")
    rocker_alpha_deg: Optional[float] = F(None, "damper arm angle to line of action", "MISSING", "deg")
    rocker_beta_deg: Optional[float] = F(None, "rod arm angle to line of action", "MISSING", "deg")
    rod_length: Optional[float] = F(None, "rod length (buckling check, push rod)", "MISSING", "m")
    rod_od: Optional[float] = F(None, "rod tube OD", "MISSING", "m")
    rod_id: Optional[float] = F(None, "rod tube ID", "MISSING", "m")
    rod_E: float = F(200e9, "rod Young's modulus (steel)", "", "Pa")
    motion_ratio: Optional[float] = F(None, "motion ratio value", "B67", "-")
    motion_ratio_convention: str = F("wheel/damper", "'wheel/damper' or 'damper/wheel'", "B62", "")
    spring_rate: Optional[float] = F(None, "coil spring rate", "B49", "N/m")
    spring_preload: Optional[float] = F(None, "spring preload", "B49", "m")
    damper_static_length: Optional[float] = F(None, "damper eye-to-eye at static", "B22", "m")
    damper_stroke_available: Optional[float] = F(None, "usable damper stroke", "MISSING", "m")
    damping_ratio_bump: Optional[float] = F(None, "fraction of critical, jounce", "B67", "-")
    damping_ratio_rebound: Optional[float] = F(None, "fraction of critical, rebound", "B67", "-")
    arb_blade_length: Optional[float] = F(None, "ARB effective blade length", "B43-45", "m")
    arb_torsion_length: Optional[float] = F(None, "ARB torsion length", "B43", "m")
    arb_od: Optional[float] = F(None, "ARB tube OD", "B43", "m")
    arb_id: Optional[float] = F(None, "ARB tube ID", "B43", "m")
    arb_G: float = F(80e9, "ARB shear modulus", "B43 (reproduces k_arb)", "Pa")
    arb_motion_ratio: Optional[float] = F(None, "ARB blade-tip travel per wheel travel", "MISSING", "-")
    arb_roll_stiffness: Optional[float] = F(None, "override: ARB roll stiffness at the wheels", "", "N*m/deg")
    static_camber_deg: Optional[float] = F(None, "static camber (negative = top in)", "N, B67", "deg")
    static_toe_deg: float = F(0.0, "static toe (+ in)", "B67", "deg")
    roll_center_height: Optional[float] = F(None, "static roll-centre height", "N (confirmed)", "m")
    fvsa_length: Optional[float] = F(None, "front-view swing-arm length", "B19 derived", "m")
    caster_deg: Optional[float] = F(None, "caster", "N, B67", "deg")
    kpi_deg: Optional[float] = F(None, "king-pin inclination", "B67", "deg")
    scrub_radius: Optional[float] = F(None, "scrub radius", "B67", "m")
    mechanical_trail: Optional[float] = F(None, "kinematic/mechanical trail", "B67", "m")
    svsa_length: Optional[float] = F(None, "side-view swing-arm length", "B19/22", "m")
    hardpoints: Optional[dict] = F(None, "front-view hardpoints {UBJ,LBJ,UI,LI,CP}: [y,z] m, left side, y outboard", "MISSING", "m")
    ride_height: Optional[float] = F(None, "static ride height (reference point)", "B49", "m")
    hardpoint_file: Optional[str] = F(None, "OptimumK hardpoint export (.csv); drives the kinematics when given", "CT17 files", "")
    use_hardpoint_rc: bool = F(True, "take roll-centre height from the hardpoints instead of roll_center_height", "", "")
    use_hardpoint_motion_ratio: bool = F(True, "take motion ratio from the hardpoints instead of motion_ratio", "", "")
    use_hardpoint_anti: bool = F(True, "take anti-dive/lift/squat from the hardpoints", "", "")


@dc.dataclass
class Anti:
    anti_dive: Optional[float] = F(None, "front anti-dive fraction", "N (confirmed 50%)", "-")
    anti_lift: Optional[float] = F(None, "rear anti-lift fraction (braking)", "B19", "-")
    anti_squat: Optional[float] = F(None, "rear anti-squat fraction", "N, B19", "-")


@dc.dataclass
class Steering:
    ackermann_fraction: Optional[float] = F(None, "Ackermann fraction (1 = 100%)", "N 92% / B67 99%", "-")
    steer_ratio: Optional[float] = F(None, "steering-wheel angle / road-wheel angle", "B67", "-")
    steering_arm: Optional[float] = F(None, "steering arm length", "B67", "m")
    pinion_radius: Optional[float] = F(None, "pinion effective radius", "B55", "m")
    rack_efficiency: Optional[float] = F(None, "rack efficiency", "B55", "-")
    ujoint_efficiency: Optional[float] = F(None, "U-joint efficiency", "B55", "-")
    ujoint_angle_deg: Optional[float] = F(None, "column U-joint operating angle", "B56", "deg")
    wheel_radius: Optional[float] = F(None, "steering-wheel radius", "B55", "m")


@dc.dataclass
class Aero:
    model: str = F("components", "'components' (attitude-sensitive wings + body, lapsim.subsystems.aero) or 'constant' "
                   "(fixed ClA_front / ClA_rear / CdA, the original model)", "", "")
    rho: Optional[float] = F(1.225, "air density; null -> from ambient_pressure / ambient_temperature_C", "", "kg/m^3")
    ambient_pressure: Optional[float] = F(None, "ambient pressure (sets rho when rho is null)", "logs", "Pa")
    ambient_temperature_C: Optional[float] = F(None, "ambient temperature in deg C (sets rho when rho is null)", "logs", "degC")
    reference_area: Optional[float] = F(None, "frontal area that CL / CD in the binder refer to", "aero binder", "m^2")
    calib_ClA: Optional[float] = F(None, "whole-car ClA the component model is scaled to (measured)", "aero binder", "m^2")
    calib_CdA: Optional[float] = F(None, "whole-car CdA the component model is scaled to (measured)", "aero binder", "m^2")
    calib_speed: Optional[float] = F(None, "speed of the calibration measurement (ride height solved there)", "aero binder", "m/s")
    calib_front_fraction: Optional[float] = F(None, "downforce centre of pressure as front-axle fraction", "aero binder", "-")
    calib_front_axle_share: Optional[float] = F(None, "measured front-axle share of the aero load (includes the rearward transfer "
                                                "from drag); used when calib_front_fraction is null", "team measurement", "-")
    components: dict = FL(dict, "front_wing / rear_wing / body: CFD split, position, sensitivities (see lapsim.subsystems.aero)",
                          "aero binder + ASSUMED sensitivities", "")
    ground_effect: Optional[list] = F(None, "front-wing ground effect [[h/c, CL/CL_max], ...] (default: published shape)",
                                      "", "")
    drs: dict = FL(dict, "{enabled, cl_ratio, cd_ratio, max_ay}: rear-wing DRS (not fitted to CT-17)", "aero binder", "")
    aero_iterations: int = F(3, "fixed-point iterations aero <-> ride height / roll / slip angle", "", "")

    def density(self):
        """Air density: rho if given, else from the ambient pressure and temperature (dry air)."""
        if self.rho is not None:
            return self.rho
        if self.ambient_pressure is None or self.ambient_temperature_C is None:
            return 1.225
        return self.ambient_pressure / (287.058 * (self.ambient_temperature_C + 273.15))
    ClA_front: Optional[float] = F(None, "front-axle downforce coefficient x area", "B65 derived", "m^2")
    ClA_rear: Optional[float] = F(None, "rear-axle downforce coefficient x area", "B65 derived", "m^2")
    ref_speed: Optional[float] = F(None, "speed of a measured downforce point", "B65 (35 mph)", "m/s")
    ref_downforce_front: Optional[float] = F(None, "front-axle downforce at ref_speed", "B65", "N")
    ref_downforce_rear: Optional[float] = F(None, "rear-axle downforce at ref_speed", "B65", "N")
    CdA: Optional[float] = F(None, "drag coefficient x area (coast-down fit)", "MISSING / log", "m^2")

    def cla(self):
        """(ClA_front, ClA_rear); derived from the reference downforce point if not given directly."""
        if self.ClA_front is not None and self.ClA_rear is not None:
            return self.ClA_front, self.ClA_rear
        if None in (self.ref_speed, self.ref_downforce_front, self.ref_downforce_rear):
            return None
        q = 0.5 * self.density() * self.ref_speed ** 2
        return self.ref_downforce_front / q, self.ref_downforce_rear / q
    settings: dict = FL(dict, "wing settings: {name: {ClA_front, ClA_rear, CdA}}", "MISSING values", "")


@dc.dataclass
class Powertrain:
    kind: Optional[str] = F(None, "'EV' or 'ICE' (energy reporting only)", "confirm", "")
    max_power: Optional[float] = F(None, "max power at the wheels (or identified from accel log)", "MISSING / log", "W")
    max_drive_force: Optional[float] = F(None, "max tractive force from motor torque (or log)", "MISSING / log", "N")
    torque_curve_file: Optional[str] = F(None, ".csv: rpm, torque_Nm (motor/engine)", "MISSING", "")
    final_drive_ratio: Optional[float] = F(None, "overall ratio motor->wheel", "32T rear sprocket (B49); drive sprocket MISSING", "-")
    drivetrain_efficiency: Optional[float] = F(None, "mechanical efficiency", "MISSING", "-")
    regen_fraction: float = F(0.0, "fraction of braking energy recovered (EV)", "", "-")
    diff: str = F("load_proportional", "'load_proportional' (splits drive by wheel load, notes), 'equal' or 'open'", "notes", "")


@dc.dataclass
class Brakes:
    front_bias: Optional[float] = F(None, "front brake force fraction", "B19", "-")
    max_decel: Optional[float] = F(None, "optional cap on deceleration", "B19 (1.88 g design case)", "m/s^2")
    # brake torque per unit line pressure, per wheel (dynamic tire / braking simulator). Give these
    # directly (or fit them from a log: tire_dynamic.fit_brake_torque), or give the hardware below.
    torque_per_bar_front: Optional[float] = F(None, "brake torque per wheel per unit front line pressure", "log fit / hardware", "N*m/Pa")
    torque_per_bar_rear: Optional[float] = F(None, "brake torque per wheel per unit rear line pressure", "log fit / hardware", "N*m/Pa")
    piston_area_front: Optional[float] = F(None, "caliper piston area acting on ONE pad, front", "MISSING", "m^2")
    piston_area_rear: Optional[float] = F(None, "caliper piston area acting on ONE pad, rear", "MISSING", "m^2")
    pad_mu: Optional[float] = F(None, "pad-rotor friction coefficient", "MISSING", "-")
    rotor_radius_front: Optional[float] = F(None, "effective (pad centre) rotor radius, front", "MISSING", "m")
    rotor_radius_rear: Optional[float] = F(None, "effective (pad centre) rotor radius, rear", "MISSING", "m")
    rear_pressure_ratio: Optional[float] = F(None, "rear/front line pressure slope (balance bar)", "log fit", "-")
    rear_pressure_offset: float = F(0.0, "rear line pressure at zero front pressure", "log fit", "Pa")
    # corner hardware (brakes.py: pad pressure, rotor temperature). Front values measured from the
    # corner CAD with tools/brake_corner.py.
    pad_area_front: Optional[float] = F(None, "friction area of ONE pad, front", "CAD", "m^2")
    pad_area_rear: Optional[float] = F(None, "friction area of ONE pad, rear", "MISSING", "m^2")
    rotor_outer_radius_front: Optional[float] = F(None, "rotor outer radius, front", "CAD", "m")
    rotor_outer_radius_rear: Optional[float] = F(None, "rotor outer radius, rear", "MISSING", "m")
    rotor_thickness_front: Optional[float] = F(None, "rotor thickness, front", "CAD", "m")
    rotor_thickness_rear: Optional[float] = F(None, "rotor thickness, rear", "MISSING", "m")
    rotor_mass_front: Optional[float] = F(None, "mass of one front rotor", "CAD volume x density", "kg")
    rotor_mass_rear: Optional[float] = F(None, "mass of one rear rotor", "MISSING", "kg")
    rotor_area_front: Optional[float] = F(None, "air-cooled surface of one front rotor (both faces + edges)", "CAD", "m^2")
    rotor_area_rear: Optional[float] = F(None, "air-cooled surface of one rear rotor", "MISSING", "m^2")
    rotor_specific_heat: float = F(470.0, "rotor specific heat (steel ~470, grey iron ~500)", "ASSUMED steel", "J/(kg*K)")
    rotor_heat_fraction: float = F(0.90, "share of the friction heat that goes into the rotor (rest: pads, caliper)", "ASSUMED", "-")
    rotor_emissivity: float = F(0.55, "rotor surface emissivity (radiation)", "ASSUMED oxidised steel", "-")
    rotor_h0: Optional[float] = F(None, "rotor convection at standstill", "log fit (brakes.fit_rotor_cooling)", "W/(m^2*K)")
    rotor_h1: Optional[float] = F(None, "rotor convection speed term: h = h0 + h1 * v^0.8", "log fit", "W/(m^2*K) per (m/s)^0.8")
    # pedal box
    master_cylinder_area_front: Optional[float] = F(None, "front master cylinder bore area", "MISSING", "m^2")
    master_cylinder_area_rear: Optional[float] = F(None, "rear master cylinder bore area", "MISSING", "m^2")
    pedal_ratio: Optional[float] = F(None, "pedal mechanical ratio (foot force -> balance bar force)", "MISSING", "-")
    balance_bar_front: Optional[float] = F(None, "share of the balance-bar force on the front master cylinder", "MISSING", "-")

    def torque_gains(self):
        """(front, rear) brake torque per wheel per Pa of line pressure, or None if unknown."""
        if self.torque_per_bar_front is not None and self.torque_per_bar_rear is not None:
            return self.torque_per_bar_front, self.torque_per_bar_rear
        hw = (self.piston_area_front, self.piston_area_rear, self.pad_mu, self.rotor_radius_front, self.rotor_radius_rear)
        if all(v is not None for v in hw):
            af, ar, mu, rf, rr = hw
            return 2 * af * mu * rf, 2 * ar * mu * rr          # two pad faces per rotor
        return None


@dc.dataclass
class Adjust:
    """Current state of the trackside adjustments (relative to the design / hardpoint file)."""
    camber_shim_front: float = F(0.0, "added shim thickness at the upper bracket (+ = more positive camber)", "notes", "m")
    camber_shim_rear: float = F(0.0, "", "notes", "m")
    tie_rod_front: float = F(0.0, "tie-rod length change (toe)", "notes", "m")
    tie_rod_rear: float = F(0.0, "rear toe-link length change", "notes", "m")
    pull_rod_front: float = F(0.0, "pull-rod length change (longer = lower)", "notes", "m")
    pull_rod_rear: float = F(0.0, "", "notes", "m")
    wing_setting: Optional[str] = F(None, "name of an entry in aero.settings", "notes", "")


@dc.dataclass
class Adjustability:
    """What the car can physically be set to. A null range means 'not recorded':
    that feature is left out of the setup optimisation and reported as missing."""
    spring_rates: list = FL(list, "available spring rates", "notes: 150-400 in steps of 50 (unit assumed lbf/in)", "N/m")
    arb_blades: list = FL(list, "available ARB blade lengths", "B43-45", "m")
    arb_can_disconnect: bool = F(False, "may the ARB be disconnected", "not recorded", "")
    shim_thicknesses: list = FL(list, "individual shim thicknesses", "N: 0.02, 0.03 (in)", "m")
    camber_shim_range: Optional[list] = F(None, "[min, max] added shim thickness relative to baseline", "MISSING", "m")
    tie_rod_range: Optional[list] = F(None, "[min, max] tie-rod length change", "MISSING", "m")
    tie_rod_step: Optional[float] = F(None, "tie-rod adjustment step (e.g. thread pitch per turn)", "MISSING", "m")
    pull_rod_range: Optional[list] = F(None, "[min, max] pull-rod length change", "MISSING", "m")
    pull_rod_step: Optional[float] = F(None, "pull-rod adjustment step", "MISSING", "m")
    pressure_range: Optional[list] = F(None, "[min, max] hot pressure", "MISSING (factory minimum)", "Pa")
    pressure_step: Optional[float] = F(None, "pressure adjustment step", "MISSING", "Pa")
    wing_settings: list = FL(list, "names of wing settings in aero.settings", "notes / setup sheets", "")


@dc.dataclass
class Events:
    accel_length: float = F(75.0, "acceleration length (FSAE rules)", "rules", "m")
    skidpad_radius: float = F(9.125, "skidpad driven-line radius (15.25 m inner dia + 1.5 m)", "rules", "m")
    autocross_track: Optional[str] = F(None, ".csv (x,y) or (s,curvature), or a logged .xrk/.xrz/.csv", "MISSING", "")
    endurance_track: Optional[str] = F(None, "as above", "MISSING", "")
    endurance_distance: float = F(22000.0, "endurance total distance (FSAE rules)", "rules", "m")
    # best times for scoring (Tmin). None -> inferred or not scored
    tmin_accel: Optional[float] = F(None, "", "", "s")
    tmin_skidpad: Optional[float] = F(None, "", "", "s")
    tmin_autocross: Optional[float] = F(None, "", "", "s")
    tmin_endurance: Optional[float] = F(None, "", "", "s")


@dc.dataclass
class SimSettings:
    ds: float = F(0.5, "distance step", "", "m")
    v_grid_max: float = F(40.0, "max speed of GGV grid", "", "m/s")
    n_v: int = F(30, "GGV speed points", "", "")
    n_ay: int = F(25, "GGV lateral points", "", "")
    load_transfer_iterations: int = F(6, "fixed-point iterations for load transfer", "", "")
    cornering_drag: bool = F(True, "tire cornering drag (Fy sin(slip angle)) in the resistance; needs "
                             "tires.cornering_stiffness_per_load", "", "")


@dc.dataclass
class CarConfig:
    name: str = "car"
    mass: Mass = dc.field(default_factory=Mass)
    geometry: Geometry = dc.field(default_factory=Geometry)
    tires: Tires = dc.field(default_factory=Tires)
    front: Axle = dc.field(default_factory=Axle)
    rear: Axle = dc.field(default_factory=Axle)
    anti: Anti = dc.field(default_factory=Anti)
    steering: Steering = dc.field(default_factory=Steering)
    aero: Aero = dc.field(default_factory=Aero)
    powertrain: Powertrain = dc.field(default_factory=Powertrain)
    brakes: Brakes = dc.field(default_factory=Brakes)
    events: Events = dc.field(default_factory=Events)
    adjust: Adjust = dc.field(default_factory=Adjust)
    adjustability: Adjustability = dc.field(default_factory=Adjustability)
    sim: SimSettings = dc.field(default_factory=SimSettings)

    # ------------------------------------------------------------------ access
    def get(self, path: str):
        obj = self
        for part in path.split("."):
            obj = getattr(obj, part)
        return obj

    def set(self, path: str, value):
        """Set a value by dotted path, e.g. cfg.set('rear.spring_rate', '275 lbf/in')."""
        *parents, leaf = path.split(".")
        obj = self
        for part in parents:
            obj = getattr(obj, part)
        if not hasattr(obj, leaf):
            raise AttributeError(f"no config field '{path}'")
        setattr(obj, leaf, _convert(leaf, value))
        return self

    def copy(self) -> "CarConfig":
        return copy.deepcopy(self)

    def require(self, *paths: str, purpose: str = ""):
        """Raise MissingDataError naming every listed field that is None."""
        missing = [p for p in paths if self.get(p) is None]
        if missing:
            raise MissingDataError(
                f"{purpose or 'This analysis'} needs: " + ", ".join(missing)
                + ". Add them to the config file or identify them from logged data.")


# ---------------------------------------------------------------------------
def _convert(key: str, value):
    return parse_quantity(value, angle_field=key.endswith("_deg"),
                          accel_field=key in ("max_decel",))


def _fill(dc_obj, data: dict, where: str):
    names = {f.name: f for f in dc.fields(dc_obj)}
    for k, v in (data or {}).items():
        if k not in names:
            raise KeyError(f"unknown config key '{where}.{k}'")
        cur = getattr(dc_obj, k)
        if dc.is_dataclass(cur):
            _fill(cur, v, f"{where}.{k}")
        elif k == "hardpoints" and v is not None:
            setattr(dc_obj, k, {p: [parse_quantity(c) for c in xy] for p, xy in v.items()})
        elif k == "settings":
            setattr(dc_obj, k, {n: {kk: parse_quantity(vv) for kk, vv in (d or {}).items()} for n, d in (v or {}).items()})
        elif k == "components":
            setattr(dc_obj, k, {n: {kk: _convert(kk, vv) for kk, vv in (d or {}).items()} for n, d in (v or {}).items()})
        elif k == "drs":
            setattr(dc_obj, k, {kk: parse_quantity(vv, accel_field=(kk == "max_ay")) for kk, vv in (v or {}).items()})
        elif k == "kt_rates":
            setattr(dc_obj, k, [[parse_quantity(c) for c in row] for row in v])
        else:
            setattr(dc_obj, k, _convert(k, v))


# file paths in the vehicle section, resolved relative to the car file
FILE_FIELDS = [("powertrain", "torque_curve_file"), ("events", "autocross_track"), ("events", "endurance_track"),
               ("front", "hardpoint_file"), ("rear", "hardpoint_file")]


def from_dict(data: dict, base: Path, name: str = "car") -> CarConfig:
    """A CarConfig from the parsed `vehicle:` section; relative file paths are resolved against ``base``."""
    data = dict(data or {})
    cfg = CarConfig()
    cfg.name = data.pop("name", name)
    _fill(cfg, data, "vehicle")
    if cfg.aero.rho is None:                       # density from the measured ambient
        cfg.aero.rho = cfg.aero.density()
    for sec, key in FILE_FIELDS:
        val = getattr(getattr(cfg, sec), key)
        if val and not Path(val).is_absolute():
            setattr(getattr(cfg, sec), key, str((Path(base) / val).resolve()))
    return cfg
