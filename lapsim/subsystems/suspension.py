"""Suspension: springs, anti-roll bars, tire vertical stiffness, axle kinematics and steering.

* ``axle_rates``   wheel, ride and roll rates of an axle (springs + ARB in series with the tires);
* ``AxleModel``    an axle's kinematics, solved from the OptimumK hard-point export (3-D linkage,
                   ``subsystems.hardpoints``) or, without a file, from the roll-centre height + FVSA
                   in the car file (linear camber gain);
* ``ackermann_angles`` / ``steer_camber_deg``  steering geometry;
* ``design_report``  the suspension numbers design judges ask about, from the current model.
"""
from __future__ import annotations

import dataclasses as dc
import functools

import numpy as np
import pandas as pd
from scipy import optimize

from lapsim.config.schema import Axle, CarConfig, MissingDataError
from lapsim.config.units import G0, to_unit
from lapsim.subsystems import hardpoints as hp


# --------------------------------------------------------------------------- rates
def mr_damper_per_wheel(ax: Axle) -> float:
    """Motion ratio as damper travel / wheel travel, whatever convention was entered."""
    if ax.motion_ratio is None:
        raise MissingDataError("motion_ratio not set")
    if ax.motion_ratio_convention == "wheel/damper":
        return 1.0 / ax.motion_ratio
    if ax.motion_ratio_convention == "damper/wheel":
        return ax.motion_ratio
    raise ValueError("motion_ratio_convention must be 'wheel/damper' or 'damper/wheel'")


def tire_rate(cfg: CarConfig, pressure: float | None, load: float) -> float:
    """Tire vertical stiffness [N/m], bilinear in the manufacturer table (clamped)."""
    t = cfg.tires
    if not t.kt_rates:
        raise MissingDataError("tires.kt_* stiffness table not set")
    P, Lo, K = np.array(t.kt_pressures), np.array(t.kt_loads), np.array(t.kt_rates)
    p = np.clip(pressure if pressure is not None else P.mean(), P.min(), P.max())
    l = np.clip(load, Lo.min(), Lo.max())
    row = np.array([np.interp(l, Lo, K[i]) for i in range(len(P))])
    return float(np.interp(p, P, row))


def series(k1, k2):
    return k1 * k2 / (k1 + k2)


def arb_tip_stiffness(ax: Axle):
    """Blade-tip stiffness k = G J / (L_torsion L_blade^2) [N/m] (reproduces B43-45)."""
    if None in (ax.arb_blade_length, ax.arb_torsion_length, ax.arb_od, ax.arb_id):
        return None
    J = np.pi * (ax.arb_od ** 4 - ax.arb_id ** 4) / 32
    return ax.arb_G * J / (ax.arb_torsion_length * ax.arb_blade_length ** 2)


def arb_roll_stiffness(ax: Axle, track: float):
    """ARB roll stiffness at the wheels [N*m/rad].

    Opposite wheel travel z gives opposite tip travel d = MR_arb*z at each blade;
    the bar twists 2d/L_blade, so each wheel sees 2*k_tip*MR_arb^2 and the roll
    stiffness is k_tip*MR_arb^2*t^2."""
    if ax.arb_roll_stiffness is not None:
        return np.degrees(1) * ax.arb_roll_stiffness   # given in N*m/deg
    k = arb_tip_stiffness(ax)
    if k is None or ax.arb_motion_ratio is None:
        return None
    return k * ax.arb_motion_ratio ** 2 * track ** 2


@dc.dataclass
class AxleRates:
    wheel_rate: float          # N/m, springs only
    tire_rate: float           # N/m
    ride_rate: float           # N/m, spring in series with tire
    roll_stiffness: float      # N*m/rad incl. ARB and tires
    roll_stiffness_springs: float
    roll_stiffness_arb: float
    sprung_corner_mass: float  # kg
    ride_frequency: float      # Hz
    damping_bump: float | None     # N*s/m at the damper
    damping_rebound: float | None
    notes: list


def axle_rates(cfg: CarConfig, which: str, mr_d: float | None = None, k_arb_override=None) -> AxleRates:
    ax = cfg.front if which == "front" else cfg.rear
    track = cfg.geometry.track_front if which == "front" else cfg.geometry.track_rear
    m = cfg.mass
    frac = m.front_weight_fraction if which == "front" else 1 - m.front_weight_fraction
    notes = []
    corner_mass = m.total * frac / 2
    mu = m.unsprung_mass_front if which == "front" else m.unsprung_mass_rear
    if mu is None:
        notes.append(f"{which}: unsprung mass not recorded - whole corner mass treated as sprung")
        mu = 0.0
    msc = corner_mass - mu
    mrd = mr_d if mr_d is not None else mr_damper_per_wheel(ax)
    if ax.spring_rate is None:
        raise MissingDataError(f"{which}.spring_rate not set")
    kw = ax.spring_rate * mrd ** 2
    pressure = cfg.tires.pressure_front if which == "front" else cfg.tires.pressure_rear
    kt = tire_rate(cfg, pressure, corner_mass * G0)
    kr = series(kw, kt)
    k_springs = kw * track ** 2 / 2
    k_arb = k_arb_override if k_arb_override is not None else arb_roll_stiffness(ax, track)
    if k_arb is None:
        notes.append(f"{which}: ARB roll stiffness unknown (needs arb_blade_length and arb_motion_ratio, "
                     "or arb_roll_stiffness) - ARB omitted")
        k_arb = 0.0
    k_tire_roll = kt * track ** 2 / 2
    k_roll = series(k_springs + k_arb, k_tire_roll)
    f = np.sqrt(kr / msc) / (2 * np.pi)
    c_crit_w = 2 * np.sqrt(kr * msc)
    cb = ax.damping_ratio_bump * c_crit_w / mrd ** 2 if ax.damping_ratio_bump is not None else None
    cr = ax.damping_ratio_rebound * c_crit_w / mrd ** 2 if ax.damping_ratio_rebound is not None else None
    return AxleRates(kw, kt, kr, k_roll, k_springs, k_arb, msc, f, cb, cr, notes)


# --------------------------------------------------------------------------- kinematics
@functools.lru_cache(maxsize=16)
def _load(path):
    return hp.load_optimumk_csv(path)


@functools.lru_cache(maxsize=256)
def _analyse(path, front, d_rod, d_tie, h, L, bias):
    c = hp.Corner(_load(path), front)
    travel = np.linspace(-0.045, 0.045, 31)
    k = hp.analyse_corner(c, travel, h, L, bias, d_rod=d_rod, d_tie=d_tie)
    base = hp.analyse_corner(c, np.array([0.0]), None, None, None) if (d_rod or d_tie) else k
    return c, k, base.static["damper_length_static"]


class AxleModel:
    def __init__(self, cfg: CarConfig, which: str):
        self.which = which
        self.front = which == "front"
        ax = cfg.front if self.front else cfg.rear
        self.ax = ax
        self.track = cfg.geometry.track_front if self.front else cfg.geometry.track_rear
        adj = cfg.adjust
        self.shim = adj.camber_shim_front if self.front else adj.camber_shim_rear
        self.notes = []
        self.source = "hardpoints" if ax.hardpoint_file else "config"
        if ax.hardpoint_file:
            d_rod = round(adj.pull_rod_front if self.front else adj.pull_rod_rear, 7)
            d_tie = round(adj.tie_rod_front if self.front else adj.tie_rod_rear, 7)
            c, k, L0 = _analyse(str(ax.hardpoint_file), self.front, d_rod, d_tie, cfg.mass.cg_height,
                                cfg.geometry.wheelbase, cfg.brakes.front_bias or 0.6)
            self.corner, self.k = c, k
            # static wheel position: same spring length (load) as the baseline pull rod
            if d_rod:
                f = lambda z: np.interp(z, k.travel, k.damper_len) - L0
                self.z_static = float(optimize.brentq(f, k.travel[0], k.travel[-1]))
            else:
                self.z_static = 0.0
            self.body_rise = -self.z_static          # body rises when the wheel sits in droop
            s = k.static
            self.kpi_deg, self.caster_deg = s["kpi_deg"], s["caster_deg"]
            self.scrub, self.trail = s["scrub_radius"], s["mech_trail"]
            dl = np.gradient(k.damper_len, k.travel)
            self.mr_d = float(-np.interp(self.z_static, k.travel, dl))       # damper shortening per wheel bump
            self.rod_angle_deg = s["pull_rod_angle_from_vertical_deg"]
            self.fvsa = s["fvsa"]
            self.blade_rate = (float(np.interp(self.z_static, k.travel, np.gradient(k.blade, k.travel)))
                               if k.blade is not None else None)
            self.blade_len_file = s.get("blade_length_in_file")
            self.anti = {kk: s[kk] for kk in ("anti_dive", "anti_lift", "anti_squat") if kk in s}
            self.toe_static = float(np.interp(self.z_static, k.travel, k.toe))
            if not ax.use_hardpoint_rc and ax.roll_center_height is not None:
                off = ax.roll_center_height - float(np.interp(0.0, k.travel, k.rc_height))
                self._rc = lambda z: np.interp(z, k.travel, k.rc_height) + off
                self.notes.append(f"{which}: roll-centre curve shifted to the configured static height")
            else:
                self._rc = lambda z: np.interp(z, k.travel, k.rc_height)
        else:
            if ax.roll_center_height is None or ax.fvsa_length is None:
                raise MissingDataError(f"{which}: give hardpoint_file, or roll_center_height and fvsa_length")
            self.corner = self.k = None
            self.z_static = self.body_rise = 0.0
            self.kpi_deg, self.caster_deg = ax.kpi_deg or 0.0, ax.caster_deg or 0.0
            self.scrub, self.trail = ax.scrub_radius, ax.mechanical_trail
            self.mr_d = None
            self.rod_angle_deg = ax.rod_angle_deg
            self.fvsa = ax.fvsa_length
            self.blade_rate = None
            self.blade_len_file = None
            self.anti = {}
            self.toe_static = ax.static_toe_deg
            rc0 = ax.roll_center_height
            self._rc = lambda z: np.full(np.shape(z), rc0, float) if np.ndim(z) else rc0

    # -------------------------------------------------------------- curves
    def rc(self, heave):
        return self._rc(self.z_static + np.asarray(heave, float))

    def dcamber(self, z):
        """Camber change relative to the body [deg] (+ top outboard) at wheel travel z (+ bump)."""
        if self.k is None:
            return np.degrees(-np.asarray(z, float) / self.fvsa)
        return np.interp(self.z_static + np.asarray(z, float), self.k.travel, self.k.camber) - \
            np.interp(self.z_static, self.k.travel, self.k.camber)

    def static_camber(self):
        c0 = self.ax.static_camber_deg or 0.0
        sh = hp.camber_per_shim(self.corner, self.shim) if self.corner is not None and self.shim else \
            (np.degrees(np.arctan2(self.shim, 0.2034)) if self.shim else 0.0)
        ride = (np.interp(self.z_static, self.k.travel, self.k.camber) if self.k is not None else 0.0)
        return c0 + sh + ride

    def toe(self, z=0.0):
        if self.k is None:
            return self.toe_static
        return np.interp(self.z_static + np.asarray(z, float), self.k.travel, self.k.toe)

    # -------------------------------------------------------------- ARB
    def arb_roll_stiffness(self):
        """ARB roll stiffness [N*m/rad] from the hardpoint drop-link geometry and the
        torsion bar: K = (G J / L_t) * k^2 * t^2, with k = blade rotation per wheel travel
        scaled by (blade length in the file / selected blade length)."""
        ax = self.ax
        if ax.arb_roll_stiffness is not None:
            return np.degrees(1) * ax.arb_roll_stiffness
        if self.blade_rate is None or None in (ax.arb_torsion_length, ax.arb_od, ax.arb_id):
            return None
        Lb = ax.arb_blade_length
        if Lb is None:
            Lb = self.blade_len_file
            self.notes.append(f"{self.which}: ARB blade length not set - using the {Lb / 0.0254:.2f} in "
                              "blade implied by the hardpoint file")
        if Lb == 0:
            return 0.0     # disconnected
        J = np.pi * (ax.arb_od ** 4 - ax.arb_id ** 4) / 32
        k = self.blade_rate * self.blade_len_file / Lb
        return ax.arb_G * J / ax.arb_torsion_length * k ** 2 * self.track ** 2

    def summary(self):
        d = {"source": self.source, "kpi_deg": self.kpi_deg, "caster_deg": self.caster_deg,
             "scrub_radius_mm": (self.scrub or np.nan) * 1000, "mech_trail_mm": (self.trail or np.nan) * 1000,
             "rc_static_in": float(self.rc(0.0)) / 0.0254, "fvsa_in": self.fvsa / 0.0254,
             "static_camber_deg": self.static_camber(), "static_toe_deg": float(self.toe(0.0)),
             "body_rise_mm": self.body_rise * 1000}
        if self.mr_d:
            d["motion_ratio_wheel_per_damper"] = 1 / self.mr_d
        if self.rod_angle_deg is not None:
            d["rod_angle_deg"] = self.rod_angle_deg
        d.update({k: v for k, v in self.anti.items()})
        return d


# --------------------------------------------------------------------------- steering
def ackermann_angles(R, L, t, fraction):
    """Inner/outer road-wheel angles [rad] at turn radius R for an Ackermann fraction."""
    do = np.arctan(L / (R + t / 2))
    di_ideal = np.arctan(L / (R - t / 2))
    di = do + fraction * (di_ideal - do)
    return di, do


def steer_camber_deg(delta_rad, caster_deg, kpi_deg):
    """Turn-relative camber change of the (outside, inside) front wheel when steered by delta.
    Caster leans both tops toward the turn (helpful, negative); KPI leans both tops
    outward relative to the car, which hurts the outside wheel and helps the inside."""
    nu, sig = np.radians(caster_deg), np.radians(kpi_deg)
    cas = -np.degrees(nu * np.sin(delta_rad))
    kpi = np.degrees(sig * (1 - np.cos(delta_rad)))
    return cas + kpi, cas - kpi


# --------------------------------------------------------------------------- report
def design_report(veh):
    """Suspension numbers design judges ask about, from the current model."""
    c = veh.cfg
    rows = {}
    for nm, A, R, t in (("front", veh.Af, veh.Rf, veh.tf), ("rear", veh.Ar, veh.Rr, veh.tr)):
        s = A.summary()
        rows[nm] = {
            "kinematics source": s["source"], "roll-centre height [in]": s["rc_static_in"],
            "front-view swing arm [in]": s["fvsa_in"], "static camber [deg]": s["static_camber_deg"],
            "camber gain [deg/in bump]": float(A.dcamber(0.0254) - A.dcamber(0.0)),
            "KPI [deg]": s["kpi_deg"], "caster [deg]": s["caster_deg"],
            "scrub radius [mm]": s["scrub_radius_mm"], "mechanical trail [mm]": s["mech_trail_mm"],
            "motion ratio wheel/damper": s.get("motion_ratio_wheel_per_damper", 1 / (veh.mrf if nm == "front" else veh.mrr)),
            "pull-rod angle [deg]": s.get("rod_angle_deg", np.nan),
            "wheel rate [lbf/in]": to_unit(R.wheel_rate, "lbf/in"), "ride rate [lbf/in]": to_unit(R.ride_rate, "lbf/in"),
            "ride frequency [Hz]": R.ride_frequency,
            "roll stiffness springs [N*m/deg]": R.roll_stiffness_springs / 57.2958,
            "roll stiffness ARB [N*m/deg]": R.roll_stiffness_arb / 57.2958,
            "roll stiffness total incl. tire [N*m/deg]": R.roll_stiffness / 57.2958,
        }
    df = pd.DataFrame(rows)
    ay1 = veh.state(15.0, 0.0, G0)
    extra = {
        "roll gradient [deg/g]": float(ay1["roll_deg"]),
        "front roll-stiffness share [%]": 100 * veh.Kf / (veh.Kf + veh.Kr),
        "front lateral load-transfer share at 1 g [%]": float(100 * ay1["lt_front"] * veh.tf /
                                                              (ay1["lt_front"] * veh.tf + ay1["lt_rear"] * veh.tr)),
        "anti-dive [%]": 100 * veh.anti["anti_dive"], "anti-lift [%]": 100 * veh.anti["anti_lift"],
        "anti-squat [%]": 100 * veh.anti["anti_squat"], "CG height used [in]": veh.h / 0.0254,
    }
    for v in (10, 20, 30):
        aym, mf, mr = veh.limit_balance(np.array([float(v)]))
        extra[f"lateral limit at {v} m/s [g]"] = float(aym[0] / G0)
        extra[f"limit balance at {v} m/s"] = "front-limited (understeer)" if mf[0] < mr[0] else "rear-limited (oversteer)"
    return df, pd.Series(extra, dtype=object)
