"""Brakes: hardware -> torque and balance, rotor temperature.

Everything reads ``cfg.brakes`` (see config.schema.Brakes). The front corner hardware (rotor, pads,
effective radius) was measured from the corner CAD.

Torque and balance
    torque per wheel = 2 * piston_area * pad_mu * effective_radius * line_pressure
    hardware_summary(cfg)             what the hardware gives next to what a log fit gave
    implied_pad_mu(cfg)               pad friction that makes the hardware match the fitted torque
    force_fraction_front(cfg, p_f)    front share of the brake force the system produces
    ideal_front_fraction(veh, g)      share that uses both axles equally (load transfer + aero)
    balance_table(veh)                the two side by side over deceleration -> which axle locks first
    line_pressure_for(veh, g)         front / rear line pressure for a deceleration

Rotor temperature (one lumped mass per rotor; Biot number ~0.01 for a 5 mm steel disc)
    m c dT/dt = heat_fraction * P_brake - h(v) A (T - T_air) - eps sigma A (T^4 - T_air^4)
    h(v) = h0 + h1 * v^0.8
    wheel_brake_power(cfg, v, p_f, p_r, R)   friction power per front / rear wheel
    rotor_temperature(t, P, v, rotor)        integrate the model
    repeated_stops(veh, v_hi, v_lo, n)       rotor temperature over n identical stops (fade check)
"""
from __future__ import annotations

import dataclasses as dc

import numpy as np

from lapsim.config.schema import MissingDataError
from lapsim.config.units import G0

SIGMA = 5.670374419e-8
_trapz = getattr(np, "trapezoid", None) or np.trapz      # numpy 1.x / 2.x
# density [kg/m^3], specific heat [J/(kg K)] of common rotor materials
ROTOR_MATERIALS = {"steel": (7850.0, 470.0), "stainless_410": (7740.0, 460.0), "grey_iron": (7200.0, 500.0),
                   "titanium_6al4v": (4430.0, 560.0), "aluminium_mmc": (2800.0, 880.0)}


# ============================================================================ hardware
def _gain(area, mu, radius):
    return None if None in (area, mu, radius) else 2.0 * area * mu * radius      # two pad faces per rotor


def implied_pad_mu(cfg):
    """(front, rear) pad friction that makes piston area x radius reproduce the fitted
    brakes.torque_per_bar_*; None where the hardware or the fit is not known."""
    b = cfg.brakes
    out = []
    for k, a, r in ((b.torque_per_bar_front, b.piston_area_front, b.rotor_radius_front),
                    (b.torque_per_bar_rear, b.piston_area_rear, b.rotor_radius_rear)):
        out.append(None if None in (k, a, r) else k / (2.0 * a * r))
    return tuple(out)


def hardware_summary(cfg, p_ref=30e5):
    """Dict of what the brake hardware in the config gives; entries are None where data is missing.
    ``p_ref`` [Pa] is the front line pressure the forces / pad pressures are quoted at."""
    b = cfg.brakes
    hw_f = _gain(b.piston_area_front, b.pad_mu, b.rotor_radius_front)
    hw_r = _gain(b.piston_area_rear, b.pad_mu, b.rotor_radius_rear)
    mu_f, mu_r = implied_pad_mu(cfg)
    gains = b.torque_gains()
    out = {"torque_per_bar_front_fit": None if b.torque_per_bar_front is None else b.torque_per_bar_front * 1e5,
           "torque_per_bar_rear_fit": None if b.torque_per_bar_rear is None else b.torque_per_bar_rear * 1e5,
           "torque_per_bar_front_hardware": None if hw_f is None else hw_f * 1e5,
           "torque_per_bar_rear_hardware": None if hw_r is None else hw_r * 1e5,
           "implied_pad_mu_front": mu_f, "implied_pad_mu_rear": mu_r,
           "clamp_force_front_N": None if b.piston_area_front is None else b.piston_area_front * p_ref,
           "pad_pressure_front_MPa": (None if None in (b.piston_area_front, b.pad_area_front)
                                      else b.piston_area_front * p_ref / b.pad_area_front / 1e6),
           "pad_pressure_rear_MPa": (None if None in (b.piston_area_rear, b.pad_area_rear)
                                     else b.piston_area_rear * p_ref / b.pad_area_rear / 1e6),
           "p_ref_bar": p_ref / 1e5}
    if gains is not None and b.rear_pressure_ratio is not None:
        out["force_fraction_front_at_p_ref"] = force_fraction_front(cfg, p_ref)
    return out


def rear_pressure(cfg, p_f):
    b = cfg.brakes
    if b.rear_pressure_ratio is None:
        raise MissingDataError("rear line pressure needs brakes.rear_pressure_ratio (balance bar / log fit)")
    p_f = np.asarray(p_f, float)
    return np.maximum(b.rear_pressure_ratio * p_f + b.rear_pressure_offset * (p_f > 0), 0.0)


def _gains(cfg):
    g = cfg.brakes.torque_gains()
    if g is None:
        raise MissingDataError("brake torque needs brakes.torque_per_bar_front/rear or the hardware "
                               "(piston_area_*, pad_mu, rotor_radius_*)")
    return g


def force_fraction_front(cfg, p_f):
    """Front share of the total brake force at front line pressure ``p_f`` [Pa] (same tire radius
    front and rear), from the torque gains and the rear/front pressure relation."""
    kf, kr = _gains(cfg)
    p_f = np.asarray(p_f, float)
    tf, tr = kf * p_f, kr * rear_pressure(cfg, p_f)
    return tf / np.maximum(tf + tr, 1e-12)


def ideal_front_fraction(veh, decel_g, v=None):
    """Front share of the brake force that loads both axles' tires equally at ``decel_g``: the front
    share of the vertical load, with load transfer and (when ``v`` [m/s] is given) aero load."""
    a = np.asarray(decel_g, float) * G0
    wf = veh.m * G0 * veh.ff + veh.m * a * veh.h / veh.L
    wr = veh.m * G0 * (1 - veh.ff) - veh.m * a * veh.h / veh.L
    if v is not None:
        ff_, fr_, _ = (np.asarray(x, float) for x in veh.aero(np.asarray(v, float)))
        wf, wr = wf + ff_, wr + fr_
    return wf / (wf + wr)


def line_pressure_for(veh, decel_g, v=0.0, radius=None):
    """(p_front, p_rear) [Pa] that give ``decel_g`` at speed ``v`` (drag helps), wheels rolling."""
    c = veh.cfg
    kf, kr = _gains(c)
    R = radius or c.tires.tire_radius
    drag = float(np.asarray(veh.aero(float(v)))[2]) if v else 0.0
    need = max(veh.m * decel_g * G0 - drag, 0.0)                 # brake force at the ground
    b = c.brakes
    if b.rear_pressure_ratio is None:
        raise MissingDataError("line pressure needs brakes.rear_pressure_ratio")
    # 2 (kf p_f + kr (ratio p_f + offset)) / R = need
    p_f = (need * R / 2.0 - kr * b.rear_pressure_offset) / (kf + kr * b.rear_pressure_ratio)
    p_f = max(p_f, 0.0)
    return p_f, float(rear_pressure(c, p_f))


def balance_table(veh, decel_g=(0.4, 0.8, 1.2, 1.6), v=15.0):
    """Rows of (decel g, front line pressure bar, actual front share, ideal front share, first to lock)."""
    rows = []
    for g in decel_g:
        p_f, _ = line_pressure_for(veh, g, v)
        act = float(force_fraction_front(veh.cfg, p_f))
        ideal = float(ideal_front_fraction(veh, g, v))
        rows.append({"decel_g": g, "p_front_bar": p_f / 1e5, "front_share": act, "ideal_front_share": ideal,
                     "locks_first": "front" if act > ideal else "rear"})
    return rows


# ============================================================================ rotor temperature
@dc.dataclass
class Rotor:
    mass: float                    # kg
    area: float                    # m^2 air-cooled surface
    cp: float = 470.0              # J/(kg K)
    h0: float = 10.0               # W/(m^2 K) at standstill
    h1: float = 9.0                # W/(m^2 K) per (m/s)^0.8
    emissivity: float = 0.55
    heat_fraction: float = 0.90

    def h(self, v):
        return self.h0 + self.h1 * np.abs(v) ** 0.8


# convection used until a log fit is in the config: flat-disc correlations give roughly
# 10 + 9 v^0.8 W/(m^2 K) for a 190 mm rotor inside a 10 in wheel (ASSUMED)
H0_DEFAULT, H1_DEFAULT = 10.0, 9.0


def rotors(cfg):
    """(front, rear) Rotor from the config. A missing rear rotor is taken equal to the front one
    (flagged by ``rear_assumed``)."""
    b = cfg.brakes
    if b.rotor_mass_front is None or b.rotor_area_front is None:
        raise MissingDataError("rotor temperature needs brakes.rotor_mass_front and brakes.rotor_area_front "
                               "(tools/brake_corner.py measures them from the corner CAD)")
    common = dict(cp=b.rotor_specific_heat, emissivity=b.rotor_emissivity, heat_fraction=b.rotor_heat_fraction,
                  h0=H0_DEFAULT if b.rotor_h0 is None else b.rotor_h0, h1=H1_DEFAULT if b.rotor_h1 is None else b.rotor_h1)
    front = Rotor(b.rotor_mass_front, b.rotor_area_front, **common)
    rear = Rotor(b.rotor_mass_rear or b.rotor_mass_front, b.rotor_area_rear or b.rotor_area_front, **common)
    rear.rear_assumed = b.rotor_mass_rear is None or b.rotor_area_rear is None
    return front, rear


def wheel_brake_power(cfg, v, p_f, p_r=None, radius=None):
    """Friction power [W] at ONE front and ONE rear brake: torque x wheel speed, wheels rolling
    (a locked wheel makes no rotor heat). v [m/s], pressures [Pa]."""
    kf, kr = _gains(cfg)
    R = radius or cfg.tires.tire_radius
    v = np.asarray(v, float)
    p_f = np.clip(np.asarray(p_f, float), 0.0, None)
    p_r = rear_pressure(cfg, p_f) if p_r is None else np.clip(np.asarray(p_r, float), 0.0, None)
    w = np.clip(v, 0.0, None) / R
    return kf * p_f * w, kr * p_r * w


def rotor_temperature(t, power, v, rotor: Rotor, T0=None, T_air=25.0):
    """Rotor temperature [C] over time for a friction power trace [W] (one brake) and car speed.
    Semi-implicit in the cooling term, so it is stable at any logging rate."""
    t, power, v = (np.asarray(x, float) for x in (t, power, v))
    power = np.nan_to_num(power)
    v = np.nan_to_num(v)
    T = np.empty(t.size)
    T[0] = T_air if T0 is None else T0
    C = rotor.mass * rotor.cp
    Ta = T_air + 273.15
    for i in range(1, t.size):
        dt = t[i] - t[i - 1]
        Tk = T[i - 1] + 273.15
        hr = rotor.emissivity * SIGMA * (Tk * Tk + Ta * Ta) * (Tk + Ta)       # linearised radiation
        G = (rotor.h(0.5 * (v[i] + v[i - 1])) + hr) * rotor.area
        q = rotor.heat_fraction * 0.5 * (power[i] + power[i - 1])
        T[i] = (T[i - 1] + dt / C * (q + G * T_air)) / (1.0 + dt * G / C)
    return T


def stop_energy(veh, v_hi, v_lo=0.0):
    """Energy one stop puts into ONE front and ONE rear brake [J]: kinetic energy less the drag work
    (drag integrated at the mean deceleration is small; ignored -> conservative), split by the
    brake force share at the mean pressure of a 1 g stop."""
    c = veh.cfg
    e = 0.5 * veh.m * (v_hi ** 2 - v_lo ** 2)
    p_f, _ = line_pressure_for(veh, 1.0, 0.5 * (v_hi + v_lo))
    share = float(force_fraction_front(c, max(p_f, 1e5)))
    return 0.5 * e * share, 0.5 * e * (1 - share)


def repeated_stops(veh, v_hi, v_lo, n=20, decel_g=1.2, accel_g=0.5, T_air=25.0, dt=0.02):
    """Rotor temperatures over ``n`` identical cycles: brake v_hi -> v_lo at ``decel_g``, accelerate
    back at ``accel_g``. Returns a DataFrame t, v, T_front_C, T_rear_C and the per-cycle peaks.
    A quick fade / sizing check: the peaks level off where cooling per cycle equals heating."""
    import pandas as pd
    c = veh.cfg
    front, rear = rotors(c)
    t_b = (v_hi - v_lo) / (decel_g * G0)
    t_a = (v_hi - v_lo) / (accel_g * G0)
    tt = np.arange(0.0, n * (t_b + t_a), dt)
    ph = np.mod(tt, t_b + t_a)
    braking = ph < t_b
    v = np.where(braking, v_hi - decel_g * G0 * ph, v_lo + accel_g * G0 * (ph - t_b))
    p_f = np.array([line_pressure_for(veh, decel_g, float(s))[0] for s in np.linspace(v_lo, v_hi, 40)])
    p_f = np.where(braking, np.interp(v, np.linspace(v_lo, v_hi, 40), p_f), 0.0)
    Pf, Pr = wheel_brake_power(c, v, p_f)
    df = pd.DataFrame({"t": tt, "v": v, "p_front_bar": p_f / 1e5,
                       "T_front_C": rotor_temperature(tt, Pf, v, front, T_air, T_air),
                       "T_rear_C": rotor_temperature(tt, Pr, v, rear, T_air, T_air)})
    cyc = (tt // (t_b + t_a)).astype(int)
    peaks = df.groupby(cyc)[["T_front_C", "T_rear_C"]].max()
    return df, peaks


