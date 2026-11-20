"""Driveline: the rear differential (both cars) and the IC car's driveline from crank to contact patch.

crank --primary gear (85/41)--> clutch --gear k--> countershaft sprocket
--520 chain--> rear sprocket / Drexler diff --> 2 tripod CV halfshafts
--> hubs / bearings --> rear tires.

Every stage has a load-dependent efficiency and, where it matters, a
speed-dependent spin loss, so the losses per stage can be reported. Rotating
inertia of every part is reflected to the vehicle as an equivalent mass per gear.
"""
from __future__ import annotations

import dataclasses as dc

import numpy as np

from lapsim.config.schema_ic import PTConfig


def omega(rpm):
    """rpm -> rad/s"""
    return np.asarray(rpm, float) * 2 * np.pi / 60

STAGES = ("primary", "gearbox", "chain", "differential", "cv_joints", "hubs")


# --------------------------------------------------------------------------- diff
class Differential:
    """Ramp-and-clutch-pack limited-slip differential (Drexler FSAE V3) and the
    reference cases open / spool / ideal (load-proportional).

    Locking:  dT = preload + L * |T_in|     with   L = n*mu*r_f / (2*r_ramp*tan(ramp))
    (torque difference between the two outputs the plate pack can hold);
    torque bias ratio  TBR = (1 + L) / (1 - L)."""

    def __init__(self, pt: PTConfig):
        self.c = pt.differential
        self.type = self.c.type.lower()
        self.k_slip = pt.wheels.slip_stiffness_per_load

    def locking_coef(self, coast=False):
        c = self.c
        if self.type == "open":
            return 0.0
        if self.type in ("spool", "ideal"):
            return np.inf
        th = np.radians(c.coast_ramp_deg if coast else c.drive_ramp_deg)
        if c.lockup_at_45deg is not None:            # Drexler's published lock-up (PT45)
            return float(np.clip(c.lockup_at_45deg / np.tan(th), 0.0, 0.95))
        return float(np.clip(c.friction_faces * c.plate_mu * c.friction_radius / (2 * c.ramp_radius * np.tan(th)),
                             0.0, 0.95))

    def preload(self):
        return 0.0 if self.type == "open" else (np.inf if self.type == "spool" else self.c.preload)

    def lock_torque(self, t_in, coast=False):
        """Largest torque difference between the outputs [N m] for axle input torque t_in."""
        if self.type in ("spool", "ideal"):
            return np.full(np.shape(t_in), np.inf)
        return self.preload() + self.locking_coef(coast) * np.abs(np.asarray(t_in, float))

    def split(self, D, cap_i, cap_o, fz_i, fz_o, kappa_t, r):
        """Drive force split for total rear drive D [N] (vectorised).

        cap_*: peak longitudinal force of the inside/outside tire; kappa_t = track * |path curvature|
        (relative wheel-speed difference in the turn). Returns (F_inside, F_outside, feasible).
        A locked axle drives the slower (inside) wheel harder by tire slip stiffness * kappa_t,
        up to the locking limit; once the inside tire saturates it spins up and the bias flips
        to the outside wheel, up to the same limit."""
        D = np.asarray(D, float)
        if self.type == "ideal":
            tot = np.maximum(fz_i + fz_o, 1e-6)
            Fi, Fo = D * fz_i / tot, D * fz_o / tot
            return Fi, Fo, (Fi <= cap_i * 1.0001) & (Fo <= cap_o * 1.0001)
        dF_lock = self.lock_torque(D * r) / r
        if self.type == "open":
            Fi = Fo = D / 2
        else:
            Fi_k, Fo_k = self.locked_split(D, cap_i, cap_o, fz_i, fz_o, kappa_t)
            dk = Fi_k - Fo_k                      # > 0: slower inside wheel drives harder
            dF = np.sign(dk) * np.minimum(np.abs(dk), dF_lock)        # plate pack slips beyond the lock limit
            Fi, Fo = D / 2 + dF / 2, D / 2 - dF / 2
        sat = Fi > cap_i
        Fi = np.where(sat, cap_i, Fi)
        Fo = np.where(sat, D - cap_i, Fo)
        ok = (np.abs(Fo) <= cap_o * 1.0001) & (Fo - Fi <= dF_lock * 1.0001 + 1e-9) & (Fi <= cap_i * 1.0001)
        return Fi, Fo, ok

    def locked_split(self, D, cap_i, cap_o, fz_i, fz_o, kappa_t, n_iter=28):
        """Forces of a locked axle (both wheels at one speed): the inside wheel runs kappa_t/2
        more slip than the mean, the outside kappa_t/2 less; each tire F = cap*tanh(k Fz kappa / cap).
        The mean slip is found so that the two forces add up to D."""
        cap_i = np.maximum(np.asarray(cap_i, float), 1.0)
        cap_o = np.maximum(np.asarray(cap_o, float), 1.0)
        ki, ko = self.k_slip * np.asarray(fz_i, float), self.k_slip * np.asarray(fz_o, float)
        F = lambda km: (cap_i * np.tanh(ki * (km + kappa_t / 2) / cap_i), cap_o * np.tanh(ko * (km - kappa_t / 2) / cap_o))
        D = np.broadcast_to(np.asarray(D, float), np.broadcast(cap_i, kappa_t).shape)
        lo, hi = np.full(D.shape, -1.0), np.full(D.shape, 1.0)
        for _ in range(n_iter):
            mid = 0.5 * (lo + hi)
            fi, fo = F(mid)
            low = fi + fo < D
            lo, hi = np.where(low, mid, lo), np.where(low, hi, mid)
        return F(0.5 * (lo + hi))

# --------------------------------------------------------------------------- driveline
@dc.dataclass
class StageLosses:
    power_in: np.ndarray
    losses: dict          # stage -> W
    power_out: np.ndarray


class Drivetrain:
    def __init__(self, pt: PTConfig, tire_radius: float, engine, electrical):
        self.pt = pt
        self.eng, self.elec = engine, electrical      # power.ic.Engine, power.ic.Electrical
        self.diff = Differential(pt)
        g, f = pt.gearbox, pt.final_drive
        self.r_unloaded = tire_radius
        self.r = tire_radius * pt.wheels.rolling_radius_factor
        self.primary = g.primary_ratio
        self.gears = np.asarray(g.gear_ratios, float)
        self.n_gears = len(self.gears)
        self.fd = f.rear_sprocket / f.front_sprocket
        self.overall = self.primary * self.gears * self.fd

    # ------------------------------------------------------------ kinematics
    def rpm(self, v, gear):
        """Engine rpm at speed v in gear (1-based, array ok)."""
        k = np.asarray(gear, int) - 1
        return np.asarray(v, float) / self.r * self.overall[k] * 60 / (2 * np.pi)

    def v_at(self, rpm, gear):
        return np.asarray(rpm, float) * 2 * np.pi / 60 * self.r / self.overall[np.asarray(gear, int) - 1]

    def top_speed_in_gear(self, gear=None):
        rl = self.pt.engine.rev_limit
        return self.v_at(rl, np.arange(1, self.n_gears + 1) if gear is None else gear)

    # ------------------------------------------------------------ efficiencies
    def eta_chain(self):
        f = self.pt.final_drive
        return 1.0 - f.chain_artic_coef * (1.0 / f.front_sprocket + 1.0 / f.rear_sprocket)

    def eta_cv(self):
        h = self.pt.halfshaft
        return max(h.cv_efficiency - h.cv_loss_per_deg * h.cv_angle_deg, 0.5) ** 2   # two joints per shaft

    def transmit(self, T_crank, rpm, gear):
        """Crank torque -> torque at both rear contact patches, with the loss in every stage [W]."""
        T = np.asarray(T_crank, float)
        rpm = np.asarray(rpm, float)
        k = np.asarray(gear, int) - 1
        g, w = self.pt.gearbox, omega(rpm)
        ls = g.loss_scale
        eff = lambda e: 1.0 - ls * (1.0 - e)
        P_in = T * w
        L = {}
        # primary
        ep = eff(g.primary_efficiency)
        T1 = T * self.primary * np.where(T >= 0, ep, 1 / ep)
        w1 = w / self.primary
        L["primary"] = P_in - T1 * w1
        # gearbox: one mesh + spin losses on the output
        w2 = w1 / self.gears[k]
        spin = ls * (g.spin_loss_a + g.spin_loss_b * w2)
        em = eff(g.mesh_efficiency)
        T2 = T1 * self.gears[k] * np.where(T >= 0, em, 1 / em) - spin
        L["gearbox"] = T1 * w1 - T2 * w2
        # chain
        ec = eff(self.eta_chain())
        w3 = w2 / self.fd
        T3 = T2 * self.fd * np.where(T2 >= 0, ec, 1 / ec)
        L["chain"] = T2 * w2 - T3 * w3
        # differential (plate slip losses added separately in cornering)
        ed = eff(self.pt.differential.efficiency)
        T4 = T3 * np.where(T3 >= 0, ed, 1 / ed)
        L["differential"] = (T3 - T4) * w3
        # CV joints
        ecv = eff(self.eta_cv())
        T5 = T4 * np.where(T4 >= 0, ecv, 1 / ecv)
        L["cv_joints"] = (T4 - T5) * w3
        # hubs / bearings (two rear wheels)
        T6 = T5 - 2 * ls * self.pt.wheels.bearing_drag
        L["hubs"] = (T5 - T6) * w3
        return T6, StageLosses(P_in, L, T6 * w3)

    # ------------------------------------------------------------ engine at the crank
    def crank_torque_wot(self, rpm, coolant_temp=None, fan_on=False):
        """WOT brake torque minus alternator drag (electrical load incl. fan)."""
        rpm = np.asarray(rpm, float)
        p_el = self.elec.output(rpm, self.elec.load(fan_on))
        return self.eng.torque_wot(rpm, coolant_temp) - self.elec.drag_torque(rpm, p_el)

    def wheel_force_wot(self, v, gear, coolant_temp=None, fan_on=False, launch=True):
        """Tractive force at the rear contact patches at full throttle in ``gear`` [N].
        Below the launch rpm in first gear the clutch slips with the engine held at launch_rpm."""
        v = np.asarray(v, float)
        e = self.pt.engine
        rpm = self.rpm(v, gear)
        gear_a = np.broadcast_to(np.asarray(gear), v.shape)
        slip = launch & (gear_a == 1) & (rpm < e.launch_rpm)
        rpm_e = np.where(slip, e.launch_rpm, rpm)
        Tc = self.crank_torque_wot(rpm_e, coolant_temp, fan_on)
        Tc = np.minimum(Tc, self.pt.clutch.torque_capacity)
        T6, _ = self.transmit(Tc, rpm, gear_a)
        F = T6 / self.r
        ok = (rpm <= e.rev_limit * 1.0005) & ((rpm >= e.min_drive_rpm) | slip)
        return np.where(ok, np.maximum(F, 0.0), 0.0)

    def forces_all_gears(self, v, coolant_temp=None, fan_on=False):
        v = np.atleast_1d(np.asarray(v, float))
        return np.stack([self.wheel_force_wot(v, k, coolant_temp, fan_on) for k in range(1, self.n_gears + 1)])

    # ------------------------------------------------------------ inertia
    def rotating_inertia_at_wheel(self, gear):
        """Sum of I*(ratio to wheel)^2 of every rotating part, at the rear axle [kg m^2]."""
        p = self.pt
        k = np.asarray(gear, int) - 1
        I_axle = (2 * p.wheels.rear_corner_inertia + 2 * p.halfshaft.inertia + p.differential.inertia
                  + p.final_drive.sprocket_inertia)
        I_front = 2 * p.wheels.front_corner_inertia            # spun up by the road, same wheel speed
        I_out = p.gearbox.output_inertia * self.fd ** 2
        I_cl = p.clutch.inertia * (self.gears[k] * self.fd) ** 2
        I_cr = p.engine.crank_inertia * self.overall[k] ** 2
        return {"wheels_axle": I_axle + I_front, "gearbox_output": I_out, "clutch": I_cl, "crank": I_cr}

    def equivalent_mass(self, gear, m, clutch_slipping=False):
        """Vehicle mass + rotating inertia reflected to the contact patch. While the clutch slips
        (launch) the crank and clutch basket are not accelerated with the car."""
        parts = self.rotating_inertia_at_wheel(gear)
        if clutch_slipping:
            parts = {k: v for k, v in parts.items() if k not in ("crank", "clutch")}
        return m + sum(parts.values()) / self.r ** 2

    def launch_lockup_speed(self):
        return float(self.v_at(self.pt.engine.launch_rpm, 1))

    # ------------------------------------------------------------ shifting
    def optimal_shift_rpm(self, m, resist=lambda v: 0.0, coolant_temp=None):
        """Upshift rpm per gear that maximises acceleration (crossing of the
        acceleration curves incl. rotating inertia), capped at the rev limit."""
        rl = self.pt.engine.rev_limit
        out = []
        for k in range(1, self.n_gears):
            v = np.linspace(self.v_at(self.pt.engine.min_drive_rpm, k), self.v_at(rl, k), 400)
            a1 = (self.wheel_force_wot(v, k, coolant_temp, launch=False) - resist(v)) / self.equivalent_mass(k, m)
            a2 = (self.wheel_force_wot(v, k + 1, coolant_temp, launch=False) - resist(v)) / self.equivalent_mass(k + 1, m)
            better = np.flatnonzero((a2 > a1) & (self.rpm(v, k + 1) >= self.pt.engine.min_drive_rpm))
            out.append(float(self.rpm(v[better[0]], k)) if better.size else rl)
        return out

    def shift_rpms(self, m, resist=lambda v: 0.0):
        s = self.pt.gearbox.shift_rpm
        if s is None:
            return self.optimal_shift_rpm(m, resist)
        return [min(float(s), self.pt.engine.rev_limit)] * (self.n_gears - 1)

    # ------------------------------------------------------------ chain & shafts
    def sprocket_pitch_radius(self, z):
        return self.pt.final_drive.chain_pitch / (2 * np.sin(np.pi / np.asarray(z, float)))

    def chain_tension(self, T_countershaft):
        return np.asarray(T_countershaft, float) / self.sprocket_pitch_radius(self.pt.final_drive.front_sprocket)

    def chain_links(self, centre=None):
        f = self.pt.final_drive
        C = centre or f.centre_distance
        p, z1, z2 = f.chain_pitch, f.front_sprocket, f.rear_sprocket
        n = 2 * C / p + (z1 + z2) / 2 + ((z2 - z1) / (2 * np.pi)) ** 2 * p / C
        return float(n)

    def halfshaft_stress(self, T_shaft):
        h = self.pt.halfshaft
        J = np.pi * (h.od ** 4 - h.id ** 4) / 32
        return np.asarray(T_shaft, float) * (h.od / 2) / J

    def summary(self, m):
        rows = []
        for k in range(1, self.n_gears + 1):
            rows.append({"gear": k, "ratio": self.gears[k - 1], "overall": self.overall[k - 1],
                         "v_at_rev_limit_kph": 3.6 * float(self.top_speed_in_gear(k)),
                         "equivalent_mass_kg": self.equivalent_mass(k, m),
                         "rotating_mass_%": 100 * (self.equivalent_mass(k, m) / m - 1)})
        return rows
