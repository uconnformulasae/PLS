"""Aerodynamics: attitude-sensitive component model of the aero package (front wing, rear wing, body).

The car is split into three aerodynamic components (front wing, rear wing,
body = chassis + sidepods + wheels + driver + radiator). Each has a downforce
and drag at the reference attitude (the binder's 3D CFD split, scaled to the
coast-down measurement), a centre of pressure (x, z) and a span. At any
operating point the model takes the chassis attitude from the suspension model

    zf, zr   body drop at the front / rear axle (+ = lower), so heave and pitch
    phi      roll (+ = body leans to the outside of the turn)
    beta     body slip angle at the CG (+ = velocity points into the turn)
    kappa    path curvature (yaw rate / speed)

and returns the forces and moments on the car:

  * ground effect - the front wing works close to the ground; its downforce
    follows CL(h/c) (published single-element shape, Zerihan & Zhang 2000,
    replaceable by a CFD ride-height sweep). Heave and pitch move the wing.
  * pitch - nose-down pitch raises every wing's angle of attack
    (dClA/dalpha, saturating beyond alpha_lin as the elements stall).
  * roll - each wing is two halves at y = +/- span/4. Roll moves one half
    toward the ground and the other away (front wing in ground effect), and
    in a turn the outer half travels faster (v (1 + y kappa)): both shift
    downforce to the outside and give an aero rolling moment.
  * yaw - every component sees its own flow angle
    beta_i = beta + x_i kappa (x_i from the CG): the front wing sees the flow
    from inside the turn, the rear wing from outside (curved flow). Downforce
    falls and drag rises with the square of the local angle; endplates and
    body sides give a side force and an aero yaw moment.
  * drag - per component, with an induced part that follows the downforce,
    acting at its own height: the drag pitching moment unloads the front axle.
  * DRS (optional, not on CT-17) - rear wing flap opened on straights.

The front/rear axle split includes the drag moment:
    F_front = (sum F_i (x_i + L) - sum D_i z_i) / L,   x_i from the front axle.

Everything is vectorised; ``AeroModel.evaluate`` is called inside the
vehicle state's fixed-point loop (aero <-> ride height <-> slip angle).
"""
from __future__ import annotations

import dataclasses as dc

import numpy as np
import pandas as pd

from lapsim.config.schema import CarConfig, MissingDataError
from lapsim.config.units import G0

# Front wing CL / CL(max) versus ground clearance / chord. Shape after Zerihan & Zhang
# (2000), single-element wing in ground effect with a moving ground: downforce rises as the
# wing approaches the ground, peaks near h/c = 0.1 and drops below it (boundary-layer
# separation on the suction side). Replace with a CFD ride-height sweep when available
# (aero.ground_effect: [[h/c, factor], ...]).
GROUND_EFFECT_DEFAULT = np.array([
    [0.040, 0.70], [0.056, 0.77], [0.067, 0.84], [0.079, 0.91], [0.090, 0.96], [0.112, 1.00],
    [0.134, 0.99], [0.179, 0.94], [0.224, 0.88], [0.313, 0.80], [0.448, 0.71], [0.671, 0.63],
    [1.000, 0.58], [2.000, 0.54], [10.00, 0.52]])

COMPONENTS = ("front_wing", "rear_wing", "body")
DEG = np.pi / 180


@dc.dataclass
class Component:
    name: str
    ClA: float                 # downforce coefficient x area at the reference attitude [m^2] (+ = down)
    CdA: float                 # [m^2]
    x: float | None            # centre of pressure, from the front axle, + forward [m]
    z: float                   # height of the drag / side-force line of action [m]
    span: float                # [m]
    chord: float | None = None              # main-element chord (ground effect) [m]
    ground_clearance: float | None = 0.0508 # static clearance of the lowest element [m]; None = no ground effect
    dClA_dalpha: float = 0.0                # per degree of nose-down pitch [m^2/deg]
    alpha_lin_deg: float = 3.0              # pitch range of the linear slope (saturates beyond)
    dClA_dh: float = 0.0                    # extra ride-height sensitivity [m^2/m], + = more downforce when lower
    yaw_loss_10deg: float = 0.0             # fraction of downforce lost at 10 deg local flow angle
    yaw_drag_10deg: float = 0.0             # fractional drag rise at 10 deg
    side_area: float = 0.0                  # side-force slope CyA [m^2/rad]
    induced_drag: float = 0.0               # fraction of CdA that scales with (ClA/ClA_ref)^2
    mass: float = 0.0                       # [kg] (wings-off comparisons, inertia)
    drs_cl_ratio: float | None = None       # ClA open / closed (rear wing only)
    drs_cd_ratio: float | None = None
    cfd_lift: float | None = None           # binder CFD split [N] (sets the shares before calibration)
    cfd_drag: float | None = None

    @classmethod
    def from_dict(cls, name, d: dict):
        known = {f.name for f in dc.fields(cls)}
        bad = set(d) - known - {"cfd_downforce", "x_from_front_axle"}
        if bad:
            raise KeyError(f"aero.components.{name}: unknown keys {sorted(bad)}")
        kw = {k: v for k, v in d.items() if k in known}
        if "cfd_downforce" in d:
            kw["cfd_lift"] = d["cfd_downforce"]
        if "x_from_front_axle" in d:
            kw["x"] = d["x_from_front_axle"]
        kw.setdefault("ClA", 0.0)
        kw.setdefault("CdA", 0.0)
        kw.setdefault("z", 0.3)
        kw.setdefault("span", 1.0)
        kw.setdefault("x", None)
        return cls(name=name, **kw)


class AeroModel:
    """Component aero model. ``L`` wheelbase, ``a`` CG to front axle."""

    def __init__(self, comps: list[Component], L: float, a: float, rho: float, ground_effect=None,
                 drs: dict | None = None, constant: bool = False):
        self.comps = comps
        self.L, self.a, self.rho = L, a, rho
        ge = np.asarray(ground_effect if ground_effect is not None else GROUND_EFFECT_DEFAULT, float)
        self.ge_hc, self.ge_f = ge[:, 0], ge[:, 1]
        self.drs = dict(drs or {})
        self.constant = constant            # True: no attitude sensitivity (original model)
        self.notes = []

    # ------------------------------------------------------------------ construction
    @classmethod
    def from_config(cls, cfg: CarConfig, L: float, a: float):
        A = cfg.aero
        rho = A.density()
        comps = []
        if A.model == "constant" or not A.components:
            # original model: two point loads at the axles, drag at the CG height (no moment)
            cla = A.cla() or (0.0, 0.0)
            cda = A.CdA or 0.0
            comps = [Component("front_axle", cla[0], cda * 0.5, 0.0, cfg.mass.cg_height or 0.3, 1.0),
                     Component("rear_axle", cla[1], cda * 0.5, -L, cfg.mass.cg_height or 0.3, 1.0)]
            m = cls(comps, L, a, rho, constant=True)
            m.notes.append("aero: constant model (no ride-height / pitch / roll / yaw sensitivity)")
            return m
        for name, d in A.components.items():
            comps.append(Component.from_dict(name, d))
        m = cls(comps, L, a, rho, A.ground_effect, A.drs)
        # shares from the CFD split when ClA / CdA are not given directly
        if all(c.cfd_lift is not None for c in comps):
            tl = sum(c.cfd_lift for c in comps)
            td = sum(c.cfd_drag for c in comps)
            tgt_l = A.calib_ClA if A.calib_ClA is not None else tl
            tgt_d = A.calib_CdA if A.calib_CdA is not None else td
            for c in comps:
                c.ClA = c.cfd_lift / tl * tgt_l
                c.CdA = c.cfd_drag / td * tgt_d
        # centre of pressure: solve the one unknown x (normally the body) from the measured balance
        unk = [c for c in comps if c.x is None]
        if len(unk) > 1:
            raise MissingDataError("aero.components: only one component may have x = null "
                                   f"(solved from calib_front_fraction); got {[c.name for c in unk]}")
        if unk:
            share = getattr(A, "calib_front_axle_share", None)
            if A.calib_front_fraction is None and share is None:
                raise MissingDataError(f"aero.components.{unk[0].name}.x is null: give it, or "
                                       "aero.calib_front_fraction / aero.calib_front_axle_share to solve it")
            c0 = unk[0]
            tot = sum(c.ClA for c in comps)
            known = sum(c.ClA * (c.x + L) for c in comps if c is not c0)
            if abs(c0.ClA) < 1e-9:
                raise MissingDataError(f"aero: cannot place {c0.name} (zero downforce)")
            if A.calib_front_fraction is not None:
                # centre of pressure of the downforce alone
                c0.x = (A.calib_front_fraction * tot * L - known) / c0.ClA - L
                m.notes.append(f"aero: {c0.name} centre of pressure solved at x = {c0.x:+.3f} m from the front axle "
                               f"to give {A.calib_front_fraction:.1%} front balance")
            else:
                # measured axle loads: drag at height z moves D z / L from the front axle to the rear,
                # so the downforce has to sit further forward than the measured split
                drag_moment = sum(c.CdA * c.z for c in comps)
                c0.x = (share * tot * L + drag_moment - known) / c0.ClA - L
                m.notes.append(f"aero: {c0.name} centre of pressure solved at x = {c0.x:+.3f} m from the front axle "
                               f"to give {share:.1%} of the aero load on the front axle (drag transfer included; "
                               f"downforce centre of pressure {(share * tot * L + drag_moment) / (tot * L):.1%})")
        return m

    def scale(self, k_lift=1.0, k_drag=1.0):
        for c in self.comps:
            c.ClA *= k_lift
            c.dClA_dalpha *= k_lift
            c.dClA_dh *= k_lift
            c.CdA *= k_drag

    def ground_factor(self, h, c: Component):
        """CL(h) / CL(h_static) for a component in ground effect."""
        if c.ground_clearance is None or not c.chord:
            return np.ones_like(np.asarray(h, float))
        hc = np.maximum(np.asarray(h, float), 1e-3) / c.chord
        f = np.where(hc < self.ge_hc[0], self.ge_f[0] * hc / self.ge_hc[0], np.interp(hc, self.ge_hc, self.ge_f))
        f0 = np.interp(c.ground_clearance / c.chord, self.ge_hc, self.ge_f)
        return f / f0

    @staticmethod
    def _sat(x, lim):
        return lim * np.tanh(x / lim) if lim > 0 else x

    def drs_open(self, ax, ay_abs):
        d = self.drs
        if not d.get("enabled"):
            return np.zeros(np.broadcast(ax, ay_abs).shape, bool)
        return (np.asarray(ax) > 0.05) & (np.asarray(ay_abs) < d.get("max_ay", 0.4 * G0))

    # ------------------------------------------------------------------ evaluation
    def evaluate(self, v, zf=0.0, zr=0.0, phi=0.0, beta=0.0, kappa=0.0, drs=False, detail=False):
        """Forces at speed v [m/s] and chassis attitude (turn-relative signs, arrays broadcast).

        Returns dict: F_front, F_rear (axle loads incl. drag moment), D, F (total downforce),
        Fy (side force, + toward the turn centre), Mz (+ yaws into the turn), Mx (+ loads the
        outside wheels, about the centreline at the ground), Fy_z (sum Fy_i z_i), balance
        (downforce centre of pressure, front fraction), comps (per component, if detail)."""
        v = np.asarray(v, float)
        shape = np.broadcast(v, zf, zr, phi, beta, kappa, drs).shape
        v, zf, zr, phi, beta, kappa, drs = (np.broadcast_to(np.asarray(x), shape) for x in
                                            (v, zf, zr, phi, beta, kappa, drs))
        q = 0.5 * self.rho * v ** 2
        L, a = self.L, self.a
        pitch = (zf - zr) / L                                 # rad, + nose down
        Ftot = np.zeros(shape); Mfront = np.zeros(shape); D = np.zeros(shape)
        Fy = np.zeros(shape); Mz = np.zeros(shape); Mx = np.zeros(shape); Fyz = np.zeros(shape)
        Dz = np.zeros(shape)
        per = {}
        for c in self.comps:
            if self.constant:
                Fi = c.ClA * q
                Di = c.CdA * q
                Ftot += Fi; Mfront += Fi * (c.x + L); D += Di
                if detail:
                    per[c.name] = {"F": Fi, "D": Di, "Fy": np.zeros(shape), "Mx": np.zeros(shape),
                                   "yaw_deg": np.zeros(shape), "clearance": np.full(shape, np.nan),
                                   "clearance_min": np.full(shape, np.nan)}
                continue
            xc = c.x + a                                      # from the CG, + forward
            z_loc = zf + (zf - zr) * c.x / L                  # body drop at the component
            dalpha = self._sat(np.degrees(pitch), c.alpha_lin_deg)
            cla = c.ClA + c.dClA_dalpha * dalpha + c.dClA_dh * z_loc
            cda = c.CdA
            if c.drs_cl_ratio is not None and np.any(drs):
                cla = np.where(drs, cla * c.drs_cl_ratio, cla)
                cda = np.where(drs, cda * (c.drs_cd_ratio if c.drs_cd_ratio is not None else 1.0), cda)
            # local flow angle (curved flow + body slip) and its downforce / drag effect
            b_i = beta + xc * kappa
            yr = (b_i / (10 * DEG)) ** 2
            f_yaw = np.clip(1.0 - c.yaw_loss_10deg * yr, 0.3, 1.0)
            # two halves: roll changes their ground clearance, yaw rate their speed
            yc = c.span / 4
            h0 = (c.ground_clearance if c.ground_clearance is not None else np.nan) - z_loc
            h_out, h_in = h0 - phi * yc, h0 + phi * yc
            q_out = q * (1 + yc * kappa) ** 2
            q_in = q * np.maximum(1 - yc * kappa, 0.0) ** 2
            g_out, g_in = self.ground_factor(h_out, c), self.ground_factor(h_in, c)
            F_out = 0.5 * cla * f_yaw * g_out * q_out
            F_in = 0.5 * cla * f_yaw * g_in * q_in
            Fi = F_out + F_in
            q_eff = 0.5 * (q_out + q_in)
            ratio = Fi / np.maximum(q_eff, 1e-9) / c.ClA if abs(c.ClA) > 1e-9 else 1.0
            Di = cda * (1 - c.induced_drag + c.induced_drag * ratio ** 2) * (1 + c.yaw_drag_10deg * yr) * q_eff
            Fyi = -c.side_area * b_i * q_eff
            Mxi = (F_out - F_in) * yc
            Ftot += Fi; Mfront += Fi * (c.x + L); D += Di; Dz += Di * c.z
            Fy += Fyi; Mz += Fyi * xc; Mx += Mxi; Fyz += Fyi * c.z
            if detail:
                per[c.name] = {"F": Fi, "D": Di, "Fy": Fyi, "Mx": Mxi, "yaw_deg": np.degrees(b_i),
                               "clearance": h0, "clearance_min": np.minimum(h_out, h_in),
                               "F_out": F_out, "F_in": F_in, "cla": cla * f_yaw, "f_yaw": f_yaw}
        if self.constant:
            Ff = Mfront / L                                    # original model: no drag moment
        else:
            Ff = (Mfront - Dz) / L
        out = {"F_front": Ff, "F_rear": Ftot - Ff, "F": Ftot, "D": D, "Fy": Fy, "Mz": Mz, "Mx": Mx,
               "Fy_z": Fyz, "balance": Mfront / L / np.where(np.abs(Ftot) > 1e-6, Ftot, np.nan),
               "q": q, "pitch_deg": np.degrees(pitch),
               "M_cg": Mfront - Ftot * (L - a),                 # downforce pitching moment about the CG (+ nose down)
               "Dz": Dz if not self.constant else D * 0.0}      # sum D_i z_i (drag moment about the ground)
        if detail:
            out["comps"] = per
        return out

    # ------------------------------------------------------------------ reporting
    def table(self) -> pd.DataFrame:
        rows = []
        for c in self.comps:
            rows.append({"component": c.name, "ClA [m^2]": c.ClA, "CdA [m^2]": c.CdA,
                         "L/D": c.ClA / c.CdA if c.CdA else np.nan,
                         "x from front axle [m]": c.x, "z [m]": c.z, "span [m]": c.span,
                         "clearance [mm]": None if c.ground_clearance is None else 1e3 * c.ground_clearance,
                         "dClA/dpitch [m^2/deg]": c.dClA_dalpha, "yaw loss @10 deg": c.yaw_loss_10deg,
                         "CyA [m^2/rad]": c.side_area, "mass [kg]": c.mass})
        df = pd.DataFrame(rows).set_index("component")
        tot = df[["ClA [m^2]", "CdA [m^2]", "mass [kg]"]].sum()
        df.loc["total", ["ClA [m^2]", "CdA [m^2]", "mass [kg]"]] = tot.values
        df.loc["total", "L/D"] = tot["ClA [m^2]"] / tot["CdA [m^2]"] if tot["CdA [m^2]"] else np.nan
        return df


# --------------------------------------------------------------------------- inertia estimates
def estimate_inertias(cfg: CarConfig, aero: AeroModel | None = None):
    """(Izz, Ixx_sprung, Iyy_sprung) [kg m^2]: configured values, otherwise estimated from the
    mass and wheelbase/track (radius of gyration 0.38 L in yaw, 0.30 x average track in roll),
    plus the wings as point masses at their centres of pressure. Returns (I, notes)."""
    m = cfg.mass.total
    L = cfg.geometry.wheelbase
    t = 0.5 * (cfg.geometry.track_front + cfg.geometry.track_rear)
    h = cfg.mass.cg_height
    a = L * (1 - cfg.mass.front_weight_fraction)
    notes = []
    wings = []
    if aero is not None and not aero.constant:
        wings = [(c.mass, c.x + a, c.z - h) for c in aero.comps if c.mass and c.name != "body"]
    mw = sum(w[0] for w in wings)
    Iz_w = sum(mm * x ** 2 for mm, x, _ in wings)
    Ix_w = sum(mm * z ** 2 for mm, _, z in wings)
    Iy_w = sum(mm * (x ** 2 + z ** 2) for mm, x, z in wings)
    Iz = cfg.mass.yaw_inertia
    if Iz is None:
        Iz = (m - mw) * (0.38 * L) ** 2 + Iz_w
        notes.append(f"mass.yaw_inertia ESTIMATED {Iz:.0f} kg m^2 (supply from CAD)")
    Ix = cfg.mass.roll_inertia
    if Ix is None:
        Ix = (m - mw) * (0.30 * t) ** 2 * 0.85 + Ix_w
        notes.append(f"mass.roll_inertia ESTIMATED {Ix:.1f} kg m^2 (supply from CAD)")
    Iy = cfg.mass.pitch_inertia
    if Iy is None:
        Iy = 0.9 * (m - mw) * (0.38 * L) ** 2 + Iy_w
        notes.append(f"mass.pitch_inertia ESTIMATED {Iy:.0f} kg m^2 (supply from CAD)")
    return (float(Iz), float(Ix), float(Iy)), notes
