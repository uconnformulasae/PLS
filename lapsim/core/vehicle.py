"""Quasi-steady-state four-wheel vehicle model.

For any (v, ax, ay) the model computes, vectorised over arrays:
  aerodynamic loads and drag; longitudinal load transfer; lateral load transfer
  split into geometric (roll centres), elastic (roll-stiffness distribution) and
  unsprung parts; body roll; heave and pitch with anti-dive/lift/squat; camber
  of every wheel from static camber, roll, FVSA camber gain and steer (caster,
  KPI, Ackermann); tire grip per wheel with load sensitivity and camber; and
  combined-slip (friction-ellipse) capacity per axle.

From these it finds the performance envelope (GGV): the largest lateral
acceleration at each speed with both axles in yaw equilibrium, and the largest
driving/braking acceleration available at each lateral acceleration.
"""
from __future__ import annotations

import dataclasses as dc

import numpy as np

from lapsim.config.schema import CarConfig, MissingDataError
from lapsim.config.units import G0
from lapsim.subsystems import suspension as kin
from lapsim.subsystems.aero import AeroModel
from lapsim.subsystems.suspension import AxleModel, axle_rates, mr_damper_per_wheel
from lapsim.subsystems.tires import Tire


class Vehicle:
    """The chassis of any car. A power module (lapsim.subsystems.power) subclasses it and sets, after
    ``Vehicle.__init__``: ``pt`` (drive-force limit), ``dt`` (driveline: ``.r`` rolling radius),
    ``diff`` (rear differential) and ``coast_torque(v)`` (see power/base.py)."""

    def __init__(self, cfg: CarConfig):
        cfg.require("mass.mass_no_driver", "mass.driver_mass", "mass.front_weight_fraction", "mass.cg_height",
                    "geometry.wheelbase", "geometry.track_front", "geometry.track_rear", "tires.tire_radius",
                    purpose="Vehicle model")
        self.cfg = cfg
        self.warnings = []
        c = cfg
        self.m = c.mass.total
        self.L = c.geometry.wheelbase
        self.tf, self.tr = c.geometry.track_front, c.geometry.track_rear
        self.h = c.mass.cg_height
        self.ff = c.mass.front_weight_fraction
        self.b = self.L * self.ff          # CG to rear axle
        self.a = self.L - self.b           # CG to front axle
        self.tire = Tire(c.tires)
        self.p_f, self.p_r = c.tires.pressure_front, c.tires.pressure_rear
        # axle kinematics: CT17 hardpoints (3-D linkage) or summary values
        self.Af, self.Ar = AxleModel(c, "front"), AxleModel(c, "rear")
        kaf, kar = self.Af.arb_roll_stiffness(), self.Ar.arb_roll_stiffness()
        self.warnings += self.Af.notes + self.Ar.notes
        self.Rf = axle_rates(c, "front", self.Af.mr_d, kaf)
        self.Rr = axle_rates(c, "rear", self.Ar.mr_d, kar)
        self.warnings += self.Rf.notes + self.Rr.notes
        self.mrf = self.Af.mr_d if self.Af.mr_d is not None else mr_damper_per_wheel(c.front)
        self.mrr = self.Ar.mr_d if self.Ar.mr_d is not None else mr_damper_per_wheel(c.rear)
        # ride-height adjustment raises/lowers the sprung CG
        self.h = self.h + (self.Af.body_rise * self.ff + self.Ar.body_rise * (1 - self.ff))
        anti = {"anti_dive": c.anti.anti_dive, "anti_lift": c.anti.anti_lift, "anti_squat": c.anti.anti_squat}
        if c.front.hardpoint_file and c.front.use_hardpoint_anti and "anti_dive" in self.Af.anti:
            anti["anti_dive"] = self.Af.anti["anti_dive"]
        if c.rear.hardpoint_file and c.rear.use_hardpoint_anti:
            anti.update({k: v for k, v in self.Ar.anti.items() if k in ("anti_lift", "anti_squat")})
        self.anti = {k: (v or 0.0) for k, v in anti.items()}
        self.toe_f, self.toe_r = float(self.Af.toe(0.0)), float(self.Ar.toe(0.0))
        # wing setting
        ws = c.adjust.wing_setting
        if ws is not None:
            st_ = c.aero.settings.get(ws)
            if not st_ or any(st_.get(k) is None for k in ("ClA_front", "ClA_rear", "CdA")):
                raise MissingDataError(f"aero.settings['{ws}'] needs ClA_front, ClA_rear and CdA")
        self.mu_f = c.mass.unsprung_mass_front or 0.0
        self.mu_r = c.mass.unsprung_mass_rear or 0.0
        self.ms = self.m - 2 * self.mu_f - 2 * self.mu_r
        self.hu = c.mass.unsprung_cg_height or c.tires.tire_radius
        self.hs = (self.m * self.h - 2 * (self.mu_f + self.mu_r) * self.hu) / self.ms
        self.crr = c.tires.rolling_resistance or 0.0
        if c.tires.rolling_resistance is None:
            self.warnings.append("tires: rolling resistance not set - omitted")
        if (c.tires.gamma_opt_deg is None or c.tires.k_gamma is None):
            self.warnings.append("tires: no camber sensitivity (gamma_opt_deg, k_gamma) - camber, caster, KPI "
                                 "and roll-camber changes have NO effect on grip until fitted from TTC data")
        self.bias = c.brakes.front_bias if c.brakes.front_bias is not None else None
        self.pt = self.dt = self.diff = None          # set by the power module
        self.mux = c.tires.mux_over_muy
        self.Kf, self.Kr = self.Rf.roll_stiffness, self.Rr.roll_stiffness
        self.cam0 = (self.Af.static_camber(), self.Ar.static_camber())
        # toe scrub drag (each wheel at slip angle = toe): F = C_alpha * toe^2 per wheel
        cpl = c.tires.cornering_stiffness_per_load
        self.toe_drag_coef = 0.0
        if cpl is not None:
            tf, tr = np.radians(self.toe_f), np.radians(self.toe_r)
            self.toe_drag_coef = cpl * (self.ff * tf ** 2 + (1 - self.ff) * tr ** 2)   # x total normal load
        elif abs(self.toe_f) > 1e-3 or abs(self.toe_r) > 1e-3:
            self.warnings.append("toe: cornering stiffness unknown - toe scrub drag not modelled")
        if c.tires.k_gamma is not None and c.tires.k_gamma > 0.05:
            self.warnings.append(f"tires.k_gamma = {c.tires.k_gamma} per deg^2 removes {100 * c.tires.k_gamma:.0f}% of "
                                 "grip per degree^2 from the optimum camber - check the units (typical 0.002-0.02)")
        self._build_aero(c)
        # rear-slip estimate for the body slip angle (aero yaw): cornering stiffness per load
        self._cpl = cpl
        self.cornering_drag = bool(c.sim.cornering_drag)
        self._straight_table()

    # ---------------------------------------------------------------- aero
    def _build_aero(self, c):
        self.rho = c.aero.density()
        self._cpl = None
        if c.adjust.wing_setting is not None:        # a named wing setting: constant model with its values
            st_ = c.aero.settings[c.adjust.wing_setting]
            c = c.copy()
            c.aero.model, c.aero.ClA_front, c.aero.ClA_rear, c.aero.CdA = \
                "constant", st_["ClA_front"], st_["ClA_rear"], st_["CdA"]
        if c.aero.model == "constant" and c.aero.cla() is None:
            self.warnings.append("aero: no downforce data - downforce omitted")
        if c.aero.model == "constant" and c.aero.CdA is None:
            self.warnings.append("aero: CdA not set (fit from coast-down) - drag omitted")
        self.aero_model = AeroModel.from_config(c, self.L, self.a)
        self.warnings += self.aero_model.notes
        self.aero_iters = 1 if self.aero_model.constant else max(1, int(c.aero.aero_iterations))
        self._tab = None
        # scale to the measured whole-car values at the measured speed (ride height solved there)
        A = c.aero
        if not self.aero_model.constant and A.calib_speed and (A.calib_ClA or A.calib_CdA):
            for _ in range(3):
                st = self._aero_state(np.array([A.calib_speed]), np.zeros(1), np.zeros(1))
                kl = A.calib_ClA / float(st["aero"]["F"][0] / st["aero"]["q"][0]) if A.calib_ClA else 1.0
                kd = A.calib_CdA / float(st["aero"]["D"][0] / st["aero"]["q"][0]) if A.calib_CdA else 1.0
                self.aero_model.scale(kl, kd)
                if abs(kl - 1) < 1e-4 and abs(kd - 1) < 1e-4:
                    break
        q1 = 0.5 * self.rho
        st0 = self.aero_model.evaluate(np.ones(1))
        # nominal (static attitude) coefficients - kept for code that reads claf / clar / cda
        self.claf = float(st0["F_front"][0] / q1)
        self.clar = float(st0["F_rear"][0] / q1)
        self.cda = float(st0["D"][0] / q1)

    def _aero_state(self, v, ax, ay):
        """Heave / pitch / roll / slip <-> aero fixed point (used by state(); exposed for calibration)."""
        return self.state(v, ax, ay, _need_tire=False)

    def _straight_table(self):
        """Straight-line aero at the aero-loaded ride height on a speed grid (for aero(v) / resist(v))."""
        vg = np.linspace(0.0, 60.0, 121)
        st = self.state(vg, np.zeros_like(vg), np.zeros_like(vg), _need_tire=False)
        q = np.maximum(0.5 * self.rho * vg ** 2, 1e-12)
        A = st["aero"]
        cf, cr, cd = A["F_front"] / q, A["F_rear"] / q, A["D"] / q
        cf[0], cr[0], cd[0] = cf[1], cr[1], cd[1]
        self._tab = (vg, cf, cr, cd)
        # accelerating in a straight line (DRS open when enabled)
        st2 = self.state(vg, np.full_like(vg, 1.0), np.zeros_like(vg), _need_tire=False)
        A2 = st2["aero"]
        self._tab_acc = (A2["F"] / q, A2["D"] / q)
        self._tab_acc[0][0], self._tab_acc[1][0] = self._tab_acc[0][1], self._tab_acc[1][1]

    def elastic_share_front(self):
        """Share of the elastic (spring / bar) lateral load transfer taken by the front axle.
        Rigid frame: Kf / (Kf + Kr). With mass.chassis_torsional_stiffness Kc the front and rear
        of the car roll by different angles: the roll moment enters each end in proportion to the
        sprung weight it carries and the frame, a torsion spring between them, passes on only part
        of the difference - so a soft frame pulls the split towards the weight distribution and
        takes away what the springs and bars were set to do."""
        Kf, Kr = self.Kf, self.Kr
        Kc = self.cfg.mass.chassis_torsional_stiffness
        if Kc is None or not np.isfinite(Kc):
            return Kf / (Kf + Kr)
        return Kf * (self.ff * (Kr + Kc) + (1 - self.ff) * Kc) / (Kf * Kr + Kc * (Kf + Kr))

    def aero(self, v):
        """(front-axle downforce, rear-axle downforce, drag) [N] in a straight line at the
        aero-loaded ride height (speed dependent with the component model)."""
        v = np.asarray(v, float)
        q = 0.5 * self.rho * v ** 2
        if self._tab is None:
            return self.claf * q, self.clar * q, self.cda * q
        vg, cf, cr, cd = self._tab
        va = np.clip(np.abs(v), 0, vg[-1])
        return np.interp(va, vg, cf) * q, np.interp(va, vg, cr) * q, np.interp(va, vg, cd) * q

    def resist(self, v):
        """Drag + rolling resistance in a straight line [N]."""
        ff, fr, d = self.aero(v)
        return d + (self.crr + self.toe_drag_coef) * (self.m * G0 + ff + fr)

    def resist_drive(self, v):
        """Drag + rolling resistance in a straight line while accelerating (DRS open if enabled)."""
        v = np.asarray(v, float)
        if self._tab is None:
            return self.resist(v)
        vg = self._tab[0]
        q = 0.5 * self.rho * v ** 2
        va = np.clip(np.abs(v), 0, vg[-1])
        F = np.interp(va, vg, self._tab_acc[0]) * q
        D = np.interp(va, vg, self._tab_acc[1]) * q
        return D + (self.crr + self.toe_drag_coef) * (self.m * G0 + F)

    def resist_state(self, st):
        """Drag + rolling resistance + tire cornering drag at a solved state (drag at that
        attitude, yaw and DRS)."""
        return st["drag"] + (self.crr + self.toe_drag_coef) * (self.m * G0 + st["downforce"]) \
            + st.get("cornering_drag", 0.0)

    def _axle_slip(self, W, req, p, D=None):
        """Axle slip angle [rad] for a lateral force req from the inverse simplified Magic Formula
        (C = 1.4, slope = cornering stiffness per load x load, peak D = mu W or the given capacity)."""
        if D is None:
            D = self.tire.mu(np.maximum(W / 2, 1.0), 0.0, p) * W
        D = np.maximum(D, 1.0)
        C = 1.4
        B = self._cpl * W / (C * D)
        u = np.clip(req / D, 0.0, 0.999)
        return np.tan(np.arcsin(u) / C) / np.maximum(B, 1e-6)

    def _beta(self, v, aay, kappa, Wr, req_r):
        """Body slip angle at the CG (+ = velocity points into the turn): b/R - alpha_rear."""
        if self._cpl is None:
            return self.b * kappa
        return self.b * kappa - self._axle_slip(Wr, req_r, self.p_r)

    def state(self, v, ax, ay, _need_tire=True):
        """Full quasi-static state for arrays v, ax, ay (ay > 0: left turn)."""
        v, ax, ay = np.broadcast_arrays(np.asarray(v, float), np.asarray(ax, float), np.asarray(ay, float))
        aay = np.abs(ay)
        kappa = np.where(v > 0.5, aay / np.maximum(v, 0.5) ** 2, 0.0)
        AM = self.aero_model
        drs = AM.drs_open(ax, aay)
        brake = ax < 0
        anti_f = np.where(brake, self.anti["anti_dive"], 0.0)
        anti_r = np.where(brake, self.anti["anti_lift"], self.anti["anti_squat"])
        dWx = self.m * ax * self.h / self.L
        Ktot = self.Kf + self.Kr
        zf = zr = phi = beta = np.zeros_like(v)
        for _ in range(self.aero_iters):
            # aero at the current attitude, then the attitude from the loads
            A = AM.evaluate(v, zf, zr, phi, beta, kappa, drs)
            Faf, Far, D = A["F_front"], A["F_rear"], A["D"]
            Wf = self.m * G0 * self.ff + Faf - dWx
            Wr = self.m * G0 * (1 - self.ff) + Far + dWx
            # heave / pitch (spring deflection per wheel, + = compression); anti from geometry
            zf = (Faf / 2 - dWx / 2 * (1 - anti_f)) / self.Rf.ride_rate
            zr = (Far / 2 + dWx / 2 * (1 - anti_r)) / self.Rr.ride_rate
            rcf, rcr = self.Af.rc(zf), self.Ar.rc(zr)
            h_ra = rcf + (rcr - rcf) * self.a / self.L
            # roll: inertia + aero rolling moment (asymmetric downforce, side force above the roll axis)
            arm = self.hs - h_ra
            Mx_a = A["Mx"] - (A["Fy_z"] - A["Fy"] * h_ra)
            phi = (self.ms * aay * arm + Mx_a) / np.maximum(Ktot - self.ms * G0 * arm, 1.0)          # rad
            if not AM.constant:
                req_r = np.maximum(((self.m * aay - A["Fy"]) * self.a + A["Mz"]) / self.L, 0.0)
                beta = self._beta(v, aay, kappa, Wr, req_r)
        Mel = self.ms * arm * (aay + G0 * phi) + Mx_a
        el_f = self.elastic_share_front()
        dF_f = (self.ms * aay * self.ff * rcf + Mel * el_f + 2 * self.mu_f * aay * self.hu) / self.tf
        dF_r = (self.ms * aay * (1 - self.ff) * rcr + Mel * (1 - el_f) + 2 * self.mu_r * aay * self.hu) / self.tr
        Fz = {"fo": Wf / 2 + dF_f, "fi": Wf / 2 - dF_f, "ro": Wr / 2 + dF_r, "ri": Wr / 2 - dF_r}
        lift = np.minimum.reduce([Fz[k] for k in Fz]) < 0
        Fz = {k: np.maximum(x, 0.0) for k, x in Fz.items()}
        aero_st = {"F_front": Faf, "F_rear": Far, "F": Faf + Far, "D": D, "Fy": A["Fy"], "Mz": A["Mz"],
                   "Mx": Mx_a, "q": A["q"], "balance": A["balance"]}
        if not _need_tire:
            return {"v": v, "ax": ax, "ay": ay, "Fz": Fz, "aero": aero_st, "heave_front": zf, "heave_rear": zr,
                    "roll_deg": np.degrees(phi), "beta": beta, "drag": D, "downforce": Faf + Far}
        # wheel travel per corner (+ bump) and camber (turn-relative, deg; - = leaning into the turn)
        zfo, zfi = zf + phi * self.tf / 2, zf - phi * self.tf / 2
        zro, zri = zr + phi * self.tr / 2, zr - phi * self.tr / 2
        phid = np.degrees(phi)
        cf, cr = self.cam0
        kappa = np.where(v > 0.5, aay / np.maximum(v, 0.5) ** 2, 0.0)
        R = np.where(kappa > 1e-4, 1 / np.maximum(kappa, 1e-4), 1e4)
        ack = self.cfg.steering.ackermann_fraction if self.cfg.steering.ackermann_fraction is not None else 1.0
        di, do = kin.ackermann_angles(R, self.L, self.tf, ack)
        so, _ = kin.steer_camber_deg(do, self.Af.caster_deg, self.Af.kpi_deg)
        _, si = kin.steer_camber_deg(di, self.Af.caster_deg, self.Af.kpi_deg)
        gam = {"fo": cf + self.Af.dcamber(zfo) + phid + so,
               "fi": -(cf + self.Af.dcamber(zfi)) + phid + si,
               "ro": cr + self.Ar.dcamber(zro) + phid,
               "ri": -(cr + self.Ar.dcamber(zri)) + phid}
        mu = {k: self.tire.mu(Fz[k], gam[k], self.p_f if k[0] == "f" else self.p_r) for k in Fz}
        # tire cornering drag: lateral force x sin(slip angle) of each axle opposes the motion
        F_cd = np.zeros_like(v)
        if self.cornering_drag and self._cpl is not None:
            T = self.m * aay - A["Fy"]
            rq_f = np.maximum((T * self.b - A["Mz"]) / self.L, 0.0)
            rq_r = np.maximum((T * self.a + A["Mz"]) / self.L, 0.0)
            al_f = self._axle_slip(Wf, rq_f, self.p_f, mu["fo"] * Fz["fo"] + mu["fi"] * Fz["fi"])
            al_r = self._axle_slip(Wr, rq_r, self.p_r, mu["ro"] * Fz["ro"] + mu["ri"] * Fz["ri"])
            F_cd = rq_f * np.sin(al_f) + rq_r * np.sin(al_r)
        return {"v": v, "ax": ax, "ay": ay, "Fz": Fz, "camber": gam, "mu": mu, "roll_deg": phid,
                "pitch_deg": np.degrees((zf - zr) / self.L), "heave_front": zf, "heave_rear": zr,
                "travel": {"fo": zfo, "fi": zfi, "ro": zro, "ri": zri},
                "damper": {"fo": zfo * self.mrf, "fi": zfi * self.mrf, "ro": zro * self.mrr, "ri": zri * self.mrr},
                "rc_front": rcf, "rc_rear": rcr,
                "lt_front": dF_f, "lt_rear": dF_r, "wheel_lift": lift, "drag": D,
                "downforce": Faf + Far, "steer_inner": di, "steer_outer": do,
                "aero": aero_st, "aero_front": Faf, "aero_rear": Far, "aero_balance": A["balance"],
                "beta": beta, "beta_deg": np.degrees(beta), "kappa": kappa, "drs": drs, "phi": phi,
                "cornering_drag": F_cd}

    def _capacity(self, st, fx):
        """Per-wheel lateral capacity given longitudinal forces fx (tire model's combined slip)."""
        cap = {}
        for k in st["Fz"]:
            fy, over = self.tire.lateral_capacity(st["Fz"][k], st["camber"][k], fx.get(k, 0.0),
                                                  self.p_f if k[0] == "f" else self.p_r)
            cap[k], cap[k + "_xover"] = fy, over
        return cap

    def _rear_caps(self, st):
        p = self.p_r
        ci = self.tire.peak_fx(st["Fz"]["ri"], st["camber"]["ri"], p)
        co = self.tire.peak_fx(st["Fz"]["ro"], st["camber"]["ro"], p)
        return np.asarray(ci, float), np.asarray(co, float)

    def _wheel_fx(self, v, ax, st=None):
        """Per-wheel longitudinal force demand for acceleration ax (+drive / -brake). On power the rear
        drive is split by the differential using each rear tire's own capacity; under braking the coast
        side of the differential is loaded by ``coast_torque`` (engine braking or regen)."""
        need = self.m * ax + (self.resist_state(st) if st is not None else self.resist(v))
        drive = np.maximum(need, 0.0)
        brk = np.maximum(-need, 0.0)
        bias = self.bias if self.bias is not None else 0.6
        fo = fi = brk * bias / 2
        rb = brk * (1 - bias) / 2
        if st is None:
            return {"fo": fo, "fi": fi, "ro": drive / 2 + rb, "ri": drive / 2 + rb}, drive, brk
        kap_t = self.tr * np.abs(st["ay"]) / np.maximum(st["v"], 1.0) ** 2
        ci, co = self._rear_caps(st)
        Fi, Fo, ok = self.diff.split(drive, ci, co, st["Fz"]["ri"], st["Fz"]["ro"], kap_t, self.dt.r)
        if np.any(brk > 0) and self.diff.type != "open":
            t_coast = self.coast_torque(st["v"])
            # locked-axle scrub under braking, limited by the coast ramp + preload
            fi_k, fo_k = self.diff.locked_split(-2 * rb, ci, co, st["Fz"]["ri"], st["Fz"]["ro"], kap_t)
            lock_c = self.diff.lock_torque(t_coast, coast=True) / self.dt.r
            dF_c = np.where(brk > 0, np.minimum(np.maximum(fi_k - fo_k, 0.0), lock_c), 0.0)
        else:
            dF_c = np.zeros_like(drive)
        ro = np.where(brk > 0, rb + dF_c / 2, Fo)
        ri = np.where(brk > 0, np.abs(rb - dF_c / 2), Fi)
        self._diff_ok = np.where(brk > 0, True, ok)
        self._mz = np.where(brk > 0, -dF_c, Fo - Fi) * self.tr / 2
        return {"fo": fo, "fi": fi, "ro": ro, "ri": ri}, drive, brk

    def coast_torque(self, v):
        """Axle torque [N m] on the coast side of the differential while braking (power module)."""
        raise NotImplementedError

    def axle_requirements(self, st, mz=0.0):
        """Lateral force each axle must give: tires carry m*ay minus the aero side force, and the
        axle split balances the aero yaw moment (and any extra yaw moment mz, e.g. the diff)."""
        aay = np.abs(st["ay"])
        T = self.m * aay - st["aero"]["Fy"]
        Mz = st["aero"]["Mz"] + mz
        return np.maximum((T * self.b - Mz) / self.L, 0.0), np.maximum((T * self.a + Mz) / self.L, 0.0)

    def margins(self, v, ax, ay):
        """Ratios capacity/required for front and rear axle (>=1 feasible), and power margin."""
        st = self.state(v, ax, ay)
        fx, drive, _ = self._wheel_fx(st["v"], st["ax"], st)
        cap = self._capacity(st, fx)
        mz = self._mz
        req_f, req_r = self.axle_requirements(st, mz)
        capf, capr = cap["fo"] + cap["fi"], cap["ro"] + cap["ri"]
        xok = ~(cap["fo_xover"] | cap["fi_xover"] | cap["ro_xover"] | cap["ri_xover"]) & self._diff_ok
        mf = np.where(req_f > 1e-6, capf / np.maximum(req_f, 1e-6), np.inf)
        mr = np.where(req_r > 1e-6, capr / np.maximum(req_r, 1e-6), np.inf)
        mf = np.where(xok, mf, 0.0)
        mr = np.where(xok, mr, 0.0)
        if self.pt.unlimited or not np.any(drive > 0):
            mp = np.full_like(mf, np.inf)
        else:
            mp = np.where(drive > 0, self.pt.force(st["v"]) / np.maximum(drive, 1e-6), np.inf)
        st["diff_mz"] = mz
        return mf, mr, mp, st

    def diff_state(self, v, ax, ay):
        """Rear wheel forces, diff yaw moment and locking state for a solved lap (arrays)."""
        st = self.state(v, ax, ay)
        fx, drive, brk = self._wheel_fx(st["v"], st["ax"], st)
        return {"F_ri": fx["ri"], "F_ro": fx["ro"], "Mz": self._mz, "drive": drive, "brake": brk, "state": st}

    # ---------------------------------------------------------------- limits
    def _bisect(self, feasible, lo, hi, n=40):
        lo, hi = np.array(lo, float), np.array(hi, float)
        for _ in range(n):
            mid = 0.5 * (lo + hi)
            ok = feasible(mid)
            lo = np.where(ok, mid, lo)
            hi = np.where(ok, hi, mid)
        return lo

    def ay_max(self, v, ax=0.0):
        v = np.atleast_1d(np.asarray(v, float))
        axa = np.broadcast_to(ax, v.shape)
        def ok(ay):
            mf, mr, mp, st = self.margins(v, axa, ay)
            return (mf >= 1) & (mr >= 1) & (mp >= 1) & ~st["wheel_lift"]
        return self._bisect(ok, np.zeros_like(v), np.full_like(v, 5 * G0))

    def ax_accel_max(self, v, ay):
        v, ay = np.broadcast_arrays(np.atleast_1d(np.asarray(v, float)), np.atleast_1d(np.asarray(ay, float)))
        def ok(ax):
            mf, mr, mp, st = self.margins(v, ax, ay)
            return (mf >= 1) & (mr >= 1) & (mp >= 1)
        hi = np.full(v.shape, 3 * G0)
        a = self._bisect(ok, np.zeros(v.shape), hi)
        # if even coasting is infeasible (drag > drive at top speed) allow negative
        coast = -self.resist(v) / self.m
        return np.where(a <= 1e-6, np.minimum(coast, 0.0), a)

    def ax_brake_max(self, v, ay):
        """Largest deceleration (returned positive)."""
        v, ay = np.broadcast_arrays(np.atleast_1d(np.asarray(v, float)), np.atleast_1d(np.asarray(ay, float)))
        def ok(d):
            mf, mr, mp, st = self.margins(v, -d, ay)
            return (mf >= 1) & (mr >= 1)
        d = self._bisect(ok, np.zeros(v.shape), np.full(v.shape, 4 * G0))
        if self.cfg.brakes.max_decel is not None:
            d = np.minimum(d, self.cfg.brakes.max_decel)
        return np.maximum(d, self.resist(v) / self.m)

    def limit_balance(self, v):
        """At the lateral limit, which axle saturates first: returns (ay_max, front_margin, rear_margin).
        A front margin of 1 with rear margin > 1 means limit understeer (front-limited)."""
        aym = self.ay_max(v)
        mf, mr, _, _ = self.margins(v, 0.0, aym * 0.999)
        return aym, mf, mr

    def handling_curve(self, v, ay):
        """Steady-state understeer angle (delta - L/R) [deg] versus lateral acceleration.

        For each ay the front and rear axle slip angles are solved so that each
        axle's summed tire force (per-wheel loads and cambers, Magic Formula)
        equals its yaw-balance requirement. Positive = understeer. Needs
        tires.cornering_stiffness_per_load (TTC fit)."""
        from scipy.optimize import brentq
        if self.cfg.tires.cornering_stiffness_per_load is None:
            raise MissingDataError("tires.cornering_stiffness_per_load needed for the handling curve")
        out = []
        for a_y in np.atleast_1d(ay):
            st = self.state(v, 0.0, a_y)
            req = {"f": self.m * a_y * self.b / self.L, "r": self.m * a_y * self.a / self.L}
            alpha = {}
            for ax_, (o, i) in {"f": ("fo", "fi"), "r": ("ro", "ri")}.items():
                pr = self.p_f if ax_ == "f" else self.p_r
                f = lambda al: (self.tire.fy(al, st["Fz"][o], st["camber"][o], pr)
                                + self.tire.fy(al, st["Fz"][i], st["camber"][i], pr) - req[ax_]).item()
                grid = np.radians(np.linspace(0, 20, 201))
                vals = np.array([f(a) for a in grid])
                pos = np.flatnonzero(vals >= 0)
                alpha[ax_] = (brentq(f, grid[pos[0] - 1], grid[pos[0]]) if pos.size and pos[0] > 0 else
                              (0.0 if pos.size else np.nan))
            out.append(np.degrees(alpha["f"] - alpha["r"]))
        return np.array(out)

    # ---------------------------------------------------------------- GGV
    def ggv(self, v_max=None, n_v=None, n_ay=None):
        s = self.cfg.sim
        v_max = v_max or s.v_grid_max
        vg = np.linspace(1.0, v_max, n_v or s.n_v)
        aym = self.ay_max(vg)
        fr = np.linspace(0, 1, n_ay or s.n_ay)
        V, Fr = np.meshgrid(vg, fr, indexing="ij")
        AY = Fr * aym[:, None] * 0.999
        acc = self.ax_accel_max(V.ravel(), AY.ravel()).reshape(V.shape)
        brk = self.ax_brake_max(V.ravel(), AY.ravel()).reshape(V.shape)
        return GGV(vg, fr, aym, acc, brk)

    def traction_ggv(self, **kw):
        """GGV limited by the tires and differential only (no power limit; the solvers apply the power
        module's limit gear by gear / state by state), same arguments as ``ggv``."""
        self.pt.unlimited = True
        try:
            return self.ggv(**kw)
        finally:
            self.pt.unlimited = False


@dc.dataclass
class GGV:
    v: np.ndarray
    frac: np.ndarray
    ay_max: np.ndarray
    ax_acc: np.ndarray     # [v, frac]
    ax_brk: np.ndarray     # [v, frac] positive deceleration

    def _interp(self, table, v, ay):
        from scipy.interpolate import RegularGridInterpolator
        key = id(table)
        if not hasattr(self, "_rgi"):
            self._rgi = {}
        if key not in self._rgi:
            self._rgi[key] = RegularGridInterpolator((self.v, self.frac), table, bounds_error=False, fill_value=None)
        v = np.clip(np.asarray(v, float), self.v[0], self.v[-1])
        ay = np.asarray(ay, float)
        aym = np.interp(v, self.v, self.ay_max)
        f = np.clip(np.abs(ay) / np.maximum(aym, 1e-6), 0, 1)
        r = self._rgi[key](np.column_stack([np.atleast_1d(v), np.atleast_1d(f)]))
        r = r.reshape(np.shape(v)) if np.ndim(v) else r[0]
        return np.where(np.abs(ay) > aym * 1.0001, 0.0, r)

    def _fast(self, table, v, ay):
        """Scalar bilinear lookup (used inside the lap-solver loop)."""
        import bisect
        vg = self.v
        v = min(max(v, vg[0]), vg[-1])
        i = min(max(bisect.bisect_right(self._vl, v) - 1, 0), len(vg) - 2)
        tv = (v - vg[i]) / (vg[i + 1] - vg[i])
        aym = self.ay_max[i] * (1 - tv) + self.ay_max[i + 1] * tv
        a = abs(ay)
        if a > aym * 1.0001:
            return 0.0
        f = a / aym if aym > 0 else 0.0
        nf = len(self.frac) - 1
        jf = f * nf
        j = min(int(jf), nf - 1)
        tf = jf - j
        r0 = table[i, j] * (1 - tf) + table[i, j + 1] * tf
        r1 = table[i + 1, j] * (1 - tf) + table[i + 1, j + 1] * tf
        return r0 * (1 - tv) + r1 * tv

    def acc(self, v, ay):
        if np.ndim(v) == 0 and np.ndim(ay) == 0:
            return self._fast(self.ax_acc, float(v), float(ay))
        return self._interp(self.ax_acc, v, ay)

    def brk(self, v, ay):
        if np.ndim(v) == 0 and np.ndim(ay) == 0:
            return self._fast(self.ax_brk, float(v), float(ay))
        return self._interp(self.ax_brk, v, ay)

    def __post_init__(self):
        self._vl = list(self.v)

    def ay_lim(self, v):
        return np.interp(v, self.v, self.ay_max)
