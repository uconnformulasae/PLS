"""Transient (time-domain) solver: checks the quasi-steady lap with full pitch, roll and yaw dynamics.

The quasi-steady lap simulation (solvers/qss*.py) assumes every point is a
steady state. This model drives the same car (suspension rates, anti
geometry, roll centres, tires, aero, powertrain limit) around the same track
in the time domain, so the attitude lags, overshoots and the aero changes it
causes are visible.

Degrees of freedom (ISO 8855: x forward, y left, z up)
    planar      X, Y, yaw psi, u, v, yaw rate r               (whole car, mass m, Izz)
    sprung      heave z (+ down), pitch theta (+ nose down), roll phi (+ right side down)
                (sprung mass ms, Iyy, Ixx about the roll axis; springs, ARBs, dampers)
    tires       lateral slip of each wheel with relaxation length sigma:
                d(alpha)/dt = (Vx / sigma) (alpha_kinematic - alpha)

Wheel loads: static + suspension (spring + damper, heave/pitch/roll) + anti
jacking (anti-dive / lift / squat) + geometric transfer through the roll
centres + unsprung inertia. Tire forces: the vehicle's tire model (peak mu with
load and camber, Magic-Formula lateral curve), friction ellipse for combined
slip. Aero: ``AeroModel`` at the instantaneous ride heights, roll, body slip,
yaw rate and (optional) crosswind; downforce pitching moment and drag moment
act on the sprung mass. Driver: preview path follower (feed-forward
L*kappa + understeer, feedback on the previewed lateral error and on the yaw
rate) and a speed follower on the quasi-steady speed profile scaled by
``driver_level``.

Explicit RK4, dt = 2 ms; driver commands, aero and the drive-force limit are
held over a step.
"""
from __future__ import annotations

import dataclasses as dc
import math

import numpy as np
import pandas as pd

from lapsim.config.units import G0
from lapsim.core.integrator import rk4_step
from lapsim.core.track import Track
from lapsim.subsystems.aero import estimate_inertias


@dc.dataclass
class TransientResult:
    name: str
    df: pd.DataFrame
    qss: pd.DataFrame | None
    summary: dict
    notes: list


def _wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


class Chassis:
    def __init__(self, veh, dt=0.002, sigma=0.30, driver_level=0.98, wind=(0.0, 0.0)):
        """``veh``: ICVehicle or EVVehicle. ``wind``: (speed m/s, direction deg the wind blows TOWARD,
        in the track x-y frame)."""
        self.veh = veh
        c = veh.cfg
        self.dt, self.sigma, self.level = dt, sigma, driver_level
        self.back_off, self.recover = 0.6, 0.35       # driver: grip-margin rates [1/s]
        self.lp_k, self.lp_0, self.ff_t, self.kp_mult, self.e_backoff = 0.3, 2.0, 0.2, 1.2, 1.5
        self.beta_lift_deg = 8.0                       # driver lifts beyond this body slip
        self.notes = []
        self.m, self.ms = veh.m, veh.ms
        self.a, self.b, self.L = veh.a, veh.b, veh.L
        self.tf, self.tr = veh.tf, veh.tr
        self.h, self.hs, self.hu = veh.h, veh.hs, veh.hu
        self.muf, self.mur = veh.mu_f, veh.mu_r
        (self.Iz, self.Ix, self.Iy), n = estimate_inertias(c, veh.aero_model)
        self.notes += n
        self.Wf0 = veh.m * G0 * veh.ff
        self.Wr0 = veh.m * G0 * (1 - veh.ff)
        self.Kf_ax, self.Kr_ax = 2 * veh.Rf.ride_rate, 2 * veh.Rr.ride_rate
        self.Kphf, self.Kphr = veh.Kf, veh.Kr
        zeta = []
        for ax_ in (c.front, c.rear):
            zb, zr = ax_.damping_ratio_bump, ax_.damping_ratio_rebound
            zeta.append(0.5 * ((zb or 0.7) + (zr or 0.7)))
        mcf = self.ms * veh.ff / 2
        mcr = self.ms * (1 - veh.ff) / 2
        cwf = 2 * zeta[0] * math.sqrt(veh.Rf.ride_rate * mcf)
        cwr = 2 * zeta[1] * math.sqrt(veh.Rr.ride_rate * mcr)
        self.Cf_ax, self.Cr_ax = 2 * cwf, 2 * cwr
        self.Cphf, self.Cphr = cwf * self.tf ** 2 / 2, cwr * self.tr ** 2 / 2
        self.rcf0, self.rcr0 = float(veh.Af.rc(0.0)), float(veh.Ar.rc(0.0))
        self.h_ra = self.rcf0 + (self.rcr0 - self.rcf0) * self.a / self.L
        self.anti = veh.anti
        # camber: static + linear gain per metre of wheel travel
        self.cf0, self.cr0 = veh.cam0
        d = 0.005
        self.gcf = float(veh.Af.dcamber(d) - veh.Af.dcamber(-d)) / (2 * d)
        self.gcr = float(veh.Ar.dcamber(d) - veh.Ar.dcamber(-d)) / (2 * d)
        # tires
        tp = c.tires
        self.fast = tp.cornering_stiffness_per_load is not None
        if self.fast:
            self.tp = (tp.lambda_mu, tp.mu0, tp.mu_fz, tp.fz0, tp.gamma_opt_deg, tp.k_gamma,
                       tp.cornering_stiffness_per_load, tp.mux_over_muy)
        self.bias = veh.bias if veh.bias is not None else 0.6
        self.crr = veh.crr + veh.toe_drag_coef
        # drive-force limit (best gear, WOT) on a speed grid
        vg = np.linspace(0.0, 45.0, 451)
        if hasattr(veh, "engine"):        # IC car: per-gear WOT force / equivalent mass (clutch slip at launch)
            from lapsim.solvers.qss_ic import ICTables
            T = ICTables(veh)
            vg = T.v
            Fd = np.max(T.F * veh.m / T.meq_v, axis=0)
        else:                             # EV: motor force at the start-of-event state / equivalent mass
            T = veh.tables0
            vg = T.v
            Fd = T.F * veh.m / T.meq
        self._vg, self._Fd = vg, np.minimum(Fd, 1e5)
        th = math.radians(wind[1])
        self.wind = (wind[0] * math.cos(th), wind[0] * math.sin(th))
        # understeer feed-forward from the steady handling curve
        try:
            ayg = np.linspace(0, 1.6 * G0, 9)
            us = veh.handling_curve(15.0, ayg)
            self._us = (ayg, np.radians(np.nan_to_num(us, nan=float(np.nanmax(us)) if np.any(np.isfinite(us)) else 0.0)))
        except Exception:
            self._us = (np.array([0.0, 30.0]), np.zeros(2))

    # ------------------------------------------------------------------ tires
    def _mu(self, fz, gam, front):
        if self.fast:
            lam, mu0, mufz, fz0, gopt, kg, cpl, mux = self.tp
            mu = lam * (mu0 + mufz * (fz - fz0))
            if gopt is not None and kg is not None:
                mu *= min(max(1.0 - kg * (gam - gopt) ** 2, 0.3), 1.0)
            return max(mu, 0.05)
        v = self.veh
        return float(v.tire.mu(fz, gam, v.p_f if front else v.p_r))

    def _fy(self, alpha, fz, gam, front):
        if fz <= 1.0:
            return 0.0, 1.0
        if self.fast:
            mu = self._mu(fz, gam, front)
            C = 1.4
            B = self.tp[6] / (C * mu)
            D = mu * fz
            return D * math.sin(C * math.atan(B * alpha)), D
        v = self.veh
        p = v.p_f if front else v.p_r
        D = float(v.tire.peak_fy(fz, gam, p))
        return float(v.tire.fy(alpha, fz, gam, p)), D

    # ------------------------------------------------------------------ dynamics
    def _rhs(self, y, delta, Fx_cmd, aero, prev):
        u = max(y[3], 0.5)
        vy, r = y[4], y[5]
        z, zd, th, thd, ph, phd = y[6], y[7], y[8], y[9], y[10], y[11]
        al = y[12:16]
        a, b, tf, tr = self.a, self.b, self.tf, self.tr
        Fx_sum_prev, ax_prev, ay_prev, Fyf_prev = prev
        # suspension forces (axle totals) and moments
        zfb, zrb = z + a * th, z - b * th
        Sf = self.Kf_ax * zfb + self.Cf_ax * (zd + a * thd)
        Sr = self.Kr_ax * zrb + self.Cr_ax * (zd - b * thd)
        Mf = self.Kphf * ph + self.Cphf * phd
        Mr = self.Kphr * ph + self.Cphr * phd
        lt = -Fx_sum_prev * self.h / self.L                   # longitudinal transfer to the front
        brake = Fx_sum_prev < 0
        Jf = (self.anti["anti_dive"] if brake else 0.0) * lt
        Jr = -(self.anti["anti_lift"] if brake else self.anti["anti_squat"]) * lt
        dWu = 2 * (self.muf + self.mur) * ax_prev * self.hu / self.L
        Fyr_prev = self.m * ay_prev - Fyf_prev
        dLf = Mf / tf + Fyf_prev * self.rcf0 / tf + 2 * self.muf * ay_prev * self.hu / tf
        dLr = Mr / tr + Fyr_prev * self.rcr0 / tr + 2 * self.mur * ay_prev * self.hu / tr
        Wf = self.Wf0 + Sf + Jf - dWu
        Wr = self.Wr0 + Sr + Jr + dWu
        Fz = (max(Wf / 2 - dLf, 0.0), max(Wf / 2 + dLf, 0.0), max(Wr / 2 - dLr, 0.0), max(Wr / 2 + dLr, 0.0))
        # wheel kinematics (fl, fr, rl, rr)
        pos = ((a, tf / 2), (a, -tf / 2), (-b, tr / 2), (-b, -tr / 2))
        trav = (zfb - ph * tf / 2, zfb + ph * tf / 2, zrb - ph * tr / 2, zrb + ph * tr / 2)
        dal = [0.0] * 4
        Fxb = Fyb = Mz = 0.0
        Fy_front = 0.0
        ftire = []
        # pass 1: slip, camber and pure lateral force of every wheel
        w = []
        for i in range(4):
            xi, yi = pos[i]
            vxw, vyw = u - r * yi, vy + r * xi
            front = i < 2
            if front:
                ca, sa = math.cos(delta), math.sin(delta)
                vxw, vyw = vxw * ca + vyw * sa, -vxw * sa + vyw * ca
            ak = -math.atan2(vyw, max(abs(vxw), 0.5))
            dal[i] = max(abs(vxw), 0.5) / self.sigma * (ak - al[i])
            # camber, turn-relative (- = top leaning toward the force direction)
            cam = (self.cf0 + self.gcf * trav[i]) if front else (self.cr0 + self.gcr * trav[i])
            lean = (cam if yi > 0 else -cam) - math.degrees(ph)      # top toward +y, deg
            sf = 1.0 if al[i] >= 0 else -1.0
            fy0, Dmax = self._fy(abs(al[i]), Fz[i], -lean * sf, front)
            fy0 *= sf
            fxmax = (self.tp[7] if self.fast else self.veh.mux) * Dmax
            # friction ellipse with lateral priority (ideal traction control / ABS): the longitudinal
            # force is limited to what the slip angle leaves, so drive or braking never wipes out Fy
            rl = min(abs(fy0) / max(Dmax, 1e-6), 1.0)
            fx_lim = fxmax * max(math.sqrt(max(1.0 - rl * rl, 0.0)), 0.05)
            w.append((fy0, Dmax, fxmax, fx_lim))
        # longitudinal demand per wheel: drive shared by the rear wheels in proportion to the grip
        # they have left (locked / limited-slip diff); brakes by the fixed bias
        if Fx_cmd >= 0:
            lr_, rr_ = w[2][3], w[3][3]
            sh = lr_ / max(lr_ + rr_, 1e-6)
            fx_d = (0.0, 0.0, Fx_cmd * sh, Fx_cmd * (1 - sh))
        else:
            fx_d = (Fx_cmd * self.bias / 2, Fx_cmd * self.bias / 2,
                    Fx_cmd * (1 - self.bias) / 2, Fx_cmd * (1 - self.bias) / 2)
        for i in range(4):
            xi, yi = pos[i]
            fy0, Dmax, fxmax, fx_lim = w[i]
            fx = max(min(fx_d[i], fx_lim), -fx_lim)
            uu = min(abs(fx) / max(fxmax, 1e-6), 1.0)
            fy = math.copysign(min(abs(fy0), Dmax * math.sqrt(max(1.0 - uu * uu, 0.0))), fy0)
            if i < 2:
                fbx, fby = fx * ca - fy * sa, fx * sa + fy * ca
                Fy_front += fby
            else:
                fbx, fby = fx, fy
            Fxb += fbx; Fyb += fby
            Mz += xi * fby - yi * fbx
            ftire.append((fx, fy, Fz[i], fy0, Dmax))
        dy, info = self._body(y, Fxb, Fyb, Mz, aero, Sf, Sr, Jf, Jr, u, vy, r, ph, self.crr)
        return dy + dal, (Fz, ftire, Fxb, info[0], info[1], Fy_front, Sf, Sr)

    def _body(self, y, Fxb, Fyb, Mz, aero, Sf, Sr, Jf, Jr, u, vy, r, ph, crr):
        """Chassis accelerations from the summed tire forces (rolling resistance via crr)."""
        a, b = self.a, self.b
        zd, th, thd, phd = y[7], y[8], y[9], y[11]
        Mf = self.Kphf * ph + self.Cphf * phd
        Mr = self.Kphr * ph + self.Cphr * phd
        D, Fya, Mza, Mxa, Mcg, Dz, Fa = aero
        RR = crr * (self.m * G0 + Fa)
        du = (Fxb - D - RR) / self.m + vy * r
        dvy = (Fyb + Fya) / self.m - u * r
        dr = (Mz + Mza) / self.Iz
        ay_s = dvy + u * r
        arm = self.hs - self.h_ra
        zdd = (Fa - Sf - Sr - Jf - Jr) / self.ms
        Fground = Fxb - RR
        thdd = (Mcg - (Dz - D * self.hs) - Fground * self.hs - (Sf + Jf) * a + (Sr + Jr) * b) / self.Iy
        phdd = (self.ms * ay_s * arm + self.ms * G0 * arm * ph - Mf - Mr + Mxa) / self.Ix
        cpsi, spsi = math.cos(y[2]), math.sin(y[2])
        dy = [u * cpsi - vy * spsi, u * spsi + vy * cpsi, r, du, dvy, dr, zd, zdd, thd, thdd, phd, phdd]
        return dy, (du - vy * r, ay_s)

    # ------------------------------------------------------------------ aero
    def _aero(self, y, ax_prev, ay_prev):
        AM = self.veh.aero_model
        u, vy, r = max(y[3], 0.5), y[4], y[5]
        cpsi, spsi = math.cos(y[2]), math.sin(y[2])
        wx, wy = self.wind
        ux = u - (wx * cpsi + wy * spsi)
        uy = vy - (-wx * spsi + wy * cpsi)
        V = math.hypot(ux, uy)
        beta = math.atan2(uy, max(ux, 0.1))
        s = 1.0 if r >= 0 else -1.0
        kap = abs(r) / u
        zfb, zrb = y[6] + self.a * y[8], y[6] - self.b * y[8]
        drs = bool(AM.drs_open(ax_prev, abs(ay_prev)))
        A = AM.evaluate(np.array(V), zfb, zrb, y[10] * s, beta * s, kap, drs, detail=True)
        Fy = float(A["Fy"]) * s
        Mz = float(A["Mz"]) * s
        Mx = (float(A["Mx"]) - (float(A["Fy_z"]) - float(A["Fy"]) * self.h_ra)) * s
        out = (float(A["D"]), Fy, Mz, Mx, float(A["M_cg"]), float(A["Dz"]), float(A["F"]))
        return out, A, beta

    # ------------------------------------------------------------------ run
    def run(self, track: Track, qss=None, v0=None, name=None, t_max=None, record_every=5):
        """Drive ``track`` following the quasi-steady lap ``qss`` (LapResult). Returns TransientResult."""
        if track.x is None or track.y is None:
            raise ValueError("multibody needs a track with x, y coordinates")
        s_p, x_p, y_p = track.s, np.asarray(track.x, float), np.asarray(track.y, float)
        hd = np.unwrap(np.arctan2(np.gradient(y_p), np.gradient(x_p)))
        kp = track.kappa
        n = s_p.size
        closed = track.closed
        if qss is not None:
            v_ref_s, ax_ref_s = np.asarray(qss.v, float), np.asarray(qss.ax, float)
            sq = np.asarray(qss.s, float) - float(qss.s[0])
            sq_end = float(sq[-1])
        else:
            raise ValueError("pass the quasi-steady LapResult (speed reference)")
        lv = self.level
        dt = self.dt
        Ltot = float(s_p[-1])
        u0 = max(v0 if v0 is not None else float(v_ref_s[0]) * lv, 1.0)
        y = [float(x_p[0]), float(y_p[0]), float(hd[0]), u0, 0.0, u0 * float(kp[0]), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
             0.0, 0.0, 0.0, 0.0]
        prev = (0.0, 0.0, u0 ** 2 * float(kp[0]), self.m * u0 ** 2 * float(kp[0]) * self.b / self.L)
        idx, lap_s, t = 0, 0.0, 0.0
        I_e = 0.0
        t_max = t_max or 3.0 * (float(qss.time) if qss is not None else 120.0) + 5.0
        rec = []
        step = 0
        ds_mean = Ltot / max(n - 1, 1)
        e_prev = 0.0
        off = False
        g_scale = 1.0
        I_v = 0.0
        util_last = [0.0]
        g_min = 1.0
        while True:
            # ---- locate on the path (search a window ahead of the last index)
            lo, hi = max(idx - 5, 0), min(idx + 60, n - 1)
            if closed and idx > n - 60:
                cand = list(range(lo, n)) + list(range(0, 60))
            else:
                cand = range(lo, hi + 1)
            best, bd = idx, 1e18
            for j in cand:
                dd = (y[0] - x_p[j]) ** 2 + (y[1] - y_p[j]) ** 2
                if dd < bd:
                    bd, best = dd, j
            if closed and best < idx - n // 2:
                lap_s += Ltot
            idx = best
            s_now = lap_s + float(s_p[idx])
            nx, ny = -math.sin(hd[idx]), math.cos(hd[idx])
            e_y = (y[0] - x_p[idx]) * nx + (y[1] - y_p[idx]) * ny
            e_psi = _wrap(y[2] - hd[idx])
            u = max(y[3], 0.5)
            # ---- driver: preview steering + speed follower
            lp = self.lp_k * u + self.lp_0                    # feedback preview distance
            jp = int(idx + (self.ff_t * u + 0.5) / ds_mean)    # feed-forward curvature, ff_t seconds ahead
            jp = jp % n if closed else min(jp, n - 1)
            kap_p = float(kp[jp])
            jn = idx + int(0.5 * self.ff_t * u / ds_mean)
            kap_n = float(kp[jn % n] if closed else kp[min(jn, n - 1)])
            ay_ref = u * u * kap_n
            d_us = float(np.interp(abs(ay_ref), *self._us)) * (1 if ay_ref >= 0 else -1)
            e_p = e_y + lp * math.sin(e_psi)
            I_e = max(min(I_e + e_y * dt, 2.0), -2.0)
            de = (e_p - e_prev) / dt if step else 0.0
            e_prev = e_p
            delta = self.L * kap_p + d_us - self.kp_mult * 2.0 * self.L * e_p / lp ** 2 - 0.02 * I_e - 0.004 * de \
                + 0.10 * (u * kap_n - y[5]) * self.L / u
            delta = max(min(delta, math.radians(28)), -math.radians(28))
            s_q = float(s_p[idx]) + 0.5 + 0.05 * u                  # small look-ahead on the speed profile
            if closed:
                s_q = s_q % sq_end
            # adaptive grip margin: back off while any tire is saturated or the car runs wide
            util_prev = max(util_last)          # lateral (pure-slip) utilisation of the tires
            if util_prev > 0.995 or abs(e_y) > self.e_backoff:
                g_scale = max(g_scale - self.back_off * dt, 0.80)
            else:
                g_scale = min(g_scale + self.recover * dt, 1.0)
            v_ref = lv * g_scale * float(np.interp(s_q, sq, v_ref_s))
            ax_ff = lv * lv * float(np.interp(s_q, sq, ax_ref_s))
            aero, Adet, beta_air = self._aero(y, prev[1], prev[2])
            D, Fa = aero[0], aero[6]
            if ax_ff < 0:     # braking feed-forward only when at / above the reference speed
                ax_ff *= min(max((u - (v_ref - 1.5)) / 1.5, 0.0), 1.0)
            ev = v_ref - u
            I_v = min(max(I_v + ev * dt, -0.5), 1.5) if abs(ev) < 1.5 else I_v * (1 - 2 * dt)
            F_cmd = self.m * (ax_ff + 2.5 * ev + 1.5 * I_v) + D + self.crr * (self.m * G0 + Fa)
            F_cmd = min(F_cmd, float(np.interp(u, self._vg, self._Fd)))
            beta_now = math.atan2(y[4], max(y[3], 0.1))
            if abs(beta_now) > math.radians(self.beta_lift_deg) and F_cmd > 0:   # sliding: lift
                F_cmd = 0.0
                g_scale = max(g_scale - self.back_off * dt, 0.80)
            # ---- RK4 (driver commands, aero and the drive limit held over the step)
            y_next, info = rk4_step(lambda yy: self._rhs(yy, delta, F_cmd, aero, prev), y, dt)
            Fz, ftire, Fxb, ax_now, ay_now, Fyf, Sf, Sr = info
            prev = (Fxb, ax_now, ay_now, Fyf)
            util_last = [abs(ft[3]) / max(ft[4], 1) for ft in ftire]
            g_min = min(g_min, g_scale)
            if step % record_every == 0:
                cm = Adet.get("comps", {})
                fw = cm.get("front_wing", {})
                rw = cm.get("rear_wing", {})
                util = [math.hypot(ft[0] / max(ft[4], 1), ft[3] / max(ft[4], 1)) for ft in ftire]
                rec.append({
                    "t": t, "s": s_now, "x": y[0], "y": y[1], "v": math.hypot(y[3], y[4]), "u": y[3],
                    "beta_deg": math.degrees(math.atan2(y[4], max(y[3], 0.1))), "yaw_rate": y[5],
                    "yaw_deg": math.degrees(y[2]), "ax": ax_now, "ay": ay_now, "delta_deg": math.degrees(delta),
                    "steer_wheel_deg": math.degrees(delta) * (self.veh.cfg.steering.steer_ratio or 1.0),
                    "roll_deg": math.degrees(y[10]), "pitch_deg": math.degrees(y[8]), "heave_mm": 1e3 * y[6],
                    "roll_rate": math.degrees(y[11]), "pitch_rate": math.degrees(y[9]),
                    "zf_mm": 1e3 * (y[6] + self.a * y[8]), "zr_mm": 1e3 * (y[6] - self.b * y[8]),
                    "Fz_fl": Fz[0], "Fz_fr": Fz[1], "Fz_rl": Fz[2], "Fz_rr": Fz[3],
                    "alpha_f_deg": math.degrees(0.5 * (y[12] + y[13])), "alpha_r_deg": math.degrees(0.5 * (y[14] + y[15])),
                    "util_max": max(util), "e_lat": e_y,
                    "aero_F": float(Adet["F"]), "aero_front": float(Adet["F_front"]), "aero_rear": float(Adet["F_rear"]),
                    "aero_balance": float(Adet["balance"]), "drag": float(Adet["D"]), "aero_Mz": aero[2],
                    "aero_Fy": aero[1], "aero_Mx": aero[3], "beta_air_deg": math.degrees(beta_air),
                    "fw_yaw_deg": float(fw.get("yaw_deg", np.nan)) * (1 if y[5] >= 0 else -1),
                    "rw_yaw_deg": float(rw.get("yaw_deg", np.nan)) * (1 if y[5] >= 0 else -1),
                    "fw_clear_mm": 1e3 * float(fw.get("clearance_min", np.nan)),
                    "F_cmd": F_cmd, "grip_scale": g_scale})
            y = y_next
            t += dt
            step += 1
            if abs(e_y) > 6.0 or y[3] < 0.2 and t > 2.0:
                off = True
                break
            if s_now >= Ltot - 1.0 and (not closed or lap_s > 0 or t > 0.5 * qss.time):
                break
            if t > t_max:
                off = True
                break
        df = pd.DataFrame(rec)
        q = None
        if qss is not None:
            st = qss.state
            sq = np.asarray(qss.s, float)
            sm = np.clip(df["s"].to_numpy() % (Ltot if closed else 1e12), 0, sq[-1])
            sgn = np.sign(np.asarray(qss.ay, float))
            q = pd.DataFrame({
                "s": df["s"], "v": np.interp(sm, sq, qss.v), "t": np.interp(sm, sq, qss.t),
                "roll_deg": np.interp(sm, sq, st["roll_deg"] * np.where(sgn == 0, 1, sgn)),
                "pitch_deg": np.interp(sm, sq, st["pitch_deg"]),
                "beta_deg": np.interp(sm, sq, st["beta_deg"] * np.where(sgn == 0, 1, sgn)) if "beta_deg" in st else np.nan,
                "aero_F": np.interp(sm, sq, st["downforce"]), "ay": np.interp(sm, sq, qss.ay),
                "ax": np.interp(sm, sq, qss.ax),
                "yaw_rate": np.interp(sm, sq, np.asarray(qss.v) ** 2 * np.asarray(qss.ay) /
                                      np.maximum(np.asarray(qss.v), 0.5) ** 3)})
        summ = {"time_s": float(t), "qss_time_s": float(qss.time) if qss is not None else np.nan,
                "completed": not off, "driver_level": lv, "min_adaptive_grip_scale": g_min,
                "max_lateral_error_m": float(df["e_lat"].abs().max()),
                "max_roll_deg": float(df["roll_deg"].abs().max()), "max_pitch_deg": float(df["pitch_deg"].abs().max()),
                "max_beta_deg": float(df["beta_deg"].abs().max()), "max_yaw_rate_deg_s": float(np.degrees(df["yaw_rate"].abs().max())),
                "min_fw_clearance_mm": float(df["fw_clear_mm"].min()) if df["fw_clear_mm"].notna().any() else np.nan,
                "peak_downforce_N": float(df["aero_F"].max()),
                "aero_balance_range": (float(df["aero_balance"].quantile(0.02)), float(df["aero_balance"].quantile(0.98))),
                "Iz, Ix, Iy [kg m^2]": (round(self.Iz, 1), round(self.Ix, 1), round(self.Iy, 1))}
        if q is not None and len(df):
            for k in ("roll_deg", "pitch_deg"):
                summ[f"rms_{k}_transient_minus_qss"] = float(np.sqrt(np.mean((df[k] - q[k]) ** 2)))
        if off:
            self.notes.append(f"{name or track.name}: driver left the track / stopped at t = {t:.1f} s "
                              f"(lateral error {e_y:.1f} m) - lower driver_level")
        return TransientResult(name or track.name, df, q, summ, list(self.notes))


