"""QSS lap solver of the IC car: the powertrain stepped gear by gear (see solvers/qss.py for the method).

Speed profile
    1. lateral speed limit and braking (backward) pass from the traction GGV
       (tires + differential + suspension).
    2. forward pass with a powertrain state: current gear, shift timer, clutch.
       a = min( a_tire(v, ay),  (F_gear(v) - R(v)) / m_eq(gear) )
       Upshift at the shift rpm (optimal or configured): drive force is zero
       for upshift_time. Launch from rest with the clutch slipping at
       launch_rpm. Rev limiter in top gear. Downshifts happen in the braking
       zones (no time cost), to the gear that gives the best exit acceleration.
    3. every point is flagged with what limits it: braking, cornering,
       traction, power, shift or rev limit.

Powertrain outputs along the lap (``LapIC.pt``)
    gear, rpm, throttle, crank torque, MAP, air and fuel flow, BSFC, power
    lost in every driveline stage, chain tension, halfshaft torque and stress
    (left/right from the diff split), diff yaw moment, locking torque and
    plate slip power, electrical load / alternator / battery state, and the
    coolant and oil temperatures and fan state.

``LapIC.frame()`` returns the chassis channels (loads, roll,
pitch, camber, damper travel) side by side with the powertrain channels.
"""
from __future__ import annotations

import dataclasses as dc

import numpy as np
import pandas as pd

from lapsim.core.events import ROLL_IN, EventResults, event_points
from lapsim.core.results import LapResult
from lapsim.core.track import Track, circle, straight
from lapsim.solvers.qss import backward, lateral_speed_limit, make_tracks, skidpad_time
from lapsim.subsystems.power.ic import ICVehicle, Thermal, co2_kg, efficiency_factor, efficiency_score
LIMITS = ("braking", "cornering", "traction", "power", "shift", "rev_limit", "coast")
LIMIT_COLORS = {"braking": "#e34948", "cornering": "#2a78d6", "traction": "#eb6834", "power": "#008300",
                "shift": "#6250d6", "rev_limit": "#eda100", "coast": "#a3a29b"}


# --------------------------------------------------------------------------- tables
class ICTables:
    """Per-gear WOT force, equivalent mass and resistance on a fine speed grid (fast lookups)."""

    def __init__(self, veh: ICVehicle, coolant_temp=None, fan_on=False, dv=0.05, v_max=45.0):
        dt = veh.dt
        self.v = np.arange(0.0, v_max + dv, dv)
        self.dv = dv
        self.n = dt.n_gears
        self.F = np.stack([dt.wheel_force_wot(self.v, k, coolant_temp, fan_on) for k in range(1, self.n + 1)])
        self.meq = np.array([dt.equivalent_mass(k, veh.m) for k in range(1, self.n + 1)])
        resist = getattr(veh, "resist_drive", veh.resist)     # on power: DRS open when fitted
        self.R = resist(np.maximum(self.v, 0.0))
        self.vmax_gear = dt.top_speed_in_gear()
        self.shift_rpm = dt.shift_rpms(veh.m, lambda v: resist(v))
        self.v_shift = np.array([dt.v_at(r, k) for k, r in enumerate(self.shift_rpm, start=1)] + [self.vmax_gear[-1]])
        self.v_min = dt.v_at(veh.ptc.engine.min_drive_rpm, np.arange(1, self.n + 1))
        # equivalent mass per gear and speed (gear 1 below lock-up: clutch slipping at launch)
        self.v_lock = dt.launch_lockup_speed()
        self.meq_v = np.repeat(self.meq[:, None], self.v.size, axis=1)
        self.meq_v[0, self.v < self.v_lock] = dt.equivalent_mass(1, veh.m, clutch_slipping=True)
        A = (self.F - self.R[None, :]) / self.meq_v
        # best gear per speed respecting shift points
        Ab = np.where(self.v[None, :] <= self.v_shift[:, None] * 1.0001, A, -np.inf)
        Ab[0] = np.where(np.isfinite(Ab[0]), Ab[0], A[0])
        lrg = int(max(1, getattr(veh.ptc.gearbox, "lowest_running_gear", 1)))
        if lrg > 1:     # drivers' strategy: no gear below lrg after the launch (unless it would lug)
            hold = self.v[None, :] >= self.v_min[lrg - 1]
            Ab[:lrg - 1] = np.where(hold, -np.inf, Ab[:lrg - 1])
        self.best = 1 + np.argmax(Ab, axis=0)
        self.A = A
        env = A.max(0)
        neg = np.flatnonzero((env <= 0) & (self.v > 5))
        self.v_top = float(self.v[neg[0]]) if neg.size else float(self.v[-1])

    def idx(self, v):
        return min(int(v / self.dv + 0.5), self.v.size - 1)


# --------------------------------------------------------------------------- result
@dc.dataclass
class LapIC(LapResult):
    pt: dict = dc.field(default_factory=dict)
    limit: np.ndarray | None = None
    summary: dict = dc.field(default_factory=dict)

    def frame(self) -> pd.DataFrame:
        d = super().frame()
        for k, x in self.pt.items():
            if np.ndim(x) == 1 and len(x) == len(self.s):
                d[k] = x
        d["limit"] = self.limit
        return d


def _forward(s, k, vlim, vb, tggv, T: ICTables, v0, gear0, launch, upshift_time, n_gears):
    n = s.size
    ds = np.diff(s, append=s[-1])
    v = np.empty(n); gear = np.empty(n, int); lim = np.empty(n, object); shift = np.zeros(n, bool)
    v[0] = min(v0, vlim[0], vb[0])
    g = gear0
    timer = 0.0
    capped_prev = True
    for i in range(n - 1):
        vi = v[i]
        j = T.idx(vi)
        if capped_prev and timer <= 0:
            g = max(int(T.best[j]), 1) if vi > 0.5 else 1
        a_t = float(tggv.acc(vi, vi * vi * k[i]))
        if timer > 0:
            a_p = -T.R[j] / T.meq[g - 1]
            code = "shift"
        else:
            a_p = T.A[g - 1, j]
            code = "power"
            if g == n_gears and vi >= T.vmax_gear[-1] * 0.999:
                a_p, code = min(a_p, 0.0), "rev_limit"
            elif g < n_gears and vi >= T.vmax_gear[g - 1] * 0.999:
                a_p, code = min(a_p, 0.0), "rev_limit"
        if a_t < a_p:
            a, code = a_t, ("traction" if code == "power" else code)
        else:
            a = a_p
        vn = np.sqrt(max(vi * vi + 2 * a * ds[i], 0.01))
        capped_prev = False
        if vn >= vb[i + 1]:
            vn = vb[i + 1]
            capped_prev = True
            if vb[i + 1] < vi:
                code = "braking"
        if vn >= vlim[i + 1] * 0.9999:
            vn = vlim[i + 1]
            capped_prev = capped_prev or code not in ("shift",)
            if code not in ("braking", "shift"):
                code = "cornering"
        lim[i] = code
        gear[i] = g
        shift[i] = timer > 0
        dt_ = ds[i] / max(0.5 * (vi + vn), 0.05)
        timer = max(timer - dt_, 0.0)
        v[i + 1] = vn
        # upshift
        if timer <= 0 and g < n_gears and vn >= T.v_shift[g - 1] * 0.997 and code != "braking":
            g += 1
            timer = upshift_time
        # lugging -> instant downshift
        while g > 1 and vn < T.v_min[g - 1] and timer <= 0:
            g -= 1
    lim[-1], gear[-1], shift[-1] = lim[-2], g, timer > 0
    return v, gear, lim, shift


def solve(track: Track, veh: ICVehicle, tggv, tables: ICTables | None = None, v0: float | None = 0.0,
          flying=False, coolant_temp=None, fan_on=False) -> LapIC:
    T = tables or ICTables(veh, coolant_temp, fan_on)
    s, k = track.s, track.kappa
    vtop = min(T.v_top, float(tggv.v[-1]))
    vlim = np.minimum(lateral_speed_limit(k, tggv, vtop), vtop)
    vb = backward(s, k, vlim, tggv, vtop)
    up = veh.ptc.gearbox.upshift_time
    if flying:
        vs, gs = min(vlim[0], vb[0]), int(T.best[T.idx(vlim[0])])
        for _ in range(5):
            vb = backward(s, k, vlim, tggv, vs)
            v, gear, lim, shift = _forward(s, k, vlim, vb, tggv, T, vs, gs, False, up, T.n)
            if abs(v[-1] - vs) < 0.02:
                break
            vs, gs = v[-1], int(gear[-1])
    else:
        v, gear, lim, shift = _forward(s, k, vlim, vb, tggv, T, v0 if v0 is not None else vlim[0], 1, True, up, T.n)
    vm = np.maximum(0.5 * (v[:-1] + v[1:]), 0.05)
    t = np.r_[0, np.cumsum(np.diff(s) / vm)]
    ax = np.r_[np.diff(v ** 2) / (2 * np.diff(s)), 0.0]
    ay = v ** 2 * k
    lim = np.array([("coast" if (c == "power" and a < 0.02) else c) for c, a in zip(lim, ax)], dtype=object)
    lim[(ax < -0.3) & (lim != "shift")] = "braking"
    st = veh.state(v, ax, ay)
    pt = powertrain_channels(veh, T, v, ax, ay, k, t, gear, shift, lim, coolant_temp, fan_on)
    drive = np.maximum(veh.m * ax + veh.resist_state(st), 0.0)
    energy = float(np.sum((drive * v)[:-1] * np.diff(t)))
    lr = LapIC(track, s, v, ax, ay, t, float(t[-1]), st, energy, pt=pt, limit=lim)
    lr.summary = lap_summary(lr, veh)
    return lr


# --------------------------------------------------------------------------- channels
def powertrain_channels(veh: ICVehicle, T: ICTables, v, ax, ay, kappa, t, gear, shift, lim, coolant_temp=None,
                        fan_on=False):
    dt, eng, el = veh.dt, veh.engine, veh.elec
    pt = veh.ptc
    jv = np.clip(np.rint(v / T.dv).astype(int), 0, T.v.size - 1)
    meq = T.meq_v[gear - 1, jv]
    dstate = veh.diff_state(v, ax, ay)
    R = veh.resist_state(dstate["state"])        # drag at the solved attitude (pitch, roll, yaw, DRS)
    F_req = meq * ax + R                       # at the rear contact patches
    F_req = np.where(shift, 0.0, F_req)
    rpm = dt.rpm(v, gear)
    slip = (gear == 1) & (v < T.v_lock) & (F_req > 0)
    rpm_e = np.where(slip, pt.engine.launch_rpm, rpm)
    rpm_e = np.maximum(rpm_e, pt.engine.idle_rpm)
    # crank torque for the requested wheel torque: transmit() is affine in T
    T0, _ = dt.transmit(np.zeros_like(v), rpm, gear)
    T1, _ = dt.transmit(np.ones_like(v), rpm, gear)
    Tw_req = F_req * dt.r
    T_crank = np.where(F_req > 0, (Tw_req - T0) / np.maximum(T1 - T0, 1e-9), -eng.motoring_torque(rpm_e))
    p_load = el.load(fan_on)
    p_alt = el.output(rpm_e, p_load)
    T_alt = el.drag_torque(rpm_e, p_alt)
    T_brake = np.where(F_req > 0, T_crank + T_alt, T_crank)
    op = eng.operating_point(rpm_e, np.where(shift, 0.0, T_brake), coolant_temp)
    wot = eng.wot(rpm_e, coolant_temp)
    throttle = np.where(F_req > 0, op["throttle"], 0.0)
    throttle = np.where(np.isin(lim, ["power", "rev_limit"]), 1.0, throttle)
    m_fuel = np.where(shift, 0.0, op["m_fuel"])
    # driveline stage losses at the operating point (on power)
    Tc_pos = np.where(F_req > 0, T_crank, 0.0)
    T6, L = dt.transmit(Tc_pos, rpm, gear)
    losses = {k: np.where(F_req > 0, x, 0.0) for k, x in L.losses.items()}
    # countershaft torque & chain
    T_cs = Tc_pos * dt.primary * pt.gearbox.primary_efficiency * dt.gears[gear - 1] * pt.gearbox.mesh_efficiency
    chain = dt.chain_tension(T_cs)
    # diff split and yaw moment
    F_ri, F_ro = dstate["F_ri"], dstate["F_ro"]
    sgn = np.where(F_req > 0, 1.0, -1.0)
    hs_in, hs_out = sgn * F_ri * dt.r, sgn * F_ro * dt.r
    dw = np.abs(kappa) * veh.tr * v / dt.r                          # wheel-speed difference [rad/s]
    lockT = np.abs(F_ro - F_ri) * dt.r
    lock_cap = veh.diff.lock_torque(np.abs(F_req) * dt.r, coast=False)
    plate_slip = np.where(np.isfinite(lock_cap), lockT * dw, 0.0)
    plate_slip = np.where(veh.diff.type in ("open",), 0.0, plate_slip)
    # wheel slip ratio estimate from the tire's longitudinal stiffness
    k_s = pt.wheels.slip_stiffness_per_load
    Fz_ri, Fz_ro = dstate["state"]["Fz"]["ri"], dstate["state"]["Fz"]["ro"]
    slip_i = np.clip(np.where(F_req > 0, F_ri, -F_ri) / np.maximum(k_s * Fz_ri, 1.0), -0.3, 0.3)
    slip_o = np.clip(np.where(F_req > 0, F_ro, -F_ro) / np.maximum(k_s * Fz_ro, 1.0), -0.3, 0.3)
    out = {"gear": gear.astype(float), "rpm": rpm_e, "throttle": 100 * throttle, "shifting": shift.astype(float),
           "clutch_slip": slip.astype(float), "F_drive": F_req, "T_crank": T_brake, "T_crank_wot": wot["torque"],
           "P_engine_kW": np.maximum(T_brake, 0) * rpm_e * np.pi / 30 / 1e3, "map_kPa": np.where(F_req > 0, op["map"], 25.0) / 1e3,
           "m_air_g_s": 1e3 * np.where(F_req > 0, op["m_air"], 0.0), "m_fuel_g_s": 1e3 * m_fuel,
           "bsfc_g_kWh": np.where(F_req > 0, op["bsfc_g_kwh"], np.nan), "fuel_power_kW": m_fuel * eng.fuel.lhv / 1e3,
           "T_alternator": T_alt, "P_alternator": p_alt, "P_elec_load": np.full_like(v, float(p_load)),
           "T_countershaft": T_cs, "chain_tension": chain,
           "T_halfshaft_inner": hs_in, "T_halfshaft_outer": hs_out,
           "tau_halfshaft_MPa": dt.halfshaft_stress(np.maximum(np.abs(hs_in), np.abs(hs_out))) / 1e6,
           "diff_Mz": dstate["Mz"], "diff_dT": lockT, "diff_lock_cap": np.where(np.isfinite(lock_cap), lock_cap, np.nan),
           "diff_bias": np.where(np.minimum(np.abs(F_ri), np.abs(F_ro)) > 20,
                                 np.abs(F_ro) / np.maximum(np.abs(F_ri), 1e-6), np.nan),
           "diff_plate_slip_W": plate_slip, "slip_inner": slip_i, "slip_outer": slip_o,
           "F_rear_inner": F_ri, "F_rear_outer": F_ro}
    for k_, x in losses.items():
        out[f"loss_{k_}_W"] = x
    out["loss_total_W"] = sum(losses.values()) + plate_slip
    return out


def lap_summary(lr: LapIC, veh: ICVehicle):
    p, dt = lr.pt, np.diff(lr.t, append=lr.t[-1])
    T = max(lr.time, 1e-9)
    fuel_kg = float(np.sum(p["m_fuel_g_s"] * dt) / 1e3)
    fp = veh.engine.fuel
    vol = fuel_kg / fp.rho
    d = {"lap_time_s": lr.time, "fuel_L": vol * 1e3, "fuel_kg": fuel_kg, "co2_kg": co2_kg(vol, fp.name),
         "fuel_energy_MJ": fuel_kg * fp.lhv / 1e6, "avg_power_kW": float(np.sum(p["P_engine_kW"] * dt) / T),
         "driveline_loss_kJ": float(np.sum(p["loss_total_W"] * dt) / 1e3),
         "n_upshifts": int(np.sum(np.diff(p["gear"]) > 0)), "n_downshifts": int(np.sum(np.diff(p["gear"]) < 0)),
         "max_rpm": float(p["rpm"].max()), "peak_halfshaft_Nm": float(np.nanmax(np.abs([p["T_halfshaft_inner"], p["T_halfshaft_outer"]]))),
         "peak_chain_N": float(p["chain_tension"].max())}
    for c in LIMITS:
        d[f"time_{c}_%"] = 100 * float(dt[lr.limit == c].sum() / T)
    for g in range(1, veh.dt.n_gears + 1):
        d[f"time_gear{g}_%"] = 100 * float(dt[p["gear"] == g].sum() / T)
    return d


# --------------------------------------------------------------------------- events
def accel_run(veh, tggv, T=None, length=75.0):
    """Standing start; ``summary['fsae_time_s']`` is timed from the start line (0.30 m roll-in)."""
    r = solve(straight(length + ROLL_IN, 0.1), veh, tggv, T, v0=0.0)
    t0 = float(np.interp(ROLL_IN, r.s, r.t))
    r.summary["fsae_time_s"] = r.time - t0
    r.summary["time_0_75m_s"] = float(np.interp(length, r.s, r.t))
    r.summary["trap_speed_kph"] = 3.6 * float(r.v[-1])
    r.summary["time_0_100kph_s"] = float(np.interp(100 / 3.6, r.v, r.t)) if r.v.max() > 100 / 3.6 else np.nan
    return r


def run_events(cfg, pt, tracks=None, events=("acceleration", "skidpad", "autocross", "endurance"),
                  tggv=None, veh=None, thermal=True, driver_change_s=180.0, verbose=False) -> EventResults:
    veh = veh or ICVehicle(cfg, pt)
    tggv = tggv or veh.traction_ggv()
    T = ICTables(veh)
    ev = cfg.events
    times, laps, notes = {}, {}, list(veh.warnings)
    tracks = dict(tracks or make_tracks(cfg, cfg.sim.ds))
    end_df = None
    if "acceleration" in events:
        r = accel_run(veh, tggv, T, ev.accel_length)
        times["acceleration"], laps["acceleration"] = r.summary["fsae_time_s"], r
    if "skidpad" in events:
        tl, vs = skidpad_time(veh, tggv, ev.skidpad_radius)
        times["skidpad"] = tl
        laps["skidpad"] = solve(circle(ev.skidpad_radius), veh, tggv, T, flying=True)
    if "autocross" in events and "autocross" in tracks:
        r = solve(tracks["autocross"], veh, tggv, T, v0=0.0)
        times["autocross"], laps["autocross"] = r.time, r
    elif "autocross" in events:
        notes.append("autocross: no track (events.autocross_track)")
    if "endurance" in events and "endurance" in tracks:
        end_df, end_laps = endurance(veh, tggv, tracks["endurance"], ev.endurance_distance, thermal=thermal,
                                     driver_change_s=driver_change_s, verbose=verbose)
        times["endurance"] = float(end_df["lap_time_s"].sum())
        laps["endurance"] = end_laps[1] if len(end_laps) > 1 else end_laps[0]
        laps["endurance_first"] = end_laps[0]
        laps["endurance_last"] = end_laps[-1]
        laps["endurance_all"] = end_laps
    elif "endurance" in events:
        notes.append("endurance: no track (events.endurance_track)")
    pts = event_points(ev, times)
    if end_df is not None:
        sc = pt.scoring
        n = len(end_df)
        t_lap = times["endurance"] / n
        co2_lap = float(end_df["co2_kg"].sum()) / n
        t_min_lap = sc.t_min_lap or (ev.tmin_endurance / n if ev.tmin_endurance else None)
        if t_min_lap:
            ef = efficiency_factor(t_lap, co2_lap, t_min_lap, sc.co2_min_per_lap)
            pts["efficiency"] = efficiency_score(ef, sc.ef_min, sc.ef_max)
            notes.append(f"efficiency factor {ef:.3f} (scoring constants ASSUMED - check the rules year)")
    return EventResults(times, pts, laps, end_df, tggv, veh, T, notes)


def endurance(veh: ICVehicle, tggv, track: Track, distance=22000.0, thermal=True, driver_change_s=180.0,
              verbose=False):
    """Lap-by-lap endurance: coolant/oil temperature carried from lap to lap, the engine derated
    when hot, fuel and battery state integrated, driver change at half distance (engine off)."""
    pt = veh.ptc
    n = max(1, int(round(distance / track.length)))
    th = Thermal(pt)
    s = th.init_state()
    soc = pt.electrical.start_soc
    fuel_left = pt.fuel.start_fill
    rows, laps = [], []
    t_total = 0.0
    for i in range(n):
        Tc = s.Tc if thermal else None
        T = ICTables(veh, coolant_temp=Tc, fan_on=s.fan)
        lr = solve(track, veh, tggv, T, v0=0.0 if i in (0, n // 2) else None, flying=i not in (0, n // 2),
                      coolant_temp=Tc, fan_on=s.fan)
        p = lr.pt
        dtv = np.diff(lr.t, append=lr.t[-1])
        Tc_tr = np.empty_like(lr.t); To_tr = np.empty_like(lr.t); fan_tr = np.empty_like(lr.t)
        q_rad = np.empty_like(lr.t)
        for j in range(lr.t.size):
            if thermal:
                s, q = th.step(s, dtv[j], lr.v[j], p["rpm"][j], p["fuel_power_kW"][j] * 1e3)
                q_rad[j] = q["Q_rad"]
            Tc_tr[j], To_tr[j], fan_tr[j] = s.Tc, s.To, s.fan
            p_load = veh.elec.load(s.fan)
            p_alt = veh.elec.output(p["rpm"][j], p_load + (20.0 if soc < 0.98 else 0.0))
            soc = veh.elec.battery_step(soc, p_alt, p_load, dtv[j])
        p["coolant_C"], p["oil_C"], p["fan"] = Tc_tr, To_tr, fan_tr
        p["Q_radiator_kW"] = q_rad / 1e3
        p["t_endurance"] = t_total + lr.t
        lr.summary = lap_summary(lr, veh)
        fuel_left -= lr.summary["fuel_L"] / 1e3
        t_total += lr.time
        row = {"lap": i + 1, **lr.summary, "coolant_end_C": s.Tc, "oil_end_C": s.To, "fan_on_%": 100 * fan_tr.mean(),
               "derate_%": 100 * (1 - float(veh.engine.derate(Tc_tr.mean()))) if thermal else 0.0,
               "battery_soc": soc, "fuel_left_L": fuel_left * 1e3, "elapsed_s": t_total}
        rows.append(row)
        laps.append(lr)
        if verbose:
            print(f"lap {i + 1:2d}: {lr.time:6.2f} s  coolant {s.Tc:5.1f} C  fuel left {fuel_left * 1e3:4.2f} L")
        if i == n // 2 - 1 and driver_change_s and thermal:
            # engine off: heat soak, no pump, no fan
            for _ in range(int(driver_change_s)):
                s.Tc -= 4.0 * (s.Tc - th.T_amb) / pt.cooling.thermal_mass_coolant
                s.To += pt.cooling.oil_to_coolant_ua * 0.2 * (s.Tc - s.To) / pt.cooling.thermal_mass_oil
    df = pd.DataFrame(rows).set_index("lap")
    if (df["fuel_left_L"] < 0).any():
        df.attrs["out_of_fuel_lap"] = int(df.index[df["fuel_left_L"] < 0][0])
    return df, laps
