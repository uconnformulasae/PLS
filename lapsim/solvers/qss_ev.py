"""QSS lap solver of the EV car: speed profile, powertrain and accumulator channels, the four events.

Speed profile (the method of solvers/qss.py)
    1. lateral speed limit and braking (backward) pass from the traction GGV (tires + diff + suspension);
    2. forward pass:  a = min( a_tire(v, ay),  (F_motor(v) - R(v)) / m_eq )   with F_motor from EVTables
       (motor torque inside the accumulator, inverter and rules limits at the lap's SOC and temperatures);
    3. every point is flagged with what limits it (LIMITS).

Channels along the lap (``LapEV.pt``): motor speed / torque / phase current, mechanical, dc-link and
accumulator power, losses by component, regen and friction-brake power, pack current, voltage (with
the RC sag), SOC, cell and motor temperature, wheel forces and the differential.

``LapEV.frame()`` puts them next to the chassis channels (loads, roll, pitch, camber, damper travel).
"""
from __future__ import annotations

import dataclasses as dc

import numpy as np
import pandas as pd

from lapsim.core.events import ROLL_IN, EventResults, event_points, points
from lapsim.core.results import LapResult
from lapsim.core.track import Track, circle, straight
from lapsim.solvers.qss import backward, lateral_speed_limit, make_tracks, skidpad_time
from lapsim.subsystems.power.battery import PackState
from lapsim.subsystems.power.ev import CODES, EVTables, EVVehicle

LIMITS = ("braking", "cornering", "traction", "torque", "power", "voltage", "battery", "top_speed", "coast")
LIMIT_COLORS = {"braking": "#e34948", "cornering": "#2a78d6", "traction": "#eb6834", "torque": "#6250d6",
                "power": "#008300", "voltage": "#eda100", "battery": "#e87ba4", "top_speed": "#1baf7a",
                "coast": "#a3a29b"}
LIMIT_HELP = {"braking": "tires, braking", "cornering": "tires, cornering", "traction": "rear tires on power",
              "torque": "motor / inverter torque", "power": "accumulator power limit",
              "voltage": "pack voltage (field weakening)", "battery": "cell current / voltage",
              "top_speed": "motor max rpm", "coast": "coasting"}


@dc.dataclass
class LapEV(LapResult):
    pt: dict = dc.field(default_factory=dict)
    limit: np.ndarray | None = None
    summary: dict = dc.field(default_factory=dict)
    end_state: PackState | None = None

    def frame(self) -> pd.DataFrame:
        d = super().frame()
        for k, x in self.pt.items():
            if np.ndim(x) == 1 and len(x) == len(self.s):
                d[k] = x
        d["limit"] = self.limit
        return d


# --------------------------------------------------------------------------- speed profile
def _forward(s, k, vlim, vb, tggv, T: EVTables, v0):
    n = s.size
    ds = np.diff(s, append=s[-1])
    v = np.empty(n)
    lim = np.empty(n, object)
    v[0] = min(v0, vlim[0], vb[0])
    A, code = T.A, T.code
    for i in range(n - 1):
        vi = v[i]
        j = T.idx(vi)
        a_t = float(tggv.acc(vi, vi * vi * k[i]))
        a_p = A[j]
        c = CODES[code[j]]
        if vi >= T.v_top * 0.999:
            a_p, c = min(a_p, 0.0), ("top_speed" if c == "top_speed" or a_p > 0 else c)
        if a_t < a_p:
            a, c = a_t, "traction"
        else:
            a = a_p
        vn = np.sqrt(max(vi * vi + 2 * a * ds[i], 0.01))
        if vn >= vb[i + 1]:
            vn = vb[i + 1]
            if vb[i + 1] < vi:
                c = "braking"
        if vn >= vlim[i + 1] * 0.9999:
            vn = vlim[i + 1]
            if c != "braking":
                c = "cornering"
        lim[i] = c
        v[i + 1] = vn
    lim[-1] = lim[-2]
    return v, lim


def solve(track: Track, veh: EVVehicle, tggv, tables: EVTables | None = None, v0: float | None = 0.0,
             flying=False, state: PackState | None = None) -> LapEV:
    """One pass over a track. ``state`` is the accumulator / motor state at the start (default: the
    config's start SOC and temperatures); the state at the end is ``result.end_state``."""
    state = state or (tables.state if tables is not None else veh.ep.initial_state())
    T = tables or veh.tables(state)
    s, k = track.s, track.kappa
    vtop = min(T.v_top, float(tggv.v[-1]))
    vlim = np.minimum(lateral_speed_limit(k, tggv, vtop), vtop)
    vb = backward(s, k, vlim, tggv, vtop)
    if flying:
        vs = min(vlim[0], vb[0])
        for _ in range(5):
            vb = backward(s, k, vlim, tggv, vs)
            v, lim = _forward(s, k, vlim, vb, tggv, T, vs)
            if abs(v[-1] - vs) < 0.02:
                break
            vs = v[-1]
    else:
        v, lim = _forward(s, k, vlim, vb, tggv, T, v0 if v0 is not None else vlim[0])
    vm = np.maximum(0.5 * (v[:-1] + v[1:]), 0.05)
    t = np.r_[0, np.cumsum(np.diff(s) / vm)]
    ax = np.r_[np.diff(v ** 2) / (2 * np.diff(s)), 0.0]
    ay = v ** 2 * k
    lim = np.array([("coast" if (c in CODES and a < 0.02) else c) for c, a in zip(lim, ax)], dtype=object)
    lim[ax < -0.3] = "braking"
    pt, st, end = ev_channels(veh, T, v, ax, ay, t, state)
    drive = np.maximum(veh.m * ax + veh.resist_state(st), 0.0)
    energy = float(np.sum((drive * v)[:-1] * np.diff(t)))
    lr = LapEV(track, s, v, ax, ay, t, float(t[-1]), st, energy, pt=pt, limit=lim, end_state=end)
    lr.summary = lap_summary(lr, veh)
    return lr


# --------------------------------------------------------------------------- channels
def ev_channels(veh: EVVehicle, T: EVTables, v, ax, ay, t, state: PackState):
    ep, m, evc = veh.ep, veh.ep.motor, veh.evc
    jv = np.clip(np.rint(v / T.dv).astype(int), 0, T.v.size - 1)
    ds_ = veh.diff_state(v, ax, ay)
    st = ds_["state"]
    R = veh.resist_state(st)
    F_req = T.meq * ax + R                                   # at the rear contact patches (+ drive, - brake)
    drive = F_req > 0
    bias = veh.bias if veh.bias is not None else 0.6
    brake = np.maximum(-F_req, 0.0)
    F_regen = np.minimum(brake * (1 - bias) * evc.regen.rear_brake_share, T.F_regen[jv])
    T_drive = np.minimum(np.maximum(F_req, 0.0) * ep.r / (ep.G * ep.eta), T.T[jv])
    rpm = ep.rpm(v)
    w = rpm * np.pi / 30
    i_chg = ep.pack.charge_current_limit(state.soc, state.temp, evc.regen.max_soc)
    # up to three passes: the tables limit regen for the pack at rest; with the RC sag left by the previous
    # acceleration the same power needs more current, so regen is trimmed where the cells' charge
    # limit would be exceeded and the friction brakes take the difference
    for it in range(3):
        F_fric_r = brake * (1 - bias) - F_regen
        T_m = np.where(drive, T_drive, -F_regen * ep.r * ep.eta / ep.G)
        cu, iron, inv = m.losses(T_m, rpm, state.motor_temp)
        P_mech = T_m * w
        P_dc = P_mech + cu + iron + inv
        P_acc = P_dc + ep.p_aux
        pk, end = ep.pack.simulate(t, P_acc, state)
        over = -pk["I_pack"] > i_chg
        if it == 2 or not over.any():
            break
        F_regen = np.where(over, F_regen * 0.995 * i_chg / np.maximum(-pk["I_pack"], 1e-9), F_regen)
    F_fric_f = brake * bias
    loss_dl = np.where(drive, P_mech * (1 - ep.eta), F_regen * v * (1 - ep.eta))
    # motor winding temperature (lumped)
    mc = evc.motor
    Tm = np.empty_like(t)
    x = state.motor_temp
    dtv = np.diff(t, append=t[-1])
    heat = cu + iron
    for i in range(t.size):
        Tm[i] = x
        x += dtv[i] * (heat[i] - mc.ua_coolant * (x - mc.coolant_temp)) / mc.thermal_mass
    end.motor_temp = float(x)
    F_ri, F_ro = ds_["F_ri"], ds_["F_ro"]
    axle_T = np.where(drive, F_req, -F_regen) * ep.r
    rolling = (veh.crr + veh.toe_drag_coef) * (veh.m * 9.80665 + st["downforce"])
    out = {
        "motor_rpm": rpm, "motor_torque": T_m, "motor_torque_max": T.T[jv], "regen_torque_max": T.T_regen[jv],
        "I_phase": np.abs(T_m) / mc.kt, "P_mech_kW": P_mech / 1e3, "P_dc_kW": P_dc / 1e3, "P_acc_kW": P_acc / 1e3,
        "P_regen_kW": np.where(P_dc < 0, -P_dc, 0.0) / 1e3,
        "throttle": 100 * np.clip(np.where(drive, T_drive / np.maximum(T.T[jv], 1e-6), 0.0), 0, 1),
        "F_drive": np.maximum(F_req, 0.0), "F_regen": F_regen, "F_brake_front": F_fric_f, "F_brake_rear": F_fric_r,
        "loss_copper_W": cu, "loss_iron_W": iron, "loss_inverter_W": inv, "loss_driveline_W": loss_dl,
        "loss_pack_W": pk["cell_heat_W"] * ep.pack.ns * ep.pack.npar + pk["I_pack"] ** 2 * ep.pack.r_ic,
        "P_aux_W": np.full_like(v, ep.p_aux), "P_brake_front_kW": F_fric_f * v / 1e3,
        "P_brake_rear_kW": F_fric_r * v / 1e3, "P_drag_kW": st["drag"] * v / 1e3,
        "P_rolling_kW": (rolling + st.get("cornering_drag", 0.0)) * v / 1e3,
        "motor_temp_C": Tm, "axle_torque": axle_T, "chain_tension": ep.chain_tension(axle_T),
        "T_halfshaft_inner": F_ri * ep.r, "T_halfshaft_outer": F_ro * ep.r,
        "F_rear_inner": F_ri, "F_rear_outer": F_ro, "diff_Mz": ds_["Mz"],
        "motor_efficiency": np.where(P_mech > 100, P_mech / np.maximum(P_dc, 1e-9),
                                     np.where(P_dc < -100, P_dc / np.minimum(P_mech, -1e-9), np.nan)),
    }
    out.update(pk)
    out["E_acc_kWh"] = np.r_[0.0, np.cumsum((pk["V_pack"] * pk["I_pack"])[:-1] * np.diff(t))] / 3.6e6
    return out, st, end


ENERGY_PARTS = (("aerodynamic drag", "P_drag_kW", 1e3), ("rolling + cornering drag", "P_rolling_kW", 1e3),
                ("friction brakes", None, 1.0), ("motor (copper + iron)", None, 1.0),
                ("inverter", "loss_inverter_W", 1.0), ("driveline", "loss_driveline_W", 1.0),
                ("accumulator heat", "loss_pack_W", 1.0), ("LV auxiliaries", "P_aux_W", 1.0))


def energy_breakdown(lr: LapEV) -> pd.Series:
    """Where the energy drawn from the cells goes over the lap [kWh] (regen already returned)."""
    p, dt = lr.pt, np.diff(lr.t, append=lr.t[-1])
    d = {}
    for name, ch, k in ENERGY_PARTS:
        if name == "friction brakes":
            x = (p["P_brake_front_kW"] + p["P_brake_rear_kW"]) * 1e3
        elif name.startswith("motor"):
            x = p["loss_copper_W"] + p["loss_iron_W"]
        else:
            x = p[ch] * k
        d[name] = float(np.sum(x * dt)) / 3.6e6
    return pd.Series(d)


def lap_summary(lr: LapEV, veh: EVVehicle) -> dict:
    p, dt = lr.pt, np.diff(lr.t, append=lr.t[-1])
    Tt = max(lr.time, 1e-9)
    pa = p["V_pack"] * p["I_pack"]
    e_out = float(np.sum(np.maximum(pa, 0) * dt)) / 3.6e6
    e_in = float(np.sum(np.maximum(-pa, 0) * dt)) / 3.6e6
    dist = float(lr.s[-1]) / 1e3
    d = {"lap_time_s": lr.time, "energy_kWh": e_out - e_in, "energy_out_kWh": e_out, "regen_kWh": e_in,
         "Wh_per_km": 1e3 * (e_out - e_in) / max(dist, 1e-9), "avg_power_kW": float(np.sum(pa * dt) / Tt) / 1e3,
         "peak_power_kW": float(pa.max()) / 1e3, "peak_current_A": float(p["I_pack"].max()),
         "peak_regen_current_A": float(max(-p["I_pack"].min(), 0.0)), "min_pack_V": float(p["V_pack"].min()),
         "min_cell_V": float(p["V_cell"].min()), "soc_start": float(p["soc"][0]), "soc_end": float(lr.end_state.soc),
         "cell_temp_end_C": float(lr.end_state.temp), "motor_temp_end_C": float(lr.end_state.motor_temp),
         "peak_motor_torque_Nm": float(p["motor_torque"].max()),
         "rms_motor_torque_Nm": float(np.sqrt(np.sum(p["motor_torque"] ** 2 * dt) / Tt)),
         "max_motor_rpm": float(p["motor_rpm"].max()), "top_speed_kph": 3.6 * float(lr.v.max()),
         "peak_axle_torque_Nm": float(np.abs(p["axle_torque"]).max()),
         "power_unmet_%": 100 * float(dt[p["power_unmet"]].sum() / Tt)}
    for c in LIMITS:
        d[f"time_{c}_%"] = 100 * float(dt[lr.limit == c].sum() / Tt)
    return d


# --------------------------------------------------------------------------- events
def accel_run(veh, tggv, T=None, length=75.0):
    """Standing start; ``summary['fsae_time_s']`` is timed from the start line (0.30 m roll-in)."""
    r = solve(straight(length + ROLL_IN, 0.1), veh, tggv, T, v0=0.0)
    t0 = float(np.interp(ROLL_IN, r.s, r.t))
    r.summary["fsae_time_s"] = r.time - t0
    r.summary["trap_speed_kph"] = 3.6 * float(r.v[-1])
    r.summary["time_0_100kph_s"] = float(np.interp(100 / 3.6, r.v, r.t)) if r.v.max() > 100 / 3.6 else np.nan
    return r


def efficiency(evc, e_lap_J, t_lap, t_min_lap):
    """(efficiency factor, points) or (None, None) when the scoring constants are not set."""
    sc = evc.scoring
    if not (sc.energy_min_per_lap and t_min_lap and e_lap_J > 0):
        return None, None
    ef = (t_min_lap / t_lap) * (sc.energy_min_per_lap / e_lap_J)
    return ef, float(np.clip(100 * (ef - sc.ef_min) / (sc.ef_max - sc.ef_min), 0, 100))


def run_events(cfg, evc, bcfg, tracks=None, events=("acceleration", "skidpad", "autocross", "endurance"),
                  tggv=None, veh=None, verbose=False) -> EventResults:
    veh = veh or EVVehicle(cfg, evc, bcfg)
    tggv = tggv or veh.traction_ggv()
    T = veh.tables0
    ev = cfg.events
    times, laps, notes = {}, {}, list(veh.warnings)
    tracks = dict(tracks or make_tracks(cfg, cfg.sim.ds))
    end_df = None
    if "acceleration" in events:
        r = accel_run(veh, tggv, T, ev.accel_length)
        times["acceleration"], laps["acceleration"] = r.summary["fsae_time_s"], r
    if "skidpad" in events:
        times["skidpad"] = skidpad_time(veh, tggv, ev.skidpad_radius)[0]
        laps["skidpad"] = solve(circle(ev.skidpad_radius), veh, tggv, T, flying=True)
    if "autocross" in events and "autocross" in tracks:
        r = solve(tracks["autocross"], veh, tggv, T, v0=0.0)
        times["autocross"], laps["autocross"] = r.time, r
    elif "autocross" in events:
        notes.append("autocross: no track (events.autocross_track)")
    if "endurance" in events and "endurance" in tracks:
        end_df, end_laps = endurance(veh, tggv, tracks["endurance"], ev.endurance_distance, verbose=verbose)
        times["endurance"] = float(end_df["lap_time_s"].sum())
        laps["endurance"] = end_laps[1] if len(end_laps) > 1 else end_laps[0]
        laps["endurance_all"] = end_laps
        for key, msg in (("out_of_energy_lap", "SOC falls below accumulator.soc_min"),
                         ("over_temp_lap", "cell temperature reaches the limit"),
                         ("dnf_lap", "accumulator empty - the car does NOT finish")):
            if key in end_df.attrs:
                notes.append(f"endurance: {msg} on lap {end_df.attrs[key]}")
    elif "endurance" in events:
        notes.append("endurance: no track (events.endurance_track)")
    pts = event_points(ev, times)
    if end_df is not None:
        n = len(end_df)
        t_min_lap = evc.scoring.t_min_lap or (ev.tmin_endurance / n if ev.tmin_endurance else None)
        ef, p = efficiency(evc, 3.6e6 * float(end_df["energy_kWh"].sum()) / n, times["endurance"] / n, t_min_lap)
        if ef is not None:
            pts["efficiency"] = p
            notes.append(f"efficiency factor {ef:.3f} (scoring constants ASSUMED - check the rules year)")
    return EventResults(times, pts, laps, end_df, tggv, veh, T, notes)


def endurance(veh: EVVehicle, tggv, track: Track, distance=22000.0, p_limit=None, driver_change_s=None,
              keep_laps=True, verbose=False):
    """Lap-by-lap endurance: SOC, RC sag, cell and motor temperature carried from lap to lap; the
    drive-force tables are rebuilt every lap, so the car slows as the pack voltage falls and as the
    cells or the motor derate. Driver change at half distance (standing restart).
    Returns (per-lap DataFrame, list of laps). df.attrs flags out_of_energy_lap / over_temp_lap / dnf_lap."""
    evc, ep = veh.evc, veh.ep
    n = max(1, int(round(distance / track.length)))
    p_limit = p_limit if p_limit is not None else ep.power_limit(endurance=True)
    change = evc.endurance.driver_change_s if driver_change_s is None else driver_change_s
    state = ep.initial_state()
    rows, laps = [], []
    t_total = e_total = 0.0
    attrs = {}
    for i in range(n):
        T = veh.tables(state, p_limit)
        standing = i in (0, n // 2)
        lr = solve(track, veh, tggv, T, v0=0.0 if standing else None, flying=not standing, state=state)
        state = lr.end_state
        t_total += lr.time
        e_total += lr.summary["energy_kWh"]
        lr.pt["t_endurance"] = t_total - lr.time + lr.t
        row = {"lap": i + 1, **lr.summary, "elapsed_s": t_total, "energy_total_kWh": e_total,
               "cell_derate_%": 100 * (1 - ep.pack.temp_derate(state.temp)),
               "motor_derate_%": 100 * (1 - ep.motor.derate(state.motor_temp))}
        rows.append(row)
        if keep_laps:
            laps.append(lr)
        if verbose:
            print(f"lap {i + 1:2d}: {lr.time:6.2f} s  SOC {100 * state.soc:5.1f} %  cells {state.temp:4.1f} C  "
                  f"{lr.summary['energy_kWh']:.3f} kWh")
        if state.soc < evc.accumulator.soc_min:
            attrs.setdefault("out_of_energy_lap", i + 1)
        if state.temp >= ep.pack.lim.cell_t_max:
            attrs.setdefault("over_temp_lap", i + 1)
        if state.soc <= 0.0 or lr.summary["min_cell_V"] < ep.pack.v_min * 0.98:
            attrs["dnf_lap"] = i + 1
            break
        if i == n // 2 - 1 and change:
            state = ep.pack.rest(state, change)
            mc = evc.motor
            state.motor_temp = mc.coolant_temp + (state.motor_temp - mc.coolant_temp) * float(
                np.exp(-change * mc.ua_coolant / mc.thermal_mass))
    df = pd.DataFrame(rows).set_index("lap")
    df.attrs.update(attrs, n_laps=n, p_limit_W=p_limit, finished="dnf_lap" not in attrs)
    return df, laps


# --------------------------------------------------------------------------- studies
def energy_sweep(veh: EVVehicle, tggv, track: Track, power_limits_kW=(80, 70, 60, 50, 40, 30), distance=22000.0,
                 tmin=None):
    """Endurance time against energy for a range of accumulator power limits: the fastest limit
    that still finishes above the SOC reserve is the endurance strategy."""
    rows = []
    for p in power_limits_kW:
        df, _ = endurance(veh, tggv, track, distance, p_limit=p * 1e3, keep_laps=False)
        fin = df.attrs["finished"] and len(df) == df.attrs["n_laps"]
        t = float(df["lap_time_s"].sum())
        rows.append({"power_limit_kW": p, "time_s": t if fin else np.nan, "laps_completed": len(df),
                     "energy_kWh": float(df["energy_kWh"].sum()), "regen_kWh": float(df["regen_kWh"].sum()),
                     "final_soc_%": 100 * float(df["soc_end"].iloc[-1]), "min_cell_V": float(df["min_cell_V"].min()),
                     "max_cell_temp_C": float(df["cell_temp_end_C"].max()),
                     "max_motor_temp_C": float(df["motor_temp_end_C"].max()),
                     "finishes": bool(fin), "above_reserve": bool(fin and "out_of_energy_lap" not in df.attrs),
                     "points": points("endurance", t, tmin) if (tmin and fin) else np.nan})
    return pd.DataFrame(rows).set_index("power_limit_kW")


def gearing_sweep(cfg, evc, bcfg, tggv, tracks, ratios=None):
    """Acceleration and autocross time, top speed and autocross energy for each reduction ratio."""
    rows = []
    for g in ratios or evc.drivetrain.ratio_options:
        e = evc.copy()
        e.drivetrain.ratio = float(g)
        veh = EVVehicle(cfg, e, bcfg)
        T = veh.tables0
        acc = accel_run(veh, tggv, T, cfg.events.accel_length)
        row = {"ratio": float(g), "acceleration_s": acc.summary["fsae_time_s"],
               "trap_speed_kph": acc.summary["trap_speed_kph"], "top_speed_kph": 3.6 * T.v_top}
        if "autocross" in tracks:
            a = solve(tracks["autocross"], veh, tggv, T, v0=0.0)
            row.update(autocross_s=a.time, autocross_Wh_per_km=a.summary["Wh_per_km"],
                       autocross_traction_limited_pct=a.summary["time_traction_%"])
        rows.append(row)
    return pd.DataFrame(rows).set_index("ratio")
