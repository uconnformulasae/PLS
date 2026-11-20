"""IC power module: restricted engine, fuel, cooling, low-voltage electrics and the IC car.

``ICVehicle`` is the chassis (``core.vehicle.Vehicle``) with the engine, gearbox, chain and limited-slip
differential (``subsystems.driveline``): the drive-force limit is the best-gear WOT force reduced by the
gear's rotating inertia, and under braking engine braking loads the coast side of the differential.

Engine
    Restricted Yamaha R6 engine model (mean-value, quasi-steady).

    Air path
        ambient (p0, T0) -> 20 mm restrictor venturi -> plenum (p_m, T_m) -> cylinders.
        Restrictor: isentropic nozzle flow at the throat with discharge coefficient Cd.
        The diffuser recovers part of the throat dynamic pressure, so the throat
        chokes while the plenum is still at ~0.8-0.9 p0 (as logged: MAP 0.74-0.85 bar
        at WOT and high rpm). Choked flow  m* = Cd A p0 sqrt(g/(R T0)) (2/(g+1))^((g+1)/(2(g-1))).
        Engine demand  m_e = VE(N) * p_m/(R T_m) * Vd * N/120.
        The plenum pressure is the p_m at which the two flows are equal.

    Torque
        indicated power  = eta_ind * eta_lambda * (m_air/AFR) * LHV * min(1, 1/lambda)
        friction (Chen-Flynn form)  FMEP = a + b*Sp + c*Sp^2   [kPa], Sp mean piston speed
        pumping  PMEP = p_exhaust - p_plenum
        brake torque = (P_ind/omega) - (FMEP + PMEP) * Vd / (4 pi), times the
        coolant-temperature derate.

    Part load is solved for the air flow that gives the requested brake torque,
    which gives fuel flow and BSFC everywhere on the map. Everything is
    vectorised over rpm.

Cooling
    Engine cooling: two lumped thermal nodes (coolant+block, oil) and the radiator.

        C_c dTc/dt = f_c * P_fuel - Q_rad - Q_conv + UA_oc (To - Tc)
        C_o dTo/dt = f_o * P_fuel - UA_oc (To - Tc)

    Radiator: crossflow, both fluids unmixed, effectiveness-NTU.
        air flow   V_air = duct_ratio * v * A_core  (+ fan_flow when the fan is on)
        UA         = ua_ref * (V_air / air_flow_ref)^0.6  (air-side film coefficient ~ Re^0.6)
        coolant    m_c = pump_flow_per_krpm * rpm/1000 * thermostat opening
        Q_rad      = eps * C_min * (Tc - T_amb)
    The fan switches on at fan_on_temp and off at fan_off_temp (hysteresis) and its
    electrical power goes to the electrical model. Above derate_start_temp the
    engine model loses torque (ECU timing pull).

Electrics
    Low-voltage electrical system: alternator (stator + regulator), loads, battery.

    The alternator's electrical output is taken from the crank:
        T_drag = T_friction + P_out / (eta_alt * omega)
    so electrical load costs engine torque and fuel. The battery covers any deficit
    (at low rpm, or with the fan on) and is recharged when there is spare output.
"""
from __future__ import annotations

import dataclasses as dc

import numpy as np

from lapsim.config.schema import CarConfig
from lapsim.config.schema_ic import PTConfig
from lapsim.core.vehicle import Vehicle
from lapsim.subsystems.driveline import Drivetrain, omega


# --------------------------------------------------------------------------- fuel
# LHV [J/kg], stoichiometric AFR, density [kg/m^3], CO2 [kg per litre] (FSAE rules conversion)
FUELS = {
    "gasoline": {"lhv": 43.0e6, "afr": 14.7, "rho": 740.0, "co2_per_l": 2.31, "restrictor_rule": 0.020},
    "e85": {"lhv": 29.2e6, "afr": 9.8, "rho": 785.0, "co2_per_l": 1.65, "restrictor_rule": 0.019},
}


@dc.dataclass
class FuelProps:
    name: str
    lhv: float
    afr: float
    rho: float
    co2_per_l: float
    restrictor_rule: float


def props(fuel_type: str) -> FuelProps:
    t = (fuel_type or "gasoline").lower()
    if t not in FUELS:
        raise ValueError(f"fuel.type must be one of {list(FUELS)}")
    return FuelProps(t, **FUELS[t])


def co2_kg(volume_m3: float, fuel_type: str) -> float:
    return volume_m3 * 1e3 * props(fuel_type).co2_per_l


def efficiency_factor(t_lap, co2_lap, t_min_lap, co2_min_lap):
    """FSAE efficiency factor per lap: (Tmin/T) * (CO2min/CO2)."""
    return (t_min_lap / t_lap) * (co2_min_lap / co2_lap)


def efficiency_score(ef, ef_min, ef_max, max_points=100.0):
    """Linear between the event's worst (ef_min) and best (ef_max) efficiency factors.
    The constants change between rules years: set them in scoring.* of the powertrain config."""
    return float(np.clip(max_points * (ef - ef_min) / max(ef_max - ef_min, 1e-9), 0.0, max_points))


# --------------------------------------------------------------------------- engine
R_AIR, GAMMA = 287.05, 1.4
_CRIT = (2 / (GAMMA + 1)) ** (GAMMA / (GAMMA - 1))          # p*/p0 = 0.528
_PSI_STAR = (2 / (GAMMA + 1)) ** ((GAMMA + 1) / (2 * (GAMMA - 1)))


class Engine:
    def __init__(self, pt: PTConfig):
        self.pt = pt
        e, a, i = pt.engine, pt.ambient, pt.intake
        self.e, self.i = e, i
        self.fuel = props(pt.fuel.type)
        self.p0 = a.pressure
        self.T0 = a.temperature + 273.15
        self.Tm = self.T0 + a.intake_temp_rise
        self.Vd = e.displacement
        self.A = np.pi * i.restrictor_diameter ** 2 / 4
        self._ve_rpm = np.asarray(e.ve_rpm, float)
        self._ve = np.asarray(e.ve, float)
        if self._ve_rpm.size == 0:
            raise ValueError("engine.ve_rpm / engine.ve table is empty")

    # ------------------------------------------------------------ air path
    def ve(self, rpm):
        """Volumetric efficiency incl. the runner/primary length what-if."""
        rpm = np.asarray(rpm, float)
        v = np.interp(rpm, self._ve_rpm, self._ve)
        e = self.e
        n_pk = self._ve_rpm[np.argmax(self._ve)]
        g = lambda n_t: np.exp(-0.5 * ((rpm - n_t) / 1800.0) ** 2)
        # intake runner: quarter-wave tuning, tuned rpm ~ 1 / length
        L, Lref = e.intake_runner_length, e.intake_runner_ref
        if L and Lref and abs(L - Lref) > 1e-6:
            v = v + e.tuning_amplitude * (g(n_pk * Lref / L) - g(n_pk))
        # exhaust primary: binder formula P[in] = 850 ED / rpm - 3  ->  tuned rpm = 850 ED / (P + 3)
        P, Pref = e.exhaust_primary_length / 0.0254, e.exhaust_primary_ref / 0.0254
        if P and Pref and abs(P - Pref) > 1e-4:
            v = v + e.tuning_amplitude * (g(n_pk * (Pref + 3) / (P + 3)) - g(n_pk))
        return np.maximum(v, 0.05)

    def choked_flow(self):
        return self.i.restrictor_cd * self.A * self.p0 * np.sqrt(GAMMA / (R_AIR * self.T0)) * _PSI_STAR

    def restrictor_flow(self, p_m):
        """Mass flow through the venturi for plenum pressure p_m (diffuser recovery included)."""
        p_m = np.asarray(p_m, float)
        rec = self.i.diffuser_recovery
        p_t = self.p0 - (self.p0 - p_m) / max(1 - rec, 1e-3)
        r = np.clip(p_t / self.p0, _CRIT, 1.0)
        psi = np.sqrt(np.maximum(2 * GAMMA / (GAMMA - 1) * (r ** (2 / GAMMA) - r ** ((GAMMA + 1) / GAMMA)), 0.0))
        return self.i.restrictor_cd * self.A * self.p0 / np.sqrt(R_AIR * self.T0) * psi

    def choke_map(self):
        """Plenum pressure below which the throat is sonic."""
        return self.p0 - (1 - self.i.diffuser_recovery) * (self.p0 - _CRIT * self.p0)

    def demand_coef(self, rpm):
        """Engine air demand per unit plenum pressure [kg/s/Pa]."""
        rpm = np.asarray(rpm, float)
        return self.ve(rpm) / (R_AIR * self.Tm) * self.Vd * rpm / 120.0

    def wot_air(self, rpm, n_iter=50):
        """(mass flow, plenum pressure, choked flag) at wide-open throttle."""
        rpm = np.atleast_1d(np.asarray(rpm, float))
        k = self.demand_coef(rpm)
        K = self.i.loss_coef
        lo, hi = np.full(rpm.shape, 0.2 * self.p0), np.full(rpm.shape, self.p0)
        for _ in range(n_iter):
            mid = 0.5 * (lo + hi)
            m = k * mid
            more = m < self.restrictor_flow(np.minimum(mid + K * m ** 2, self.p0))   # restrictor exit = plenum + losses
            lo = np.where(more, mid, lo)
            hi = np.where(more, hi, mid)
        pm = 0.5 * (lo + hi)
        p_exit = pm + K * (k * pm) ** 2
        return k * pm, pm, p_exit <= self.choke_map() + 1.0

    # ------------------------------------------------------------ torque parts
    def piston_speed(self, rpm):
        return 2 * self.e.stroke * np.asarray(rpm, float) / 60

    def fmep(self, rpm):
        sp = self.piston_speed(rpm)
        return 1e3 * (self.e.fmep_a + self.e.fmep_b * sp + self.e.fmep_c * sp ** 2)

    def friction_torque(self, rpm):
        return self.fmep(rpm) * self.Vd / (4 * np.pi)

    def pumping_torque(self, p_m):
        return (self.e.exhaust_back_pressure - np.asarray(p_m, float)) * self.Vd / (4 * np.pi)

    def _eta_lambda(self, lam):
        lam = np.asarray(lam, float)
        return np.clip(1.0 - 0.9 * (lam - 0.90) ** 2, 0.7, 1.0)

    def indicated_torque(self, rpm, m_air, lam):
        lam = np.asarray(lam, float)
        q = self.e.eta_indicated * self._eta_lambda(lam) * m_air / self.fuel.afr * self.fuel.lhv * np.minimum(1.0, 1.0 / lam)
        return q / np.maximum(omega(rpm), 1.0)

    def derate(self, coolant_temp=None):
        if coolant_temp is None:
            return 1.0
        e = self.e
        return np.clip(1.0 - e.derate_per_degC * np.maximum(np.asarray(coolant_temp, float) - e.derate_start_temp, 0.0), 0.3, 1.0)

    # ------------------------------------------------------------ full load
    def wot(self, rpm, coolant_temp=None):
        """Everything at wide-open throttle (dict of arrays)."""
        rpm = np.atleast_1d(np.asarray(rpm, float))
        m, pm, ch = self.wot_air(rpm)
        lam = np.full(rpm.shape, self.e.lambda_wot)
        Ti = self.indicated_torque(rpm, m, lam)
        Tf = self.friction_torque(rpm)
        Tp = self.pumping_torque(pm)
        Tb = (Ti - Tf - Tp) * self.derate(coolant_temp) * self.e.torque_scale
        Tb = np.where(rpm > self.e.rev_limit * 1.002, 0.0, Tb)
        P = Tb * omega(rpm)
        mf = self.e.fuel_scale * m / (self.fuel.afr * lam)
        return {"rpm": rpm, "torque": Tb, "power": P, "m_air": m, "map": pm, "choked": ch,
                "m_fuel": mf, "bsfc_g_kwh": mf * 3.6e9 / np.maximum(P, 1.0),
                "T_ind": Ti, "T_fric": Tf, "T_pump": Tp, "ve": self.ve(rpm),
                "demand_at_p0": self.demand_coef(rpm) * self.p0, "choked_flow": np.full(rpm.shape, self.choked_flow()),
                "lambda": lam}

    def torque_wot(self, rpm, coolant_temp=None):
        return self.wot(rpm, coolant_temp)["torque"]

    def motoring_torque(self, rpm):
        """Closed-throttle engine braking torque (positive number = drag)."""
        rpm = np.asarray(rpm, float)
        p_closed = 0.25 * self.p0
        return self.friction_torque(rpm) + self.pumping_torque(p_closed)

    # ------------------------------------------------------------ part load
    def operating_point(self, rpm, torque, coolant_temp=None):
        """Air flow, plenum pressure, throttle (air-flow fraction), fuel flow and BSFC that
        produce brake ``torque`` at ``rpm``. Torque above WOT is clipped to WOT."""
        rpm = np.atleast_1d(np.asarray(rpm, float))
        torque = np.broadcast_to(np.asarray(torque, float), rpm.shape).astype(float)
        w = self.wot(rpm, coolant_temp)
        der = self.derate(coolant_temp)
        Tq = np.minimum(torque, w["torque"])
        k = self.demand_coef(rpm)
        c4 = self.Vd / (4 * np.pi)
        lam = np.full(rpm.shape, self.e.lambda_part)
        m = w["m_air"].copy()
        for _ in range(3):
            frac = m / np.maximum(w["m_air"], 1e-9)
            lam = self.e.lambda_part + (self.e.lambda_wot - self.e.lambda_part) * np.clip((frac - 0.85) / 0.15, 0, 1)
            a = self.e.eta_indicated * self._eta_lambda(lam) / self.fuel.afr * self.fuel.lhv * np.minimum(1, 1 / lam) \
                / np.maximum(omega(rpm), 1.0)
            # Tb/der = a*m - Tf - (p_exh - m/k) * c4    -> linear in m
            m = (Tq / np.maximum(der * self.e.torque_scale, 1e-3) + self.friction_torque(rpm) + self.e.exhaust_back_pressure * c4) / (a + c4 / k)
            m = np.clip(m, 0.0, w["m_air"])
        pm = m / k
        overrun = torque <= 0.0
        cut = overrun & bool(self.e.overrun_fuel_cut) & (rpm > self.e.idle_rpm + 1000)
        mf = np.where(overrun, np.where(cut, 0.0, self.idle_fuel()), self.e.fuel_scale * m / (self.fuel.afr * lam))
        P = np.maximum(Tq, 0) * omega(rpm)
        return {"rpm": rpm, "torque": Tq, "power": P, "m_air": m, "map": pm,
                "throttle": np.clip(m / np.maximum(w["m_air"], 1e-9), 0, 1), "m_fuel": mf,
                "bsfc_g_kwh": np.where(P > 500, mf * 3.6e9 / np.maximum(P, 1.0), np.nan),
                "fuel_power": mf * self.fuel.lhv, "lambda": lam}

    def idle_fuel(self):
        return 0.12e-3                       # kg/s, ~0.6 L/h at idle

    def torque_map(self, rpm_grid, load_grid):
        """Brake torque and BSFC on an rpm x load (air-flow fraction) grid."""
        R, Lg = np.meshgrid(np.asarray(rpm_grid, float), np.asarray(load_grid, float), indexing="ij")
        w = self.wot(R.ravel())
        m = Lg.ravel() * w["m_air"]
        k = self.demand_coef(R.ravel())
        lam = self.e.lambda_part + (self.e.lambda_wot - self.e.lambda_part) * np.clip((Lg.ravel() - 0.85) / 0.15, 0, 1)
        Tb = self.indicated_torque(R.ravel(), m, lam) - self.friction_torque(R.ravel()) - self.pumping_torque(m / k)
        P = Tb * omega(R.ravel())
        mf = self.e.fuel_scale * m / (self.fuel.afr * lam)
        bsfc = np.where(P > 500, mf * 3.6e9 / np.maximum(P, 1), np.nan)
        return R, Lg, Tb.reshape(R.shape), bsfc.reshape(R.shape), (m / k).reshape(R.shape)

    def summary(self):
        r = np.arange(4000, self.e.rev_limit + 1, 50.0)
        w = self.wot(r)
        i, j = np.argmax(w["torque"]), np.argmax(w["power"])
        ch = np.flatnonzero(w["choked"])
        return {"peak_torque_Nm": float(w["torque"][i]), "at_rpm": float(r[i]),
                "peak_power_kW": float(w["power"][j] / 1e3), "peak_power_hp": float(w["power"][j] / 745.7),
                "at_rpm_power": float(r[j]), "restrictor_choked_flow_g_s": self.choked_flow() * 1e3,
                "choke_onset_rpm": float(r[ch[0]]) if ch.size else None,
                "min_bsfc_g_kWh": float(np.nanmin(w["bsfc_g_kwh"]))}


# --------------------------------------------------------------------------- cooling
CP_W, CP_AIR, RHO_AIR = 3600.0, 1007.0, 1.15     # 50/50 glycol-water [J/kg K], air


def eps_crossflow(ntu, cr):
    ntu = np.maximum(ntu, 1e-9)
    cr = np.clip(cr, 1e-6, 1.0)
    return 1 - np.exp((ntu ** 0.22 / cr) * (np.exp(-cr * ntu ** 0.78) - 1))


@dc.dataclass
class ThermalState:
    Tc: float
    To: float
    fan: bool = False


class Thermal:
    def __init__(self, pt: PTConfig):
        self.c = pt.cooling
        self.T_amb = pt.ambient.temperature
        self.A = self.c.core_width * self.c.core_height

    def init_state(self, Tc=None):
        T = self.c.start_temp if Tc is None else Tc
        return ThermalState(T, T, False)

    def thermostat(self, Tc):
        c = self.c
        return np.clip((Tc - c.thermostat_open) / max(c.thermostat_full - c.thermostat_open, 1e-3), 0.02, 1.0)

    def radiator(self, Tc, v, rpm, fan):
        c = self.c
        V_air = c.duct_ratio * np.maximum(v, 0.0) * self.A + (c.fan_flow if fan else 0.0)
        V_air = max(V_air, 0.01)
        UA = c.ua_ref * (V_air / c.air_flow_ref) ** 0.6
        C_air = RHO_AIR * V_air * CP_AIR
        if c.pump_type == "electric":
            m_c = c.pump_flow / 60.0 * 1.0 * self.thermostat(Tc)          # L/min -> kg/s (~1 kg/L)
        else:
            m_c = c.pump_flow_per_krpm * max(rpm, 1000.0) / 1000.0 * self.thermostat(Tc)
        C_c = m_c * CP_W
        Cmin, Cmax = min(C_air, C_c), max(C_air, C_c)
        eps = eps_crossflow(UA / Cmin, Cmin / Cmax)
        return float(eps * Cmin * (Tc - self.T_amb)), V_air

    def step(self, s: ThermalState, dt, v, rpm, fuel_power):
        c = self.c
        if s.Tc >= c.fan_on_temp:
            s.fan = True
        elif s.Tc <= c.fan_off_temp:
            s.fan = False
        q_rad, _ = self.radiator(s.Tc, v, rpm, s.fan)
        q_conv = 4.0 * (s.Tc - self.T_amb)            # block/hoses to air
        q_oc = c.oil_to_coolant_ua * (s.To - s.Tc)
        dTc = (c.heat_to_coolant * fuel_power - q_rad - q_conv + q_oc) / c.thermal_mass_coolant
        dTo = (c.heat_to_oil * fuel_power - q_oc) / c.thermal_mass_oil
        s.Tc += dTc * dt
        s.To += dTo * dt
        return s, {"Q_in_coolant": c.heat_to_coolant * fuel_power, "Q_rad": q_rad, "Q_oil": q_oc}

# --------------------------------------------------------------------------- electrics
class Electrical:
    def __init__(self, pt: PTConfig):
        self.c = pt.electrical
        self.fan_power = pt.cooling.fan_power

    def base_load(self):
        return float(sum(v for v in self.c.loads.values() if v))

    def load(self, fan_on=False):
        return self.base_load() + self.fan_power * np.asarray(fan_on, float)

    def available(self, rpm):
        c = self.c
        return c.alternator_max_power * np.clip(np.asarray(rpm, float) / c.alternator_full_rpm, 0.0, 1.0) ** 1.5

    def output(self, rpm, demand):
        return np.minimum(self.available(rpm), np.asarray(demand, float))

    def drag_torque(self, rpm, p_out=None):
        """Crank torque taken by the alternator for electrical output p_out (default: base load)."""
        rpm = np.asarray(rpm, float)
        if p_out is None:
            p_out = self.output(rpm, self.base_load())
        w = np.maximum(rpm * 2 * np.pi / 60, 50.0)
        return self.c.alternator_drag + np.asarray(p_out, float) / (self.c.alternator_efficiency * w)

    def battery_step(self, soc, p_alt, p_load, dt):
        cap_j = self.c.battery_capacity_ah * 3600 * self.c.battery_voltage
        return float(np.clip(soc + (p_alt - p_load) * dt / cap_j, 0.0, 1.0))

# --------------------------------------------------------------------------- the IC car
class EnginePowertrain:
    """Drop-in for fsae_vd.vehicle.Powertrain: force(v) = best-gear WOT tractive force,
    reduced by the gear's rotating-inertia share so ax from the GGV is the vehicle's."""

    def __init__(self, dt: Drivetrain, m: float):
        self.dt, self.m = dt, m
        self.unlimited = False
        self.coolant_temp = None

    def force(self, v):
        v = np.atleast_1d(np.asarray(v, float))
        if self.unlimited:
            return np.full(v.shape, np.inf)
        F = self.dt.forces_all_gears(v, self.coolant_temp)
        meq = np.array([self.dt.equivalent_mass(k, self.m) for k in range(1, self.dt.n_gears + 1)])
        Fe = F * (self.m / meq)[:, None]
        return Fe.max(0)

class ICVehicle(Vehicle):
    def __init__(self, cfg: CarConfig, pt: PTConfig):
        cfg = cfg.copy()
        # resistances: the vehicle section wins; ASSUMED powertrain defaults only fill gaps
        self.assumed = []
        if cfg.aero.CdA is None and cfg.aero.model == "constant":
            cfg.aero.CdA = pt.resistance.assumed_CdA
            self.assumed.append(f"aero.CdA = {pt.resistance.assumed_CdA} m^2 (ASSUMED, fit a coast-down)")
        if cfg.tires.rolling_resistance is None:
            cfg.tires.rolling_resistance = pt.resistance.assumed_Crr
            self.assumed.append(f"tires.rolling_resistance = {pt.resistance.assumed_Crr} (ASSUMED)")
        cfg.powertrain.kind = "ICE"
        super().__init__(cfg)
        self.ptc = pt
        self.engine = Engine(pt)
        self.elec = Electrical(pt)
        self.dt = Drivetrain(pt, cfg.tires.tire_radius, self.engine, self.elec)
        self.cfg.powertrain.final_drive_ratio = self.dt.fd * self.dt.primary
        self.pt = EnginePowertrain(self.dt, self.m)
        self.diff = self.dt.diff
        self.warnings += self.assumed

    def coast_torque(self, v):
        """Engine braking at the axle in the gear held while braking."""
        g = self.braking_gear(v)
        return self.engine.motoring_torque(self.dt.rpm(v, g)) * self.dt.overall[g - 1]

    def braking_gear(self, v):
        """Gear held while braking: the lowest gear below 92 % of the rev limit (drivers
        downshift to the exit gear)."""
        v = np.maximum(np.atleast_1d(np.asarray(v, float)), 0.5)
        rpm = np.stack([self.dt.rpm(v, k) for k in range(1, self.dt.n_gears + 1)])
        ok = rpm <= 0.92 * self.ptc.engine.rev_limit
        g = np.where(ok.any(0), np.argmax(ok, axis=0) + 1, self.dt.n_gears)
        return g.reshape(np.shape(v))
