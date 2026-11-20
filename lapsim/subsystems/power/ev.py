"""EV power module: motor, inverter, reduction, regen and the EV car.

Power flow (drive positive, regen negative), all at one operating point:

    wheels  <- reduction (efficiency) <-  motor shaft  T, omega
    P_dc  = T*omega + copper loss + iron loss + inverter loss      (dc link, traction)
    P_acc = P_dc + LV auxiliaries / DC-DC efficiency               (accumulator terminals: what the
                                                                    energy meter and the 80 kW rule see)
    pack:   V = N_s*OCV(SOC) - I*R(T),   P_acc = V*I

What limits the motor torque at a speed (``EVTables.code``):
    torque     motor peak torque / inverter phase current / driver torque limit
    power      accumulator power limit (rules 80 kW, or the strategy limit) or motor peak power
    voltage    field weakening: above the corner speed, which moves with the pack voltage
    battery    cell current limit (incl. temperature derating), pack / inverter under-voltage
    top_speed  motor max rpm

``EVVehicle`` is the chassis (``core.vehicle.Vehicle``) with the same differential model as the IC car
and the motor / inverter / accumulator (``power.battery``) in place of the engine and gearbox: the
drive-force limit is the motor torque the accumulator can supply at each speed (``EVTables``), and
under braking the regen torque loads the coast side of the differential.
"""
from __future__ import annotations

import numpy as np
import pandas as pd

from lapsim.config.schema_ev import EVConfig
from lapsim.core.vehicle import Vehicle
from lapsim.subsystems.driveline import Differential
from lapsim.subsystems.power.battery import Pack, PackState

CODES = ("torque", "power", "voltage", "battery", "top_speed")
ALPHA_CU = 0.00393       # 1/K copper resistance temperature coefficient
ASSUMED_CRR = 0.020


# --------------------------------------------------------------------------- motor + inverter
class Motor:
    def __init__(self, evc: EVConfig):
        self.c, self.inv, self.st = evc.motor, evc.inverter, evc.strategy
        self.curve = None
        if self.c.torque_curve_file:
            d = pd.read_csv(self.c.torque_curve_file)
            cols = {k.lower().strip(): k for k in d.columns}
            self.curve = (d[cols.get("rpm", d.columns[0])].to_numpy(float),
                          d[cols.get("torque_nm", d.columns[1])].to_numpy(float))

    def derate(self, temp):
        c = self.c
        if temp is None:
            return 1.0
        return float(np.clip((c.derate_end_temp - temp) / max(c.derate_end_temp - c.derate_start_temp, 1e-6), 0.0, 1.0))

    def torque_cap(self, temp=None):
        """Speed-independent torque ceiling: motor peak (derated), inverter current, driver limit."""
        cap = min(self.c.peak_torque * self.derate(temp), self.c.kt * self.inv.max_phase_current)
        return min(cap, self.st.torque_limit) if self.st.torque_limit else cap

    def torque_limit(self, rpm, vdc, temp=None):
        """(max torque [N m], limiting code index into CODES) at motor speed rpm and dc voltage vdc."""
        c = self.c
        rpm = np.asarray(rpm, float)
        vdc = np.broadcast_to(np.asarray(vdc, float), rpm.shape)
        w = np.maximum(rpm * np.pi / 30, 1e-3)
        cap = self.torque_cap(temp)
        ratio = np.maximum(vdc / c.rated_voltage, 1e-3)
        if self.curve is not None:                       # measured envelope at rated voltage, speed axis scaled
            t_v = np.interp(rpm / ratio, *self.curve, right=0.0) * self.derate(temp)
        else:
            wc = c.corner_rpm * ratio
            t_v = np.where(rpm <= wc, np.inf, c.peak_torque * wc / np.maximum(rpm, 1e-6))
        t_p = c.peak_power / w
        lim = np.minimum(np.minimum(cap, t_v), t_p)
        code = np.where(t_v <= np.minimum(cap, t_p), 2, np.where(t_p < cap, 1, 0))
        over = rpm > c.max_rpm
        return np.where(over, 0.0, lim), np.where(over, 4, code)

    def losses(self, torque, rpm, temp=None):
        """(copper, iron, inverter) losses [W] at a shaft torque (either sign) and speed."""
        c = self.c
        w = np.abs(np.asarray(rpm, float)) * np.pi / 30
        i_ph = np.abs(np.asarray(torque, float)) / c.kt
        r = c.phase_resistance * (1 + ALPHA_CU * ((c.start_temp if temp is None else temp) - 20.0))
        cu = 3 * r * i_ph ** 2
        iron = c.iron_loss_1 * w + c.iron_loss_2 * w ** 2
        inv = self.inv.loss_const + self.inv.loss_per_amp * i_ph
        return cu, iron, inv

    def p_dc(self, torque, rpm, temp=None):
        cu, iron, inv = self.losses(torque, rpm, temp)
        return np.asarray(torque, float) * np.asarray(rpm, float) * np.pi / 30 + cu + iron + inv

    def efficiency_map(self, rpm, torque, temp=None):
        """Motor + inverter efficiency on a grid (driving): shaft power / dc power."""
        R, T = np.meshgrid(rpm, torque)
        pm = T * R * np.pi / 30
        return np.where(pm > 0, pm / np.maximum(self.p_dc(T, R, temp), 1e-9), np.nan)


# --------------------------------------------------------------------------- whole powertrain
class EVPowertrain:
    def __init__(self, evc: EVConfig, bcfg, tire_radius: float, mass: float):
        self.evc = evc
        self.motor = Motor(evc)
        self.pack = Pack(bcfg, evc)
        self.G = evc.drivetrain.overall
        self.eta = evc.drivetrain.efficiency
        self.r = tire_radius * evc.wheels.rolling_radius_factor
        self.m = mass
        w = evc.wheels
        j = 2 * w.front_corner_inertia + 2 * w.rear_corner_inertia + evc.drivetrain.inertia \
            + evc.motor.rotor_inertia * self.G ** 2
        self.meq = mass + j / self.r ** 2
        self.p_aux = evc.lv.aux_power / max(evc.lv.dcdc_efficiency, 1e-6)

    def rpm(self, v):
        return np.asarray(v, float) / self.r * self.G * 30 / np.pi

    def v_at(self, rpm):
        return rpm * np.pi / 30 / self.G * self.r

    def power_limit(self, endurance=False):
        """Accumulator power limit [W]: the rules value unless the strategy sets a lower one."""
        st = self.evc.strategy
        lim = self.pack.lim.max_power
        for x in ((st.endurance_power_limit if endurance else None), st.power_limit):
            if x:
                return min(lim, x)
        return lim

    def initial_state(self):
        return self.pack.initial_state(self.evc.motor.start_temp)

    def chain_tension(self, axle_torque):
        d = self.evc.drivetrain
        if d.type != "chain" or not d.rear_sprocket:
            return np.full(np.shape(axle_torque), np.nan)
        radius = d.chain_pitch / (2 * np.sin(np.pi / d.rear_sprocket))
        return np.abs(axle_torque) / radius


class EVTables:
    """Maximum drive and regen force on a fine speed grid for one accumulator / motor state."""

    def __init__(self, ep: EVPowertrain, resist, state: PackState | None = None, p_limit=None, dv=0.05, v_max=45.0):
        self.ep = ep
        st = self.state = state or ep.initial_state()
        self.p_limit = p_limit if p_limit is not None else ep.power_limit()
        self.dv = dv
        v = self.v = np.arange(0.0, v_max + dv, dv)
        rpm = self.rpm = ep.rpm(v)
        self.T, self.code, self.P_acc = self._max_drive(rpm, st)
        self.F = self.T * ep.G * ep.eta / ep.r
        self.R = resist(np.maximum(v, 0.0))
        self.meq = ep.meq
        self.A = (self.F - self.R) / self.meq
        self.T_regen, self.P_regen = self._max_regen(rpm, st)
        self.F_regen = self.T_regen * ep.G / (ep.eta * ep.r)
        neg = np.flatnonzero((self.A <= 0) & (v > 5))
        v_top = float(v[neg[0]]) if neg.size else float(v[-1])
        if ep.evc.strategy.max_speed:
            v_top = min(v_top, ep.evc.strategy.max_speed)
        self.v_top = min(v_top, ep.v_at(ep.evc.motor.max_rpm))

    def idx(self, v):
        return min(int(v / self.dv + 0.5), self.v.size - 1)

    # -- drive: largest torque inside every limit, by bisection (all speeds at once)
    def _drive_ok(self, T, rpm, st, e, r, i_max, v_floor):
        ep, m = self.ep, self.ep.motor
        p = m.p_dc(T, rpm, st.motor_temp) + ep.p_aux
        i, real = ep.pack.current(p, e, r)
        vdc = e - i * r
        t_lim, mcode = m.torque_limit(rpm, vdc, st.motor_temp)
        c_pow = p > self.p_limit
        c_bat = ~real | (i > i_max) | (vdc < v_floor)
        c_mot = T > t_lim
        code = np.where(c_pow, 1, np.where(c_bat, 3, mcode))
        return ~(c_pow | c_bat | c_mot), code, p

    def _max_drive(self, rpm, st):
        ep, pk = self.ep, self.ep.pack
        e, r = float(pk.voc(st.soc, st.temp)) - st.v1 - st.v2, pk.r_sag(st.temp)
        i_max = min(pk.discharge_current_limit(st.temp), ep.evc.inverter.max_dc_current)
        v_floor = max(pk.ns * pk.v_min, ep.evc.inverter.min_dc_voltage)
        args = (rpm, st, e, r, i_max, v_floor)
        lo, hi = np.zeros_like(rpm), np.full_like(rpm, ep.motor.torque_cap(st.motor_temp) * 1.0001)
        for _ in range(36):
            mid = 0.5 * (lo + hi)
            ok, _, _ = self._drive_ok(mid, *args)
            lo, hi = np.where(ok, mid, lo), np.where(ok, hi, mid)
        _, code, _ = self._drive_ok(lo + 0.5, *args)          # what stops a little more torque
        _, _, p = self._drive_ok(lo, *args)
        return lo, code, p

    # -- regen: largest braking torque the motor / inverter / cells accept
    def _max_regen(self, rpm, st):
        ep, pk, rg = self.ep, self.ep.pack, self.ep.evc.regen
        i_chg = pk.charge_current_limit(st.soc, st.temp, rg.max_soc)
        if not rg.enabled or i_chg <= 0:
            return np.zeros_like(rpm), np.zeros_like(rpm)
        e, r = float(pk.voc(st.soc, st.temp)) - st.v1 - st.v2, pk.r_sag(st.temp)
        fade = np.clip((self.v - rg.min_speed) / max(rg.fade_speed - rg.min_speed, 1e-6), 0.0, 1.0)
        v_ceil = pk.ns * pk.v_max

        def ok(T):
            p_dc = ep.motor.p_dc(-T, rpm, st.motor_temp)
            i, _ = pk.current(p_dc + ep.p_aux, e, r)
            vdc = e - i * r
            t_lim, _ = ep.motor.torque_limit(rpm, vdc, st.motor_temp)
            return (-p_dc <= rg.max_power) & (-i <= i_chg) & (vdc <= v_ceil) & (T <= t_lim), p_dc
        lo, hi = np.zeros_like(rpm), rg.max_torque * fade
        full = ok(hi)[0]
        lo = np.where(full, hi, lo)
        for _ in range(30):
            mid = 0.5 * (lo + hi)
            good = ok(mid)[0]
            lo, hi = np.where(good, mid, lo), np.where(good, hi, mid)
        p_dc = ok(lo)[1]
        # regen is only worth using where it returns energy
        useful = p_dc < 0
        return np.where(useful, lo, 0.0), np.where(useful, -p_dc, 0.0)


# --------------------------------------------------------------------------- the EV car
class _DriveLimit:
    """The drive-force limit (power/base.py): the force the motor can put down at each speed."""

    def __init__(self, veh):
        self.veh = veh
        self.unlimited = False

    def force(self, v):
        v = np.atleast_1d(np.asarray(v, float))
        if self.unlimited:
            return np.full(v.shape, np.inf)
        T = self.veh.tables0
        return np.interp(v, T.v, T.F) * (self.veh.m / T.meq)


class EVVehicle(Vehicle):
    def __init__(self, cfg, evc: EVConfig, bcfg):
        cfg = cfg.copy()
        self.assumed = []
        if cfg.tires.rolling_resistance is None:
            cfg.tires.rolling_resistance = ASSUMED_CRR
            self.assumed.append(f"tires.rolling_resistance = {ASSUMED_CRR} (ASSUMED, fit a coast-down)")
        cfg.powertrain.kind = "EV"
        Vehicle.__init__(self, cfg)
        self.evc = self.ptc = evc
        self.bcfg = bcfg
        self.ep = self.dt = EVPowertrain(evc, bcfg, cfg.tires.tire_radius, self.m)
        self.cfg.powertrain.final_drive_ratio = self.ep.G
        self.diff = Differential(evc)
        self._tables0 = None
        self.pt = _DriveLimit(self)
        self.warnings += self.assumed

    # nominal tables (start-of-event SOC and temperatures)
    @property
    def tables0(self) -> EVTables:
        if self._tables0 is None:
            self._tables0 = self.tables()
        return self._tables0

    def tables(self, state=None, p_limit=None) -> EVTables:
        return EVTables(self.ep, getattr(self, "resist_drive", self.resist), state, p_limit)

    def coast_torque(self, v):
        """Regen torque at the axle (preload alone without regen)."""
        T = self.tables0
        return np.interp(v, T.v, T.F_regen) * self.ep.r
