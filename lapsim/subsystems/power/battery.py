"""Accumulator: pack voltage, resistance, current limits, state of charge and cell temperature.

The pack is a pack-level equivalent circuit (mean cell x series count; cell-to-cell spread is
the BMS tool's job):

    V = N_s OCV(SOC, T) - v1 - v2 - I (N_s r0 f(T) + R_ic)      f(T): Arrhenius resistance factor
    dv1/dt = (N_s r1 f(T) I - v1) / tau1,  same for v2          (two RC branches)
    dSOC/dt = -eta I / Q                                        (eta = coulombic efficiency on charge)
    C dT/dt = Q_cell - (T - T_amb) / R_th                       (one lumped node per physical cell)
"""
from __future__ import annotations

import dataclasses as dc

import numpy as np

R_GAS = 8.314462618  # J/(mol K)


class OCV:
    """Open-circuit voltage OCV(SOC, T): monotone cubic (PCHIP) through the table, sampled on a fine
    grid and evaluated with ``np.interp``; outside [0, 1] the end slopes are extended linearly."""

    def __init__(self, soc, v, docv_dt=0.0, t_ref=25.0, n=2001):
        from scipy.interpolate import PchipInterpolator
        soc = np.asarray(soc, float)
        v = np.asarray(v, float)
        if np.any(np.diff(soc) <= 0) or np.any(np.diff(v) <= 0):
            raise ValueError("OCV table must be strictly increasing in both SOC and voltage")
        p = PchipInterpolator(soc, v)
        self.grid = np.linspace(soc[0], soc[-1], n)
        self.v_grid = p(self.grid)
        dv = p.derivative()(self.grid)
        self.s0, self.s1 = soc[0], soc[-1]
        self.k0, self.k1 = float(dv[0]), float(dv[-1])
        self.docv_dt = docv_dt
        self.t_ref = t_ref

    def __call__(self, soc, temp=None):
        soc = np.asarray(soc, float)
        v = np.interp(soc, self.grid, self.v_grid)
        v = np.where(soc < self.s0, self.v_grid[0] + self.k0 * (soc - self.s0), v)
        v = np.where(soc > self.s1, self.v_grid[-1] + self.k1 * (soc - self.s1), v)
        if temp is not None and self.docv_dt:
            v = v + self.docv_dt * (np.asarray(temp, float) - self.t_ref)
        return v


def arrhenius(temp_c, ea, t_ref_c=25.0):
    """Resistance multiplier exp(Ea/R (1/T - 1/Tref)); > 1 when colder than Tref."""
    t = np.asarray(temp_c, float) + 273.15
    return np.exp(ea / R_GAS * (1.0 / t - 1.0 / (t_ref_c + 273.15)))


@dc.dataclass
class PackState:
    soc: float
    temp: float              # degC, cells
    v1: float = 0.0          # V, pack-level fast RC voltage
    v2: float = 0.0          # V, pack-level slow RC voltage
    motor_temp: float = 40.0

    def copy(self):
        return dc.replace(self)


class Pack:
    """The `pack:` section of the car file as a pack-level equivalent circuit."""

    def __init__(self, bcfg, evc):
        self.bcfg, self.a = bcfg, evc.accumulator
        c = bcfg.cell.resolved()
        self.ns, self.npar = int(bcfg.pack.series), int(bcfg.pack.parallel)
        self.q = c["capacity_ah"] * 3600.0 * self.npar          # A s per parallel group
        self.ocv = OCV(c["ocv_soc"], c["ocv_v"], c.get("docv_dt", 0.0) or 0.0, float(c["t_ref"]))
        self.r0, self.r1, self.r2 = c["r0"] / self.npar, c["r1"] / self.npar, c["r2"] / self.npar
        self.tau1, self.tau2 = float(c["tau1"]), float(c["tau2"])
        self.ea, self.t_ref = float(c["ea_r"]), float(c["t_ref"])
        self.r_ic, self.eta_c = float(bcfg.pack.interconnect_resistance), float(c["eta_charge"])
        self.v_min, self.v_max = float(c["v_min"]), float(c["v_max"])
        self.lim, self.th = bcfg.limits, bcfg.thermal
        self.c_th = self.th.cell_mass * self.th.cell_cp                      # J/K per physical cell
        self.r_th = self.th.r_core_surface + self.th.r_surface_ambient       # K/W per physical cell
        br = self.a.sag_branches.replace(" ", "").lower().split("+")
        self._sag = self.r0 * ("r0" in br) + self.r1 * ("r1" in br) + self.r2 * ("r2" in br)

    def initial_state(self, motor_temp=40.0):
        return PackState(self.a.start_soc, self.a.start_temp, motor_temp=motor_temp)

    @property
    def nominal_energy(self):
        """J from 100 % to 0 % SOC at open circuit."""
        s = np.linspace(0, 1, 501)
        return float(np.trapezoid(self.ocv(s), s) * self.q * self.ns)

    def voc(self, soc, temp=None):
        return self.ns * self.ocv(soc, temp)

    def r_sag(self, temp):
        """Pack resistance [ohm] used for the instantaneous power limit."""
        return self.ns * self._sag * float(arrhenius(temp, self.ea, self.t_ref)) + self.r_ic

    def temp_derate(self, temp):
        L = self.lim
        return float(np.clip((L.cell_t_max - temp) / max(L.cell_t_max - L.cell_t_derate, 1e-6), 0.0, 1.0))

    def discharge_current_limit(self, temp):
        return self.npar * self.lim.i_dis_peak * self.temp_derate(temp)

    def charge_current_limit(self, soc, temp, max_soc=1.0):
        if soc >= max_soc or temp <= self.lim.cell_t_min_charge:
            return 0.0
        return self.npar * self.lim.i_chg_peak * self.temp_derate(temp)

    def current(self, p, e, r):
        """Pack current for terminal power p with source voltage e behind resistance r (vectorised).
        Returns (I, feasible)."""
        p = np.asarray(p, float)
        disc = e * e - 4 * r * p
        ok = disc >= 0
        return (e - np.sqrt(np.maximum(disc, 0.0))) / (2 * r), ok

    def simulate(self, t, p_acc, state: PackState):
        """Integrate the pack along a power trace. Returns (channels dict, end state)."""
        n = len(t)
        I = np.zeros(n); V = np.zeros(n); soc = np.zeros(n); temp = np.zeros(n); heat = np.zeros(n)
        short = np.zeros(n, bool)
        s, T, v1, v2 = state.soc, state.temp, state.v1, state.v2
        dtv = np.diff(t, append=t[-1])
        ocv, ns, q = self.ocv, self.ns, self.q
        for k in range(n):
            f = float(arrhenius(T, self.ea, self.t_ref))
            r0 = ns * self.r0 * f + self.r_ic
            voc = ns * float(ocv(s, T))
            e = voc - v1 - v2
            disc = e * e - 4 * r0 * p_acc[k]
            if disc < 0:
                short[k] = True
                i = e / (2 * r0)
            else:
                i = (e - np.sqrt(disc)) / (2 * r0)
            v = e - i * r0
            I[k], V[k], soc[k], temp[k] = i, v, s, T
            heat[k] = i * (voc - v - i * self.r_ic) / (ns * self.npar)       # W per physical cell
            d = dtv[k]
            if d > 0:
                s -= (self.eta_c if i < 0 else 1.0) * i * d / q
                a1 = np.exp(-d / self.tau1)
                v1 = a1 * v1 + ns * self.r1 * f * (1 - a1) * i
                if self.tau2 > 0:
                    a2 = np.exp(-d / self.tau2)
                    v2 = a2 * v2 + ns * self.r2 * f * (1 - a2) * i
                T += d * (heat[k] - (T - self.th.ambient) / self.r_th) / self.c_th
        end = PackState(s, T, v1, v2, state.motor_temp)
        return {"I_pack": I, "V_pack": V, "soc": soc, "cell_temp_C": temp, "cell_heat_W": heat,
                "V_cell": (V + I * self.r_ic) / ns, "I_cell": I / self.npar, "power_unmet": short}, end

    def rest(self, state: PackState, seconds):
        """Tractive system idle (driver change): cells cool towards ambient, RC voltages relax."""
        k = np.exp(-seconds / (self.r_th * self.c_th))
        return PackState(state.soc, self.th.ambient + (state.temp - self.th.ambient) * k,
                         state.v1 * np.exp(-seconds / self.tau1),
                         state.v2 * np.exp(-seconds / max(self.tau2, 1e-9)), state.motor_temp)
