"""Properties the quasi-steady solver must have whatever the car."""
import dataclasses as dc

import numpy as np
import pytest

from lapsim.core.track import circle, straight
from lapsim.solvers import qss, qss_ev, qss_ic


@pytest.fixture(scope="module")
def ic_env(ic_vehicle):
    return ic_vehicle, ic_vehicle.traction_ggv(), qss_ic.ICTables(ic_vehicle)


def test_skidpad_left_equals_right(ic_env):
    veh, g, T = ic_env
    left = circle(9.125)
    right = dc.replace(left, kappa=-left.kappa, y=-left.y)
    a = qss_ic.solve(left, veh, g, T, flying=True)
    b = qss_ic.solve(right, veh, g, T, flying=True)
    assert a.time == pytest.approx(b.time, rel=1e-9)


def test_flying_lap_is_periodic(ic_env):
    veh, g, T = ic_env
    r = qss_ic.solve(circle(9.125), veh, g, T, flying=True)
    assert r.v[0] == pytest.approx(r.v[-1], abs=0.05)


def test_standing_start_accelerates_except_while_shifting(ic_env):
    veh, g, T = ic_env
    r = qss_ic.solve(straight(75.0, 0.1), veh, g, T, v0=0.0)
    dv = np.diff(r.v)
    shifting = r.pt["shifting"][:-1] > 0           # no drive torque during an upshift: drag slows the car
    assert r.v[0] < 0.2 and np.all(dv[~shifting] > 0) and shifting.any()


def test_speed_never_exceeds_the_lateral_limit(ic_env):
    veh, g, T = ic_env
    r = qss_ic.solve(circle(9.125), veh, g, T, flying=True)
    assert np.all(r.v ** 2 / 9.125 <= g.ay_lim(r.v) * 1.001)


def test_skidpad_time_matches_steady_circle(ic_env):
    veh, g, _ = ic_env
    t, v = qss.skidpad_time(veh, g, 9.125)
    assert t == pytest.approx(2 * np.pi * 9.125 / v)
    assert float(g.ay_lim(v)) == pytest.approx(v ** 2 / 9.125, rel=1e-6)


def test_ev_charge_balance(ev):
    """Charge drawn from the pack over the first endurance lap equals the state-of-charge drop."""
    from lapsim.subsystems.power.battery import Pack
    res = ev.run(("endurance",))
    lr = res.laps["endurance_all"][0]
    pack = Pack(ev.pack, ev.power)
    I, dt = lr.pt["I_pack"], np.diff(lr.t, append=lr.t[-1])
    charge = float(np.sum(np.where(I < 0, pack.eta_c, 1.0) * I * dt))      # A s per parallel group
    assert lr.pt["soc"][0] - lr.end_state.soc == pytest.approx(charge / pack.q, rel=1e-9)


@pytest.mark.slow
def test_transient_agrees_with_qss_on_the_skidpad(ic):
    from lapsim.solvers.transient import Chassis
    res = ic.run(("skidpad",))
    lr = res.laps["skidpad"]
    r = Chassis(res.vehicle, driver_level=0.98).run(lr.track, lr, name="skidpad")
    assert r.summary["completed"]
    assert r.summary["time_s"] == pytest.approx(lr.time, rel=0.05)
