"""Each subsystem against hand calculations (tests/reference_values.py)."""
import numpy as np
import pytest

from lapsim.config.schema import Brakes
from lapsim.core import events
from lapsim.subsystems import brakes, driveline
from lapsim.subsystems.power import battery
from lapsim.subsystems.tires import Tire
from tests import reference_values as R


def test_tire_peak_friction(ic):
    assert Tire(ic.vehicle.tires).mu(800.0, -2.0) == pytest.approx(R.TIRE_MU_800N_M2DEG, rel=1e-12)


def test_tire_lateral_curve_peaks_at_mu_fz(ic):
    t = Tire(ic.vehicle.tires)
    fy = t.fy(np.radians(np.linspace(0, 30, 3001)), 800.0)
    assert fy.max() == pytest.approx(t.mu(800.0) * 800.0, rel=1e-6)


def test_wheel_rate(ic):
    from lapsim.subsystems.suspension import mr_damper_per_wheel
    ax = ic.copy().vehicle.front
    ax.spring_rate, ax.motion_ratio, ax.motion_ratio_convention = 350 * R.LBF_PER_IN, 1.18, "wheel/damper"
    assert ax.spring_rate * mr_damper_per_wheel(ax) ** 2 == pytest.approx(R.WHEEL_RATE_350_MR118)


def test_brake_gain():
    b = Brakes()
    b.piston_area_front = b.piston_area_rear = 1e-3
    b.pad_mu, b.rotor_radius_front, b.rotor_radius_rear = 0.45, 0.08, 0.08
    assert b.torque_gains()[0] == pytest.approx(R.BRAKE_GAIN)
    assert brakes._gain(1e-3, 0.45, 0.08) == pytest.approx(R.BRAKE_GAIN)


@pytest.mark.parametrize("event", list(R.MAX_POINTS))
def test_scoring_at_tmin_is_max(event):
    assert events.points(event, 50.0, 50.0) == pytest.approx(R.MAX_POINTS[event])


def test_scoring_at_tmax_is_finishing_points():
    A, B, k, p = events.SCORING["autocross"]
    assert events.points("autocross", 50.0 * k, 50.0) == B


def test_arrhenius():
    assert battery.arrhenius(0.0, 28e3, 25.0) == pytest.approx(R.ARRHENIUS_0C)


def test_ocv_is_monotonic_and_pack_voltage(ev):
    pack = battery.Pack(ev.pack, ev.power)
    s = np.linspace(0, 1, 101)
    assert np.all(np.diff(pack.ocv(s)) > 0)
    assert pack.voc(1.0) == pytest.approx(ev.pack.pack.series * 4.18, rel=1e-3)


def test_pack_rest_cools_towards_ambient(ev):
    pack = battery.Pack(ev.pack, ev.power)
    st = pack.rest(battery.PackState(0.5, 45.0), 1e6)
    assert st.temp == pytest.approx(ev.pack.thermal.ambient, abs=1e-6)


def test_omega():
    assert float(driveline.omega(60.0)) == pytest.approx(2 * np.pi)


def test_aero_drag_grows_with_speed_squared(ic_vehicle):
    _, _, d1 = ic_vehicle.aero(10.0)
    _, _, d2 = ic_vehicle.aero(20.0)
    assert d2 / d1 == pytest.approx(4.0, rel=0.05)       # ride height changes the coefficients slightly


def test_static_wheel_loads_sum_to_weight(ic_vehicle):
    st = ic_vehicle.state(np.array([0.5]), np.array([0.0]), np.array([0.0]))
    total = sum(float(st["Fz"][k][0]) for k in st["Fz"])
    assert total == pytest.approx(ic_vehicle.m * 9.80665 + float(st["downforce"][0]), rel=1e-9)
