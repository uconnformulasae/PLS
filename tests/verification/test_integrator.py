"""The RK4 integrator reaches fourth order on a problem with an exact solution."""
import math

import numpy as np

from lapsim.core.integrator import rk4_step


def _solve(dt, t_end=2.0):
    y = [1.0, 0.0]                              # harmonic oscillator x'' = -x, x(0) = 1
    for _ in range(int(round(t_end / dt))):
        y, _ = rk4_step(lambda s: ([s[1], -s[0]], None), y, dt)
    return y[0]


def test_fourth_order_convergence():
    exact = math.cos(2.0)
    e = [abs(_solve(dt) - exact) for dt in (0.1, 0.05, 0.025)]
    orders = np.log2(np.array(e[:-1]) / np.array(e[1:]))
    assert np.all(orders > 3.8)


def test_info_is_from_the_start_of_the_step():
    _, info = rk4_step(lambda s: ([1.0], s[0]), [5.0], 0.1)
    assert info == 5.0
