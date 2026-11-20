"""Time integration for the time-domain solvers."""
from __future__ import annotations


def rk4_step(f, y, dt):
    """One classical fourth-order Runge-Kutta step of dy/dt = f(y) for a state held as a list.

    ``f(y)`` returns ``(dy, info)``; ``info`` from the first stage (the start of the step) is returned
    with the new state: ``(y_next, info)``. Inputs that ``f`` closes over are held over the step.
    """
    k1, info = f(y)
    k2, _ = f([yi + 0.5 * dt * ki for yi, ki in zip(y, k1)])
    k3, _ = f([yi + 0.5 * dt * ki for yi, ki in zip(y, k2)])
    k4, _ = f([yi + dt * ki for yi, ki in zip(y, k3)])
    return [yi + dt / 6 * (a1 + 2 * a2 + 2 * a3 + a4) for yi, a1, a2, a3, a4 in zip(y, k1, k2, k3, k4)], info
