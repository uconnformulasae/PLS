"""Tires: peak friction with load sensitivity and camber, simplified Magic Formula lateral curve.

Peak friction with load sensitivity and an inclination-angle term:

    mu(Fz, g) = lambda_mu * (mu0 + mu_fz * (Fz - Fz0)) * (1 - k_gamma * (g - g_opt)^2)

``g`` is the *turn-relative* inclination angle in degrees: negative when the
top of the tire leans toward the centre of the turn (the helpful direction).
Lateral force versus slip angle uses a simplified Magic Formula whose slope at
the origin is the cornering stiffness and whose peak is mu*Fz.
"""
from __future__ import annotations

import numpy as np

from lapsim.config.schema import MissingDataError, Tires


class Tire:
    def __init__(self, p: Tires):
        if p.mu0 is None or p.mu_fz is None:
            raise MissingDataError(
                "tires.mu0 and tires.mu_fz are not set. Fit them from TTC data "
                "or from a logged g-g envelope (the website's log import suggests a grip scale), "
                "or enter them in the car file.")
        self.p = p

    # ---- common interface (pressure is ignored by the simple model)
    def peak_fy(self, fz, gamma_deg=0.0, p=None):
        return self.mu(fz, gamma_deg) * np.asarray(fz, float)

    def peak_fx(self, fz, gamma_deg=0.0, p=None, drive=True):
        return np.maximum(self.p.mux_over_muy * self.peak_fy(fz, gamma_deg), 1.0)

    def lateral_capacity(self, fz, gamma_deg, fx, p=None):
        """Friction ellipse."""
        fmax = self.peak_fy(fz, gamma_deg)
        u = np.clip(np.abs(fx) / np.maximum(self.p.mux_over_muy * fmax, 1e-6), 0, 1)
        return np.where(u < 1, fmax * np.sqrt(1 - u ** 2), 0.0), u >= 1

    def cornering_stiffness(self, fz, gamma_deg=0.0, p=None):
        if self.p.cornering_stiffness_per_load is None:
            raise MissingDataError("tires.cornering_stiffness_per_load not set")
        return self.p.cornering_stiffness_per_load * np.asarray(fz, float)

    def mu(self, fz, gamma_deg=0.0, p=None):
        p = self.p
        m = p.lambda_mu * (p.mu0 + p.mu_fz * (np.asarray(fz, float) - p.fz0))
        if p.gamma_opt_deg is not None and p.k_gamma is not None:
            m = m * np.clip(1.0 - p.k_gamma * (np.asarray(gamma_deg, float) - p.gamma_opt_deg) ** 2, 0.3, 1.0)
        return np.maximum(m, 0.05)

    def fy(self, alpha_rad, fz, gamma_deg=0.0, p=None, C=1.4, E=0.0):
        """Lateral force [N] (simplified MF). Needs cornering_stiffness_per_load."""
        if self.p.cornering_stiffness_per_load is None:
            raise MissingDataError("tires.cornering_stiffness_per_load needed for slip-angle curves")
        D = self.mu(fz, gamma_deg) * fz
        B = self.p.cornering_stiffness_per_load * fz / (C * D)
        x = B * alpha_rad
        return D * np.sin(C * np.arctan(x - E * (x - np.arctan(x))))
