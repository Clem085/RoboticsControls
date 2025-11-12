"""
Full-State-Feedback controller for the Mass–Spring–Damper (D.11).

Implements u = -K x_hat + k_r * z_r, where x_hat = [z, z_dot_hat].
z_dot_hat is provided by a dirty-derivative (filtered differentiator) using
only the measured position z.

Dependencies: python-control (module name: control)
"""

from __future__ import annotations
import numpy as np
import massParam as P

try:
    from control import place  # supports SISO/MIMO pole placement
except Exception as _:
    place = None


class ctrlFSF:
    def __init__(self, tr: float = 2.0, zeta: float = 0.707, sigma: float = 0.05):
        # Plant matrices from D.6
        m, b, k = float(P.m), float(P.b), float(P.k)
        self.A = np.array([[0.0, 1.0],
                           [-k/m, -b/m]], dtype=float)
        self.B = np.array([[0.0],
                           [1.0/m]], dtype=float)
        self.C = np.array([[1.0, 0.0]], dtype=float)

        # Desired second-order poles from D.8
        wn = 2.2 / float(tr)
        p = np.roots([1.0, 2.0 * zeta * wn, wn ** 2])

        # Place poles
        if place is None:
            # Manual 2x2 SISO place using matching with desired char poly
            # A - B K has char poly s^2 + a1 s + a0 with a1 = 2 zeta wn, a0 = wn^2
            a1_des = 2.0 * zeta * wn
            a0_des = wn ** 2
            a1 = -np.trace(self.A)
            a0 = np.linalg.det(self.A)
            # For A = [[0,1],[-k/m,-b/m]], B = [[0],[1/m]]
            # Standard formulas for PD-equivalent gains
            kp = (a0_des - (k / m)) * m
            kd = (a1_des - (b / m)) * m
            self.K = np.array([[kp, kd]], dtype=float)
        else:
            self.K = np.asarray(place(self.A, self.B, p), dtype=float)  # (1,2)

        # Reference gain for unity DC gain from z_r to z
        self.kr = float(-1.0 / (self.C @ np.linalg.inv(self.A - self.B @ self.K) @ self.B))

        # Diagnostics for HW7 visibility
        try:
            ctrb = np.hstack([self.B, self.A @ self.B])
            rank_c = int(np.linalg.matrix_rank(ctrb))
        except Exception:
            rank_c = -1
        print("\n===== D.11 Full-State Feedback =====")
        print("Desired poles:", p)
        print("K =", self.K)
        print("kr =", self.kr)
        print("Controllability rank =", rank_c)

        # Dirty derivative state
        self.sigma = float(sigma)
        self.Ts = float(P.Ts)
        self._z_prev = 0.0
        self._zdot_hat = 0.0

    def _dirty_derivative(self, z: float) -> float:
        # y' ≈ (2/(2σ+Ts)) (z - z_prev) + ((2σ-Ts)/(2σ+Ts)) y'_prev
        a1 = (2.0 * self.sigma - self.Ts) / (2.0 * self.sigma + self.Ts)
        a2 = 2.0 / (2.0 * self.sigma + self.Ts)
        self._zdot_hat = a1 * self._zdot_hat + a2 * (z - self._z_prev)
        self._z_prev = z
        return self._zdot_hat

    def update(self, z_r: float, y_meas) -> float:
        # y_meas is z (scalar or 1x1 array). Only z is used; z_dot is estimated.
        if isinstance(y_meas, np.ndarray):
            z = float(y_meas.flatten()[0])
        else:
            z = float(y_meas)

        zdot_hat = self._dirty_derivative(z)
        xhat = np.array([[z], [zdot_hat]], dtype=float)
        u = float(-self.K @ xhat + self.kr * float(z_r))
        # Saturate to plant limit
        return np.clip(u, -P.F_max, P.F_max)
