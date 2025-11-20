"""
Full-state feedback with integral action for the mass-spring-damper (Homework D.12).

Adds an integrator on the position error (z_r - z) with simple anti-windup.
Control law:
    u = -K x_hat - k_i * xi + k_r * z_r
where xi is the integral of the position error and x_hat = [z, z_dot_hat].
"""

from __future__ import annotations

import numpy as np
import massParam as P

try:
    from control import place  # type: ignore
except Exception:  # pragma: no cover
    place = None


def _place_siso_ackermann(A: np.ndarray, B: np.ndarray, poles) -> np.ndarray:
    """Ackermann-like pole placement for controllable SISO systems."""
    n = A.shape[0]
    ctrb = B
    Ab = B
    for _ in range(1, n):
        Ab = A @ Ab
        ctrb = np.hstack((ctrb, Ab))
    if np.linalg.matrix_rank(ctrb) < n:
        raise RuntimeError("System not controllable; cannot place poles.")

    coeffs = np.poly(poles)  # [1, a_{n-1}, ..., a0] for desired (s^n + ...)
    phi = np.zeros_like(A, dtype=float)
    for i in range(n):
        power = n - i
        phi += coeffs[i] * np.linalg.matrix_power(A, power)
    phi += coeffs[-1] * np.eye(n)
    eT = np.zeros((1, n), dtype=float)
    eT[0, -1] = 1.0
    return eT @ np.linalg.inv(ctrb) @ phi


class ctrlFSFInt:
    def __init__(
        self,
        tr: float = 2.0,
        zeta: float = 0.707,
        sigma: float = 0.05,
        p_int: float = -2.5,
        use_anti_windup: bool = True,
    ):
        self.sigma = float(sigma)
        self.Ts = float(P.Ts)
        self.use_aw = bool(use_anti_windup)
        self._z_prev = 0.0
        self._zdot_hat = 0.0
        self._error_prev = 0.0
        self._integrator = 0.0

        m, b, k = float(P.m), float(P.b), float(P.k)
        self.A = np.array([[0.0, 1.0],
                           [-k / m, -b / m]], dtype=float)
        self.B = np.array([[0.0],
                           [1.0 / m]], dtype=float)
        self.C = np.array([[1.0, 0.0]], dtype=float)

        wn = 2.2 / float(tr)
        base_poles = np.roots([1.0, 2.0 * zeta * wn, wn ** 2])
        p_int = float(-abs(p_int))
        desired_poles = np.hstack((base_poles, [p_int]))

        A_aug = np.block([[self.A, np.zeros((2, 1))],
                          [-self.C, np.zeros((1, 1))]])
        B_aug = np.vstack((self.B, [[0.0]]))

        if place is not None:
            K_aug = np.asarray(place(A_aug, B_aug, desired_poles), dtype=float)
        else:  # pragma: no cover
            K_aug = _place_siso_ackermann(A_aug, B_aug, desired_poles)

        self.K = K_aug[:, :2]
        self.ki = float(K_aug[:, 2])

        # Reference gain keeps nominal unity DC gain (same formula as D.11).
        self.kr = float(-1.0 / (self.C @ np.linalg.inv(self.A - self.B @ self.K) @ self.B))

        try:
            ctrb = np.hstack([self.B, self.A @ self.B])
            rank_c = int(np.linalg.matrix_rank(ctrb))
        except Exception:  # pragma: no cover
            rank_c = -1
        print("\n===== D.12 FSF + Integrator =====")
        print("Desired poles:", desired_poles)
        print("K =", self.K)
        print("ki =", self.ki)
        print("kr =", self.kr)
        print("Controllability rank =", rank_c)

    def _dirty_derivative(self, z: float) -> float:
        a1 = (2.0 * self.sigma - self.Ts) / (2.0 * self.sigma + self.Ts)
        a2 = 2.0 / (2.0 * self.sigma + self.Ts)
        self._zdot_hat = a1 * self._zdot_hat + a2 * (z - self._z_prev)
        self._z_prev = z
        return self._zdot_hat

    def update(self, z_r: float, y_meas) -> float:
        z = float(np.asarray(y_meas).flatten()[0])
        zdot_hat = self._dirty_derivative(z)

        error = float(z_r - z)
        integ_inc = 0.5 * self.Ts * (error + self._error_prev)
        self._integrator += integ_inc

        xhat = np.array([[z], [zdot_hat]], dtype=float)
        u_unsat = float(-self.K @ xhat - self.ki * self._integrator + self.kr * z_r)
        u_sat = float(np.clip(u_unsat, -P.F_max, P.F_max))

        if self.use_aw and abs(u_unsat - u_sat) > 1e-9:
            self._integrator -= integ_inc  # freeze integrator when saturated
            u_unsat = float(-self.K @ xhat - self.ki * self._integrator + self.kr * z_r)
            u_sat = float(np.clip(u_unsat, -P.F_max, P.F_max))

        self._error_prev = error
        return u_sat

