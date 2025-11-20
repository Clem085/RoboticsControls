"""
Full-state feedback with integral action for the hover-linearized VTOL (Homework F.12).

Adds integrators (with anti-windup) on both the lateral position (z) and altitude (h)
loops. Control law (hover coordinates):
    u = -Kx - Ki * xi + Kr * r,   xi_dot = r - C_sel x
with u = [F; tau], r = [z_r; h_r].
"""

from __future__ import annotations

import numpy as np
import VTOLParam as P
from VTOLDynamics import LinearVTOL

try:
    from control import place  # type: ignore
except Exception as exc:  # pragma: no cover
    raise RuntimeError("python-control is required for VTOL integral FSF") from exc


class ctrlFSFInt_VTOL:
    def __init__(
        self,
        zeta_h: float = 0.707,
        zeta_z: float = 0.707,
        zeta_th: float = 0.707,
        tr_h: float = 3.0,
        tr_th: float = 0.3,
        M: float = 10.0,
        p_int_h: float = -1.5,
        p_int_z: float = -1.0,
        use_anti_windup: bool = True,
    ):
        lin = LinearVTOL()
        A, B, C = lin.A(), lin.B(), lin.C()
        self.A, self.B, self.C = A, B, C
        self.C_sel = C[[0, 1], :]  # outputs for z and h
        self.Ts = float(P.Ts)
        self.use_aw = bool(use_anti_windup)

        # Desired pole sets (same pairing as F.11 plus integral poles).
        wn_h = 2.2 / float(tr_h)
        wn_th = 2.2 / float(tr_th)
        wn_z = 2.2 / float(tr_th * M)
        p_h = np.roots([1.0, 2.0 * zeta_h * wn_h, wn_h ** 2])
        p_th = np.roots([1.0, 2.0 * zeta_th * wn_th, wn_th ** 2])
        p_z = np.roots([1.0, 2.0 * zeta_z * wn_z, wn_z ** 2])
        desired_poles = np.hstack([p_h, p_th, p_z, [float(p_int_h), float(p_int_z)]])

        A_aug = np.block([[A, np.zeros((6, 2))],
                          [-self.C_sel, np.zeros((2, 2))]])
        B_aug = np.vstack([B, np.zeros((2, 2))])
        K_aug = np.asarray(place(A_aug, B_aug, desired_poles), dtype=float)

        self.Kx = K_aug[:, :6]
        self.Ki = K_aug[:, 6:]

        # Reference gain (matches F.11 for nominal unity DC gain).
        Acl = A - B @ self.Kx
        G_sel = -self.C_sel @ np.linalg.inv(Acl) @ B
        self.Kr = np.linalg.inv(G_sel)

        self.mixing = P.mixing
        self.unmixing = P.unmixing
        self.max_thrust = float(P.max_thrust)

        self.integrator = np.zeros((2, 1))
        self.error_prev = np.zeros((2, 1))

        try:
            def ctrb(matA, matB):
                blocks = [matB]
                for _ in range(matA.shape[0] - 1):
                    blocks.append(matA @ blocks[-1])
                return np.hstack(blocks)
            rank_c = int(np.linalg.matrix_rank(ctrb(A_aug, B_aug)))
            cl_poles = np.linalg.eigvals(A_aug - B_aug @ K_aug)
        except Exception:  # pragma: no cover
            rank_c = -1
            cl_poles = desired_poles
        print("\n===== F.12 FSF + Integrators =====")
        print("Desired poles:", desired_poles)
        print("Closed-loop poles:", cl_poles)
        print("Kx =\n", self.Kx)
        print("Ki =\n", self.Ki)
        print("Kr =\n", self.Kr)
        print("Controllability rank (augmented) =", rank_c)

    def _saturate_motors(self, thrusts: np.ndarray) -> np.ndarray:
        thrusts = np.asarray(thrusts, dtype=float).reshape(2, 1)
        thrusts[0, 0] = np.clip(thrusts[0, 0], 0.0, self.max_thrust)
        thrusts[1, 0] = np.clip(thrusts[1, 0], 0.0, self.max_thrust)
        return thrusts

    def update(self, reference, state):
        x = np.asarray(state, dtype=float).reshape(6, 1)
        r = np.asarray(reference, dtype=float).reshape(2, 1)
        measured = self.C_sel @ x
        error = r - measured

        integ_inc = 0.5 * self.Ts * (error + self.error_prev)
        self.integrator += integ_inc

        u_lin = -self.Kx @ x - self.Ki @ self.integrator + self.Kr @ r  # [F; tau]
        motor_unsat = self.mixing @ u_lin
        motor_sat = self._saturate_motors(motor_unsat.copy())

        if self.use_aw and np.any(np.abs(motor_unsat - motor_sat) > 1e-9):
            # Freeze integrators when saturating to prevent windup
            self.integrator -= integ_inc
            u_lin = -self.Kx @ x - self.Ki @ self.integrator + self.Kr @ r
            motor_unsat = self.mixing @ u_lin
            motor_sat = self._saturate_motors(motor_unsat.copy())

        self.error_prev = error
        return motor_sat

