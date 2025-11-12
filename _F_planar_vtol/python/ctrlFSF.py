"""
Full-State-Feedback controller for Planar VTOL (F.11).

Implements u = -K x + K_r r in the linearized hover coordinates, where
u = [F; tau] (total force and body torque), x = [z, h, theta, zdot, hdot, thetadot],
and r = [z_r; h_r]. Output is converted to motor thrusts [fr; fl] using P.mixing.
"""

from __future__ import annotations
import numpy as np
import VTOLParam as P
from VTOLDynamics import LinearVTOL

try:
    from control import place
except Exception:
    place = None


class ctrlFSF_VTOL:
    def __init__(self,
                 zeta_h: float = 0.707,
                 zeta_z: float = 0.707,
                 zeta_th: float = 0.707,
                 tr_h: float = 3.0,
                 tr_th: float = 0.3,
                 M: float = 10.0):
        lin = LinearVTOL()
        A, B, C = lin.A(), lin.B(), lin.C()
        self.A, self.B, self.C = A, B, C

        # Choose second-order pairs: altitude, theta (fast), z (slower)
        wn_h = 2.2 / float(tr_h)
        wn_th = 2.2 / float(tr_th)
        wn_z = 2.2 / float(tr_th * M)
        p_h = np.roots([1.0, 2.0 * zeta_h * wn_h, wn_h ** 2])
        p_th = np.roots([1.0, 2.0 * zeta_th * wn_th, wn_th ** 2])
        p_z = np.roots([1.0, 2.0 * zeta_z * wn_z, wn_z ** 2])
        desired_poles = np.hstack([p_h, p_th, p_z])

        # Controllability check
        def ctrb(A, B):
            blocks = [B]
            for _ in range(A.shape[0] - 1):
                blocks.append(A @ blocks[-1])
            return np.hstack(blocks)
        if np.linalg.matrix_rank(ctrb(A, B)) < A.shape[0]:
            raise RuntimeError("Linear VTOL not controllable at hover")

        if place is None:
            raise RuntimeError("python-control not available: install 'control' to place MIMO poles")
        self.K = np.asarray(place(A, B, desired_poles), dtype=float)  # (2x6)

        # Reference gain for unity DC gain in z and h
        Acl = A - B @ self.K
        C_sel = C[[0, 1], :]  # rows for z, h
        G_sel = -C_sel @ np.linalg.inv(Acl) @ B  # (2x2)
        self.Kr = np.linalg.inv(G_sel)

        # Mixing from [F; tau] to [fr; fl]
        self.mixing = P.mixing
        self.max_thrust = float(P.max_thrust)

        # Diagnostics for HW7 visibility
        try:
            rank_c = int(np.linalg.matrix_rank(ctrb(A, B)))
            cl_poles = np.linalg.eigvals(A - B @ self.K)
        except Exception:
            rank_c = -1
            cl_poles = desired_poles
        print("\n===== F.11 Full-State Feedback (VTOL) =====")
        print("Desired poles:", desired_poles)
        print("Closed-loop poles:", cl_poles)
        print("K =\n", self.K)
        print("Kr =\n", self.Kr)
        print("Controllability rank =", rank_c)

    def update(self, reference, state):
        # reference: [z_r; h_r], state: full x (6,1)
        x = np.asarray(state, float).reshape(6, 1)
        r = np.asarray(reference, float).reshape(2, 1)
        u_lin = -self.K @ x + self.Kr @ r  # [F; tau]
        motor = self.mixing @ u_lin        # [fr; fl]

        # saturate motors to physical limits
        motor[0, 0] = np.clip(motor[0, 0], 0.0, self.max_thrust)
        motor[1, 0] = np.clip(motor[1, 0], 0.0, self.max_thrust)
        return motor
