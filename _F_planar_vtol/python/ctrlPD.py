"""
VTOL PD Controller matching solution outcomes:
- Altitude (h): PD on (h, hdot) with equilibrium force add-in
- Lateral: outer z-loop sets theta_r; inner theta-loop PD
- Returns motor thrusts [fr; fl] like the solution controller
"""

import numpy as np
import VTOLParam as P


class ctrlPD:
    def __init__(self):
        # physical params
        self.m = P.mc + 2.0 * P.mr
        self.J = P.Jc + 2.0 * P.mr * (P.d ** 2)
        self.mu = P.mu
        self.d = P.d
        self.Fe = (P.mc + 2.0 * P.mr) * P.g

        # tuning per solution
        zeta_h = 0.707
        zeta_z = 0.707
        zeta_th = 0.707

        tr_h = 3.0           # altitude loop (no saturation)
        tr_th = 0.3          # inner theta loop
        M = 10.0             # time-scale separation
        tr_z = tr_th * M     # outer z loop

        # altitude gains
        wn_h = 2.2 / tr_h
        self.kp_h = (wn_h ** 2) * (P.mc + 2.0 * P.mr)
        self.kd_h = (2.0 * zeta_h * wn_h) * (P.mc + 2.0 * P.mr)

        # inner theta loop gains
        wn_th = 2.2 / tr_th
        b0 = P.Jc + 2.0 * P.mr * (P.d ** 2)
        self.kp_th = (wn_th ** 2) * b0
        self.kd_th = (2.0 * zeta_th * wn_th) * b0

        # outer z loop gains
        wn_z = 2.2 / tr_z
        m = P.mc + 2.0 * P.mr
        self.kp_z = -wn_z ** 2 / P.g
        self.kd_z = (P.mu / m - 2.0 * zeta_z * wn_z) / P.g

        # limits
        self.theta_max = 10.0 * np.pi / 180.0
        self.F_limit = 2.0 * P.max_thrust
        self.tau_limit = 2.0 * P.max_thrust * P.d

    def update(self, reference: np.ndarray, state: np.ndarray) -> np.ndarray:
        z_r = reference[0][0]
        h_r = reference[1][0]
        z = state[0][0]
        h = state[1][0]
        theta = state[2][0]
        zdot = state[3][0]
        hdot = state[4][0]
        thetadot = state[5][0]

        # Altitude force
        F_tilde = self.kp_h * (h_r - h) - self.kd_h * hdot
        F = saturate(F_tilde + self.Fe, self.F_limit)

        # Lateral outer loop: z -> theta_r
        theta_r = saturate(self.kp_z * (z_r - z) - self.kd_z * zdot, self.theta_max)

        # Inner theta loop: torque
        tau = saturate(self.kp_th * (theta_r - theta) - self.kd_th * thetadot, self.tau_limit)

        # Convert to individual motor thrusts to match solution output
        motor_thrusts = P.mixing @ np.array([[F], [tau]])
        return motor_thrusts


def saturate(u, limit):
    if abs(u) > limit:
        u = limit * np.sign(u)
    return u
