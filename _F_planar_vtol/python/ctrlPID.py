import numpy as np
import VTOLParam as P


class ctrlPID:
    def __init__(self):
        # dirty derivative cutoff
        self.sigma = 0.05

        # damping ratios (tuned close to solution)
        zeta_h = 0.95
        zeta_z = 0.95
        zeta_th = 0.95

        # saturation and equilibrium
        self.theta_max = 10.0 * np.pi / 180.0
        self.Fe = (P.mc + 2.0 * P.mr) * P.g

        # altitude loop tuning
        tr_h = 3.0
        wn_h = 0.5 * np.pi / (tr_h * np.sqrt(1 - zeta_h ** 2))
        self.kp_h = (wn_h ** 2.0) * (P.mc + 2.0 * P.mr)
        self.kd_h = (2.0 * zeta_h * wn_h) * (P.mc + 2.0 * P.mr)
        self.ki_h = 1.0

        # inner theta loop
        tr_th = 0.5
        b0 = 1.0 / (P.Jc + 2.0 * P.mr * P.d ** 2)
        wn_th = 0.5 * np.pi / (tr_th * np.sqrt(1 - zeta_th ** 2))
        self.kp_th = (wn_th ** 2.0) / b0
        self.kd_th = (2.0 * zeta_th * wn_th) / b0

        # outer lateral loop
        M = 10.0
        tr_z = tr_th * M
        b1 = -self.Fe / (P.mc + 2.0 * P.mr)
        a1 = P.mu / (P.mc + 2.0 * P.mr)
        wn_z = 0.5 * np.pi / (tr_z * np.sqrt(1 - zeta_z ** 2))
        self.kp_z = (wn_z ** 2.0) / b1
        self.kd_z = (2.0 * zeta_z * wn_z - a1) / b1
        self.ki_z = 0.0  # per solution comment

        # prints
        print('kp_z: ', self.kp_z)
        print('kd_z: ', self.kd_z)
        print('ki_z: ', self.ki_z)
        print('kp_h: ', self.kp_h)
        print('kd_h: ', self.kd_h)
        print('kp_th: ', self.kp_th)
        print('kd_th: ', self.kd_th)

        # integrator/differentiator states
        self.integrator_h = 0.0
        self.error_h_prev = 0.0
        self.h_dot = P.hdot0
        self.h_prev = P.h0

        self.integrator_z = 0.0
        self.error_z_prev = 0.0
        self.z_dot = P.zdot0
        self.z_prev = P.z0

        self.theta_dot = P.thetadot0
        self.theta_prev = P.theta0

    def update(self, r, y):
        z_r = r[0][0]
        h_r = r[1][0]
        z = y[0][0]
        h = y[1][0]
        theta = y[2][0]

        # altitude loop
        error_h = h_r - h
        self.h_dot = (2 * self.sigma - P.Ts) / (2 * self.sigma + P.Ts) * self.h_dot + \
                     (2.0 / (2.0 * self.sigma + P.Ts)) * (h - self.h_prev)
        if np.abs(self.h_dot) < 0.5:
            self.integrator_h = self.integrator_h + (P.Ts / 2) * (error_h + self.error_h_prev)
        F_tilde = self.kp_h * (h_r - h) + self.ki_h * self.integrator_h - self.kd_h * self.h_dot
        # total force saturation by per-motor limit: <= 2*max_thrust
        F = saturate(F_tilde + self.Fe, 2.0 * getattr(P, 'max_thrust', 10.0))

        # lateral outer loop
        error_z = z_r - z
        self.z_dot = (2 * self.sigma - P.Ts) / (2 * self.sigma + P.Ts) * self.z_dot + \
                     (2.0 / (2.0 * self.sigma + P.Ts)) * (z - self.z_prev)
        if np.abs(self.z_dot) < 0.5:
            self.integrator_z = self.integrator_z + (P.Ts / 2) * (error_z + self.error_z_prev)
        theta_r = self.kp_z * error_z + self.ki_z * self.integrator_z - self.kd_z * self.z_dot

        # inner theta loop
        self.theta_dot = (2 * self.sigma - P.Ts) / (2 * self.sigma + P.Ts) * self.theta_dot + \
                         (2.0 / (2.0 * self.sigma + P.Ts)) * (theta - self.theta_prev)
        # torque saturation: approx by max per-motor thrust and arm
        tau = saturate(self.kp_th * (theta_r - theta) - self.kd_th * self.theta_dot,
                       getattr(P, 'max_thrust', 10.0) * P.d)

        # update delayed variables
        self.error_h_prev = error_h
        self.h_prev = h
        self.error_z_prev = error_z
        self.z_prev = z
        self.theta_prev = theta

        motor_thrusts = P.mixing @ np.array([[F], [tau]])
        return motor_thrusts


def saturate(u, limit):
    if abs(u) > limit:
        u = limit * np.sign(u)
    return u

