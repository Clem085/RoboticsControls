# ctrlStateFeedbackIntegrator.py
"""
LAB 5: HUMMINGBIRD SIMULATION
State feedback controller with integrators for pitch and yaw.
Connor Savugot
"""
import numpy as np
import control as cnt
import hummingbirdParam as P


class ctrlStateFeedbackIntegrator:
    def __init__(self):
        # equilibrium
        self.Fe = (P.m1 * P.ell1 + P.m2 * P.ell2) * P.g / P.ellT

        # hand-tuned gains (PI on theta; PID-like on yaw/roll)
        self.kp_theta = 0.8
        self.kd_theta = 0.2
        self.ki_theta = 0.2

        self.kp_phi = 1.0
        self.kd_phi = 0.25

        self.kp_psi = 0.1
        self.kd_psi = 0.05
        self.ki_psi = 0.03

        # limits
        self.force_limit = P.force_max
        self.torque_limit = 0.025

        # filters and integrators
        self.Ts = P.Ts
        sigma = 0.05
        self.beta = (2 * sigma - self.Ts) / (2 * sigma + self.Ts)
        self.phi_d1 = 0.0
        self.theta_d1 = 0.0
        self.psi_d1 = 0.0
        self.phi_dot = 0.0
        self.theta_dot = 0.0
        self.psi_dot = 0.0
        self.integrator_th = 0.0
        self.integrator_psi = 0.0
        self.error_th_d1 = 0.0
        self.error_psi_d1 = 0.0

    def update(self, r: np.ndarray, y: np.ndarray):
        theta_ref = r[0][0]
        psi_ref = r[1][0]
        phi = y[0][0]
        theta = y[1][0]
        psi = y[2][0]

        # dirty derivs
        self.phi_dot = self.beta * self.phi_dot + (1 - self.beta) * ((phi - self.phi_d1) / self.Ts)
        self.theta_dot = self.beta * self.theta_dot + (1 - self.beta) * ((theta - self.theta_d1) / self.Ts)
        self.psi_dot = self.beta * self.psi_dot + (1 - self.beta) * ((psi - self.psi_d1) / self.Ts)
        self.phi_d1 = phi
        self.theta_d1 = theta
        self.psi_d1 = psi

        # integrators
        error_th = theta_ref - theta
        error_psi = psi_ref - psi
        self.integrator_th += (self.Ts / 2.0) * (error_th + self.error_th_d1)
        self.integrator_psi += (self.Ts / 2.0) * (error_psi + self.error_psi_d1)
        self.integrator_th = float(np.clip(self.integrator_th, -0.3, 0.3))
        self.integrator_psi = float(np.clip(self.integrator_psi, -0.1, 0.1))
        self.error_th_d1 = error_th
        self.error_psi_d1 = error_psi

        # longitudinal control (negative feedback)
        force_unsat = self.Fe \
            - self.kp_theta * theta - self.kd_theta * self.theta_dot \
            + self.ki_theta * self.integrator_th
        force = saturate(force_unsat, 0.0, self.force_limit)

        # roll/yaw control (negative feedback on phi, psi)
        torque_unsat = (self.kp_psi * error_psi
                        - self.kd_psi * self.psi_dot
                        - self.kp_phi * phi
                        - self.kd_phi * self.phi_dot
                        + self.ki_psi * self.integrator_psi)
        torque = saturate(torque_unsat, -self.torque_limit, self.torque_limit)

        # convert to pwm
        pwm = np.array([[force + torque / P.d],
                        [force - torque / P.d]]) / (2 * P.km)
        pwm = saturate(pwm, 0, 1)
        return pwm, np.array([[0], [theta_ref], [psi_ref]])


def saturate(u, low_limit, up_limit):
    if isinstance(u, float) or isinstance(u, np.float64):
        if u > up_limit:
            u = up_limit
        if u < low_limit:
            u = low_limit
    else:
        for i in range(0, u.shape[0]):
            if u[i][0] > up_limit:
                u[i][0] = up_limit
            if u[i][0] < low_limit:
                u[i][0] = low_limit
    return u
