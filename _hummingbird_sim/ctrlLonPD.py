import numpy as np
import hummingbirdParam as P


class CtrlLonPD:
    """
    Longitudinal PD controller for hummingbird pitch (theta).

    update(theta, theta_d, Ts) -> F_cmd (N)
      - PD on theta with dirty derivative for theta_dot
      - Adds equilibrium feedforward F_e
    """

    def __init__(self, params=P,
                 zeta_theta: float = 0.707,
                 tr_theta: float = 0.6,
                 tau_d: float = 0.05):
        self.P = params
        self.tau_d = tau_d

        # Longitudinal plant: theta_ddot = b_theta * (F - F_e)
        denom = (
            self.P.m1 * self.P.ell1**2
            + self.P.m2 * self.P.ell2**2
            + self.P.J1y
            + self.P.J2y
        )
        self.b_theta = self.P.ellT / denom
        self.Fe = self.P.Fe

        # Map (zeta, tr) -> gains
        self.zeta_theta = zeta_theta
        self.tr_theta = tr_theta
        self.wn_theta = 2.2 / self.tr_theta
        self.kp_theta = (self.wn_theta**2) / self.b_theta
        self.kd_theta = (2.0 * self.zeta_theta * self.wn_theta) / self.b_theta

        # Dirty derivative state
        self.theta_prev = 0.0
        self.dtheta_hat = 0.0

    def reset(self):
        self.theta_prev = 0.0
        self.dtheta_hat = 0.0

    def update(self, theta: float, theta_d: float, Ts: float) -> float:
        # Dirty derivative of theta_dot
        alpha = self.tau_d / (self.tau_d + Ts)
        self.dtheta_hat = (
            alpha * self.dtheta_hat
            + (1.0 - alpha) * (theta - self.theta_prev) / Ts
        )
        self.theta_prev = theta

        # PD control on theta
        e_theta = theta_d - theta
        F_tilde = self.kp_theta * e_theta + self.kd_theta * (0.0 - self.dtheta_hat)
        F_cmd = self.Fe + F_tilde
        return F_cmd

