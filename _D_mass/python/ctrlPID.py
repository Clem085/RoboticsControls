import numpy as np
import massParam as P


class ctrlPID:
    def __init__(self):
        # tuning parameters from solution
        tr = 2.0
        zeta = 0.707
        self.ki = 1.5  # integrator gain

        # compute PD gains based on desired polynomial
        # plant: m zddot + b zdot + k z = F
        a1 = P.b / P.m
        a0 = P.k / P.m
        b0 = 1 / P.m
        wn = 2.2 / tr
        alpha1 = 2.0 * zeta * wn
        alpha0 = wn ** 2
        # a0 + b0*kp = alpha0
        self.kp = (alpha0 - a0) / b0
        # a1 + b0*kd = alpha1
        self.kd = (alpha1 - a1) / b0
        print('kp: ', self.kp)
        print('ki: ', self.ki)
        print('kd: ', self.kd)

        # dirty derivative parameters
        self.sigma = 0.05
        self.beta = (2.0 * self.sigma - P.Ts) / (2.0 * self.sigma + P.Ts)

        # states for integrator/differentiator
        self.z_dot = P.zdot0
        self.z_d1 = P.z0
        self.error_dot = 0.0
        self.error_d1 = 0.0
        self.integrator = 0.0

    def update(self, z_r, y):
        # y may be scalar or 1x1/2D; extract z
        z = y if np.isscalar(y) else y[0][0]

        # error
        error = z_r - z

        # integrate error (trapezoidal)
        self.integrator = self.integrator + (P.Ts / 2) * (error + self.error_d1)

        # differentiate z (dirty derivative)
        self.z_dot = self.beta * self.z_dot + (1 - self.beta) * ((z - self.z_d1) / P.Ts)

        # PID control
        F_unsat = self.kp * error + self.ki * self.integrator - self.kd * self.z_dot
        F = saturate(F_unsat, P.F_max)

        # integrator anti-windup (back-calculation)
        if self.ki != 0.0:
            self.integrator = self.integrator + P.Ts / self.ki * (F - F_unsat)

        # update delays
        self.error_d1 = error
        self.z_d1 = z
        return F


def saturate(u, limit):
    if abs(u) > limit:
        u = limit * np.sign(u)
    return u

