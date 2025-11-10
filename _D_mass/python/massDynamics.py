"""
Mass-Spring-Damper System Dynamics
Equation: m*z'' + b*z' + k*z = F
"""

import numpy as np
import massParam as P

class massDynamics:
    def __init__(self, alpha=0.0):
        self.state = np.array([[P.z0], [P.zdot0]])
        self.m = P.m * (1. + alpha * (2. * np.random.rand() - 1.))
        self.k = P.k * (1. + alpha * (2. * np.random.rand() - 1.))
        self.b = P.b * (1. + alpha * (2. * np.random.rand() - 1.))
        self.Ts = P.Ts

    def update(self, u):
        u = self.saturate(u, P.F_max)
        k1 = self.derivatives(self.state, u)
        k2 = self.derivatives(self.state + self.Ts / 2.0 * k1, u)
        k3 = self.derivatives(self.state + self.Ts / 2.0 * k2, u)
        k4 = self.derivatives(self.state + self.Ts * k3, u)
        self.state += self.Ts / 6.0 * (k1 + 2.0 * k2 + 2.0 * k3 + k4)
        return self.state

    def derivatives(self, state, u):
        z = state[0][0]
        zdot = state[1][0]
        zddot = (u - self.b * zdot - self.k * z) / self.m
        xdot = np.array([[zdot], [zddot]])
        return xdot

    def outputs(self):
        return self.state[0][0]

    def saturate(self, u, limit):
        if abs(u) > limit:
            u = limit * np.sign(u)
        return u
