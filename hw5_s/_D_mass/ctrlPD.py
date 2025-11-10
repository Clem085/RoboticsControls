from math import pi, sqrt
import numpy as np
import massParam as P


class ctrlPD:

    def __init__(self):
        #  tuning parameters
        tr = 2.0
        zeta = 0.707
        #zeta = 1
        
        # compute PD gains
        # open loop char polynomial and poles
        # char poly is s^2 + b/m s + k/m
        a1 = P.b/P.m
        a0 = P.k/P.m
        b0 = 1/P.m
        #wn = pi/(tr*sqrt(1 - zeta**2)*2)
        wn = 2.2/tr
        alpha1 = 2.0 * zeta * wn
        alpha0 = wn**2
        # a_0 + b_0(k_p) = alpha0
        self.kp = (alpha0 - a0)/b0
        # a_1 + b_0(k_d) = alpha1
        self.kd = (alpha1 - a1)/b0
        print('kp: ', self.kp)
        print('kd: ', self.kd)

    def update(self, z_r, state):
        z = state[0][0]
        zdot = state[1][0]

        tau_tilde = self.kp * (z_r - z) - self.kd * zdot
        tau = saturate(tau_tilde, P.F_max)
        return tau


def saturate(u, limit):
    if abs(u) > limit:
        u = limit * np.sign(u)
    return u
