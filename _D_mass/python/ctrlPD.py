"""
PD Controller for Mass-Spring-Damper System
Control Law: F = kP*(z_r - z) - kD*z'
"""

import numpy as np
import massParam as P

class ctrlPD:
    def __init__(self):
        # Match solution tuning (zeta ~= 0.707)
        tr = 2.0
        zeta = 0.707
        wn = 2.2 / tr

        alpha1 = 2.0 * zeta * wn
        alpha0 = wn**2

        self.pole_real = -zeta * wn
        self.pole_imag = wn * np.sqrt(1 - zeta**2)

        a1 = P.b / P.m
        a0 = P.k / P.m
        b0 = 1.0 / P.m

        # Gains (kp, kd) equivalent to solution
        self.kP = (alpha0 - a0) / b0
        self.kD = (alpha1 - a1) / b0

        print('\n' + '='*60)
        print('HOMEWORK D.8(a): PD Controller Design')
        print('='*60)
        print('  tr = {:.3f} s, zeta = {:.3f}'.format(tr, zeta))
        print('  wn = {:.4f} rad/s'.format(wn))
        print('  Desired poles: {:.4f} +/- j{:.4f}'.format(self.pole_real, self.pole_imag))
        print('  kP = {:.4f}, kD = {:.4f}'.format(self.kP, self.kD))
        print('='*60 + '\n')

    def update(self, z_r, state):
        z = state[0][0]
        zdot = state[1][0]
        F = self.kP * (z_r - z) - self.kD * zdot
        # Saturate to match solution behavior
        return saturate(F, P.F_max)


class ctrlPD_saturated:
    def __init__(self):
        # Keep a slower, non-saturating design for part (b)
        tr = 3.5
        zeta = 0.707
        wn = 2.2 / tr
        
        alpha1 = 2.0 * zeta * wn
        alpha0 = wn**2
        
        self.pole_real = -zeta * wn
        self.pole_imag = wn * np.sqrt(1 - zeta**2)
        
        a1 = P.b / P.m
        a0 = P.k / P.m
        b0 = 1.0 / P.m
        
        self.kP = (alpha0 - a0) / b0
        self.kD = (alpha1 - a1) / b0
        
        F_max_expected = self.kP * 1.0
        
        print('\n' + '='*60)
        print('HOMEWORK D.8(b): PD Controller with Saturation')
        print('='*60)
        print('Saturation Constraint: F_max = {} N'.format(P.F_max))
        print('Step Input Size: 1.0 meter')
        print('\nTuning Strategy:')
        print('  Adjusted rise time: tr = {} seconds'.format(tr))
        print('  Damping ratio: zeta = {}'.format(zeta))
        print('\nTuned Performance:')
        print('  Natural frequency (wn): {:.4f} rad/s'.format(wn))
        print('  Tuned poles: {:.4f} +/- j{:.4f}'.format(self.pole_real, self.pole_imag))
        print('\nTuned PD Gains:')
        print('  kP = {:.4f}'.format(self.kP))
        print('  kD = {:.4f}'.format(self.kD))
        print('\nExpected Maximum Force:')
        print('  F_max (expected) = {:.4f} N'.format(F_max_expected))
        print('  F_max (limit) = {} N'.format(P.F_max))
        if F_max_expected <= P.F_max:
            print('  Status: Controller will NOT saturate for 1m step')
        else:
            print('  Status: Controller WILL saturate (may need further tuning)')
        print('='*60 + '\n')

    def update(self, z_r, state):
        z = state[0][0]
        zdot = state[1][0]
        F = self.kP * (z_r - z) - self.kD * zdot
        return saturate(F, P.F_max)


def saturate(u, limit):
    if abs(u) > limit:
        u = limit * np.sign(u)
    return u
