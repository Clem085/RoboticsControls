# ctrlEquilibrium.py
import numpy as np
import hummingbirdParam as P

class ctrlEquilibrium:
    def __init__(self):
        # Fe = (m1*ell1 + m2*ell2)*g / ellT
        Fe = (P.m1*P.ell1 + P.m2*P.ell2) * P.g / P.ellT
        # f_L = km*u_L, f_R = km*u_R  => u_L = u_R = Fe/(2*km)
        self.u_eq = float(Fe / (2.0 * P.km))

    def update(self, x):
        pwm = np.array([[self.u_eq],
                        [self.u_eq]], dtype=float)
        refs = np.array([[0.0],[0.0],[0.0]])
        return pwm, refs
