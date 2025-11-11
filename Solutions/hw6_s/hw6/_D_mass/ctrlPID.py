import numpy as np
import massParam as P


class ctrlPID:
    def __init__(self):
        #  tuning parameters
        tr = 2.0
        zeta = 0.707
        self.ki = 1.5  # integrator gain

        # compute PD gains
        # open loop char polynomial and poles
        # char poly is s^2 + b/m s + k/m
        a1 = P.b / P.m
        a0 = P.k / P.m
        b0 = 1/P.m
        wn = 2.2/tr
        alpha1 = 2.0 * zeta * wn
        alpha0 = wn**2
        # a_0 + b_0(k_p) = alpha0
        self.kp = (alpha0 - a0)/b0
        # a_1 + b_0(k_d) = alpha1
        self.kd = (alpha1 - a1)/b0
        print('kp: ', self.kp)
        print('ki: ', self.ki)
        print('kd: ', self.kd)

        # dirty derivative gains
        self.sigma = 0.05  
        self.beta = (2.0 * self.sigma - P.Ts) / (2.0 * self.sigma + P.Ts)  # dirty derivative gain

        #----------------------------------------------------------
        # variables for integrator and differentiator
        self.z_dot = P.zdot0             # estimated derivative of y
        self.z_d1 = P.z0              # Signal y delayed by one sample
        self.error_dot = 0.0         # estimated derivative of error
        self.error_d1 = 0.0          # Error delayed by one sample
        self.integrator = 0.0        # integrator
                
    def update(self, z_r, y):
        z = y[0][0]
        # Compute the current error
        error = z_r - z

        # integrate error
        self.integrator = self.integrator + (P.Ts / 2) * (error + self.error_d1)

        # differentiate z
        self.z_dot = self.beta * self.z_dot \
                             + (1 - self.beta) * ((z - self.z_d1) / P.Ts)
        
        # PID control
        F_unsat = self.kp * error + self.ki * self.integrator - self.kd * self.z_dot
        F = saturate(F_unsat, P.F_max)

        # integrator anti - windup
        if self.ki != 0.0:
            self.integrator = self.integrator + P.Ts / self.ki * (F - F_unsat)
            
        # update delayed variables
        self.error_d1 = error
        self.z_d1 = z
        return F


def saturate(u, limit):
    if abs(u) > limit:
        u = limit * np.sign(u)
    return u







