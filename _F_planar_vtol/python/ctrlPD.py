"""
VTOL PD Controller with Successive Loop Closure
Control Architecture:
  - Altitude (h): Direct PD
  - Pitch (theta): Inner loop PD
  - Lateral (z): Outer loop PD
"""

import numpy as np
import VTOLParam as P

class ctrlPD:
    def __init__(self):
        self.m = P.mc + 2.0 * P.mr
        self.J = P.Jc + 2.0 * P.mr * (P.d**2)
        self.mu = P.mu
        self.g = P.g
        self.d = P.d
        self.Fe = self.m * self.g
        
        self.tr_h = 2.0
        self.zeta_h = 0.7
        self.tr_theta = 0.8
        self.zeta_theta = 0.7
        self.tr_z = 4.0
        self.zeta_z = 0.7
        
        self._compute_altitude_gains()
        self._compute_lateral_gains()
        
        self.F_max = 2.0 * P.max_thrust
        self.tau_max = P.max_thrust * self.d
        self.theta_max = 30.0 * np.pi / 180.0

    def _compute_altitude_gains(self):
        wn_h = 2.2 / self.tr_h
        self.kp_h = self.m * (wn_h**2)
        self.kd_h = self.m * 2.0 * self.zeta_h * wn_h
        
        print("------------------------------------------------------------")
        print("Altitude Controller (h) Gains:")
        print("  tr_h = {:.2f} s, zeta_h = {:.2f}".format(self.tr_h, self.zeta_h))
        print("  wn_h = {:.4f} rad/s".format(wn_h))
        print("  kp_h = {:.4f}".format(self.kp_h))
        print("  kd_h = {:.4f}".format(self.kd_h))
        print("------------------------------------------------------------")

    def _compute_lateral_gains(self):
        wn_theta = 2.2 / self.tr_theta
        self.kp_theta = self.J * (wn_theta**2)
        self.kd_theta = self.J * 2.0 * self.zeta_theta * wn_theta
        
        print("\nLateral Control - Inner Loop (theta) Gains:")
        print("  tr_theta = {:.2f} s, zeta_theta = {:.2f}".format(self.tr_theta, self.zeta_theta))
        print("  wn_theta = {:.4f} rad/s".format(wn_theta))
        print("  kp_theta = {:.6f}".format(self.kp_theta))
        print("  kd_theta = {:.6f}".format(self.kd_theta))
        
        wn_z = 2.2 / self.tr_z
        self.kp_z = self.m * (wn_z**2) / self.Fe
        self.kd_z = (2.0 * self.zeta_z * wn_z - self.mu / self.m) * self.m / self.Fe
        
        print("\nLateral Control - Outer Loop (z) Gains:")
        print("  tr_z = {:.2f} s, zeta_z = {:.2f}".format(self.tr_z, self.zeta_z))
        print("  wn_z = {:.4f} rad/s".format(wn_z))
        print("  kp_z = {:.6f}".format(self.kp_z))
        print("  kd_z = {:.6f}".format(self.kd_z))
        print("------------------------------------------------------------\n")

    def update(self, z_r, z, h_r, h, theta, zdot, hdot, thetadot):
        F = self.Fe + self.kp_h * (h_r - h) + self.kd_h * (0.0 - hdot)
        F = self.saturate(F, self.F_max)
        
        theta_c = -(self.m / self.Fe) * (
            self.kp_z * (z_r - z) + self.kd_z * (0.0 - zdot)
        )
        theta_c = self.saturate(theta_c, self.theta_max)
        
        tau = self.kp_theta * (theta_c - theta) + self.kd_theta * (0.0 - thetadot)
        tau = self.saturate(tau, self.tau_max)
        
        return F, tau

    def saturate(self, u, limit):
        if abs(u) > limit:
            u = limit * np.sign(u)
        return u
