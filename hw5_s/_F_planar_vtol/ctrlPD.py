from ast import Del
import numpy as np
import VTOLParam as P


class ctrlPD:
    def __init__(self):
        #--------------------------------------------------
        # defining parameters for damping behavior in different loops
        #--------------------------------------------------
        zeta_h = 0.707  # damping ratio for altitude
        zeta_z = 0.707  # damping ratio for outer lateral loop
        zeta_th = 0.707  # damping ratio for inner lateral loop

        # saturation limits
        self.theta_max = 10.0 * np.pi / 180.0  # Max theta, rads
        self.Fe = (P.mc + 2.0 * P.mr) * P.g  # equilibrium force        

        #---------------------------------------------------
        # tuning parameters for longitudinal dynamics 
        #---------------------------------------------------
        # Finding PD gains for longitudinal (altitude) control
        tr_h = 3.0  # rise time for altitude - tuned for best rise time without saturation
        #tr_h = 8.0
        wn_h = 2.2/tr_h   # natural frequency
        Delta_cl_d = [1, 2*zeta_h*wn_h, wn_h**2.0]  # desired closed loop char eq
         # use coefficient matching to find gains
        self.kp_h = Delta_cl_d[2]*(P.mc+2.0*P.mr)  # kp - altitude
        self.kd_h = Delta_cl_d[1]*(P.mc+2.0*P.mr)  # kd - altitude

        #---------------------------------------------------        
        # tuning parameters for longitudinal dynamics 
        #---------------------------------------------------
        # Finding PD gains for lateral inner loop
        tr_th = 0.3
        #tr_th = 0.8
        b0 = P.Jc+2.0*P.mr*P.d**2
        wn_th = 2.2/tr_th
        self.kp_th = (wn_th**2.0) * b0
        self.kd_th = (2.0*zeta_th*wn_th) * b0

        #---------------------------------------------------
        # Finding PD gain for lateral outer loop
        M  = 10.0  # time separation between inner and outer lateral loops
        tr_z = tr_th*M  # rise time for outer lateral loop (position) - tuned for best rise time without saturation        
        wn_z     = 2.2/tr_z
        m = P.mc+2*P.mr
        self.kp_z   = -wn_z**2.0/P.g
        self.kd_z   = (P.mu/m - 2.0*zeta_z*wn_z)/P.g

        print('kp_z: ', self.kp_z)
        print('kd_z: ', self.kd_z)
        print('kp_h: ', self.kp_h)
        print('kd_h: ', self.kd_h)
        print('kp_th: ', self.kp_th)
        print('kd_th: ', self.kd_th)

    def update(self, reference, state):
        z_r = reference[0][0]
        h_r = reference[1][0]
        z = state[0][0]
        h = state[1][0]
        theta = state[2][0]
        zdot = state[3][0]
        hdot = state[4][0]
        thetadot = state[5][0]

        # calculate control for the altitude loop
        F_tilde = self.kp_h * (h_r - h) - self.kd_h * hdot
        F = saturate( F_tilde + self.Fe, 2*P.F_max)
        
        # calculate theta_r from the outer loop
        theta_r = saturate( self.kp_z * (z_r - z) - self.kd_z * zdot, self.theta_max)
        
        # calculate torque from the inner loop
        tau = saturate( self.kp_th * (theta_r - theta) - self.kd_th * thetadot, 2*P.F_max*P.d)

        # convert force and torque to motor thrusts/forces
        motor_thrusts = P.mixing @ np.array([[F], [tau]])

        return motor_thrusts


def saturate(u, limit):
    if abs(u) > limit:
        u = limit * np.sign(u)
    return u







