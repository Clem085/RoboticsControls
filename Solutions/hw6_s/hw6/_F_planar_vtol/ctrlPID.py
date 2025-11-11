import numpy as np
import VTOLParam as P


class ctrlPID:
    def __init__(self):
        # dirty derivative parameters
        self.sigma = 0.05  # cutoff freq for dirty derivative

        #--------------------------------------------------
        # defining parameters for damping behavior in different loops
        #--------------------------------------------------
        zeta_h = 0.95  # damping ratio for altitude - originally 0.707 in F.8
        zeta_z = 0.95  # damping ratio for outer lateral loop
        zeta_th = 0.95  # damping ratio for inner lateral loop

        # saturation limits
        self.theta_max = 10.0 * np.pi / 180.0  # Max theta, rads
        self.Fe = (P.mc + 2.0 * P.mr) * P.g  # equilibrium force        

        #---------------------------------------------------
        # tuning parameters for longitudinal dynamics 
        #---------------------------------------------------
        # Finding PD gains for longitudinal (altitude) control
        tr_h = 3.0   # rise time for altitude - tuned for best rise time without saturation
        wn_h = 0.5*np.pi/(tr_h*np.sqrt(1-zeta_h**2)) # natural frequency
        Delta_cl_d = [1, 2*zeta_h*wn_h, wn_h**2.0]  # desired closed loop char eq
        self.kp_h = Delta_cl_d[2]*(P.mc+2.0*P.mr)  # kp - altitude
        self.kd_h = Delta_cl_d[1]*(P.mc+2.0*P.mr)  # kd = altitude
        self.ki_h = 1.0  # integrator on altitude

        #---------------------------------------------------        
        # tuning parameters for longitudinal dynamics 
        #---------------------------------------------------
        # Finding PD gains for lateral inner loop
        tr_th = 0.5
        b0 = 1.0/(P.Jc+2.0*P.mr*P.d**2)
        wn_th    = 0.5*np.pi/(tr_th*np.sqrt(1-zeta_th**2))
        self.kp_th = wn_th**2.0/b0
        self.kd_th = 2.0*zeta_th*wn_th/b0

        #---------------------------------------------------
        # Finding PD gain for lateral outer loop
        M  = 10.0  # time separation between inner and outer lateral loops
        tr_z = tr_th*M  # rise time for outer lateral loop (position) - tuned for best rise time without saturation        
        b1 = -self.Fe/(P.mc+2.0*P.mr)
        a1 = P.mu/(P.mc+2.0*P.mr)
        wn_z = 0.5*np.pi/(tr_z*np.sqrt(1-zeta_z**2))
        self.kp_z = wn_z**2.0/b1
        self.kd_z = (2.0*zeta_z*wn_z-a1)/b1

        # for ki_z, it actually works best if ki_z = 0.0!! This tells us something
        # about the system type. But it is possible to get reasonable performance
        # if we increase zeta for theta and z as shown below. The negative is
        # strange, but if we examine kp_z and kd_z, they are also negative. This
        # is because the plant itself has a negative gain. Making kz_i positive
        # actually drives the system to be unstable.
        # self.ki_z = -0.02  # integrator on position
        self.ki_z = 0.0 # making this zero now for simplicity and better performance

        # print control gains to terminal        
        print('kp_z: ', self.kp_z)
        print('kd_z: ', self.kd_z)
        print('ki_z: ', self.ki_z)
        print('kp_h: ', self.kp_h)
        print('kd_h: ', self.kd_h)
        print('kp_th: ', self.kp_th)
        print('kd_th: ', self.kd_th)
        

        #---------------------------------------------------
        # initialize variables needed for integrator and differentiators
        #---------------------------------------------------
        self.integrator_h = 0.
        self.error_h_prev = 0.
        self.h_dot = P.hdot0
        self.h_prev = P.h0
        self.integrator_z = 0.
        self.error_z_prev = 0.
        self.z_dot = P.zdot0
        self.z_prev = P.z0
        self.theta_dot = P.thetadot0
        self.theta_prev = P.theta0

    def update(self, r, y):
        z_r = r[0][0]
        h_r = r[1][0]
        z = y[0][0]
        h = y[1][0]
        theta = y[2][0]

        #---------------------------------------------------
        # Update altitude control
        #---------------------------------------------------
        # Compute the error in h
        error_h = h_r - h
        
        # differentiate h
        self.h_dot = (2 * self.sigma - P.Ts) / (2 * self.sigma + P.Ts) * self.h_dot + \
                        (2.0 / (2.0*self.sigma + P.Ts)) * (h - self.h_prev)

        # integrate error in h if h_dot is small. This is to prevent integrator windup.
        if np.abs(self.h_dot) < 0.5:
            self.integrator_h = self.integrator_h + (P.Ts / 2) * (error_h + self.error_h_prev)
        
        # integrator anti - windup - alternative method
        #if self.ki_h != 0.0:
        #    self.integrator_h = self.integrator_h + P.Ts / self.ki_h * (F - F_unsat)

        # PID control - unsaturated
        F_tilde = self.kp_h * (h_r - h) + self.ki_h * self.integrator_h - self.kd_h * self.h_dot
        F_unsat = F_tilde + self.Fe
        F = saturate( F_unsat, 2*P.F_max)
          
        #---------------------------------------------------
        # Update position control
        #---------------------------------------------------
        # Compute the error in z
        error_z = z_r - z
        
        # differentiate z
        self.z_dot = (2 * self.sigma - P.Ts) / (2 * self.sigma + P.Ts) * self.z_dot + \
                        (2.0 / (2.0*self.sigma + P.Ts))  * (z - self.z_prev)
        
        # integrate error in z if z_dot is small enough. This is to prevent integrator windup.
        if np.abs(self.z_dot) < 0.5:
            self.integrator_z = self.integrator_z + (P.Ts / 2) * (error_z + self.error_z_prev)

        # PID control - unsaturated
        theta_r = self.kp_z * error_z + self.ki_z * self.integrator_z - self.kd_z * self.z_dot
        
        #---------------------------------------------------
        # Update pitch control
        #---------------------------------------------------
        # differentiate theta
        self.theta_dot = (2 * self.sigma - P.Ts) / (2 * self.sigma + P.Ts) * self.theta_dot + \
                                (2.0 / (2.0*self.sigma + P.Ts))  * (theta - self.theta_prev)

         # PD control on theta
        tau = saturate( self.kp_th * (theta_r - theta) - self.kd_th * self.theta_dot, 2*P.F_max*P.d)

        # update delayed variables
        self.error_h_prev = error_h
        self.h_prev = h
        self.error_z_prev = error_z
        self.z_prev = z
        self.theta_prev = theta
        motor_thrusts = P.mixing @ np.array([[F], [tau]])

        return motor_thrusts
 

def saturate(u, limit):
    if abs(u) > limit:
        u = limit * np.sign(u)
    return u

 
