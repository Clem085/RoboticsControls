import numpy as np
import hummingbirdParam as P


def saturate(u, low_limit, up_limit):
    if isinstance(u, float) or (isinstance(u, np.ndarray) and u.shape == ()):  # scalar
        if u > up_limit:
            return up_limit
        if u < low_limit:
            return low_limit
        return u
    out = np.copy(u)
    for i in range(out.shape[0]):
        if out[i, 0] > up_limit:
            out[i, 0] = up_limit
        if out[i, 0] < low_limit:
            out[i, 0] = low_limit
    return out


class ctrlPID:
    """
    Cascaded PID for the Hummingbird simulation (H.10):
    - Longitudinal (pitch, theta): PID on theta to command total force F about hover.
    - Lateral (yaw, psi): Outer PI on psi to generate roll reference phi_r; inner PD on phi to command torque tau.

    Returns PWM commands in the order [left; right] and the reference vector [phi_r; theta_r; psi_r].
    """

    def __init__(self):
        # sample rate
        self.Ts = P.Ts

        # dirty derivative parameter
        sigma = 0.05
        self.beta = (2.0 * sigma - self.Ts) / (2.0 * sigma + self.Ts)

        # ----------------------------------------
        # Longitudinal (theta) loop gains (PID)
        # Plant approx: theta_ddot = b_theta * F_tilde
        # ----------------------------------------
        b_theta = P.ellT / (P.m1 * P.ell1 ** 2 + P.m2 * P.ell2 ** 2 + P.J1y + P.J2y)
        tr_theta = 0.5
        zeta_theta = 0.8
        wn_theta = 2.2 / tr_theta
        self.kp_theta = (wn_theta ** 2) / b_theta
        self.kd_theta = (2.0 * zeta_theta * wn_theta) / b_theta
        # choose integrator pole slower than wn
        pi_theta = wn_theta / 5.0
        self.ki_theta = (pi_theta * wn_theta ** 2) / b_theta

        # ----------------------------------------
        # Lateral inner (phi) loop gains (PD)
        # Plant approx: phi_ddot = (1/J_phi) * tau  -> use J_phi ≈ J1x
        # ----------------------------------------
        tr_phi = 0.25
        zeta_phi = 0.8
        wn_phi = 2.2 / tr_phi
        b_phi = 1.0 / P.J1x
        self.kp_phi = (wn_phi ** 2) / b_phi
        self.kd_phi = (2.0 * zeta_phi * wn_phi) / b_phi

        # ----------------------------------------
        # Lateral outer (psi) loop gains (PI or PID with small D)
        # Approx plant from phi to psi as 1/s (kinematic) for tuning.
        # ----------------------------------------
        M = 8.0  # time-scale separation outer vs inner
        tr_psi = M * tr_phi
        zeta_psi = 0.9
        wn_psi = 2.2 / tr_psi
        self.kp_psi = 2.0 * zeta_psi * wn_psi  # for plant ~ 1/s
        self.ki_psi = (wn_psi ** 2)            # for plant ~ 1/s
        self.kd_psi = 0.0                      # default no D on outer loop

        # limits
        self.theta_max = 30.0 * np.pi / 180.0
        self.phi_max = 20.0 * np.pi / 180.0
        self.force_max = 2.0 * P.km  # rough bound (per motor <= 1.0 pwm)
        self.torque_max = 2.0 * P.km * P.d

        # delayed/derivative states
        self.theta_d1 = 0.0
        self.theta_dot = 0.0
        self.phi_d1 = 0.0
        self.phi_dot = 0.0
        self.psi_d1 = 0.0
        self.psi_dot = 0.0

        # integrators
        self.int_theta = 0.0
        self.err_theta_d1 = 0.0
        self.int_psi = 0.0
        self.err_psi_d1 = 0.0

        print('ctrlPID gains:')
        print(f'  theta: kp={self.kp_theta:.3f}, ki={self.ki_theta:.3f}, kd={self.kd_theta:.3f}')
        print(f'  phi:   kp={self.kp_phi:.3f}, kd={self.kd_phi:.3f}')
        print(f'  psi:   kp={self.kp_psi:.3f}, ki={self.ki_psi:.3f}, kd={self.kd_psi:.3f}')

    def update(self, r: np.ndarray, y: np.ndarray):
        # references
        theta_ref = float(r[0][0])
        psi_ref = float(r[1][0])

        # measurements
        phi = float(y[0][0])
        theta = float(y[1][0])
        psi = float(y[2][0])

        # derivatives (dirty)
        self.theta_dot = self.beta * self.theta_dot + (1 - self.beta) * ((theta - self.theta_d1) / self.Ts)
        self.phi_dot = self.beta * self.phi_dot + (1 - self.beta) * ((phi - self.phi_d1) / self.Ts)
        self.psi_dot = self.beta * self.psi_dot + (1 - self.beta) * ((psi - self.psi_d1) / self.Ts)

        # ----------------------------------------
        # Outer yaw loop: psi -> phi_ref (PI)
        # ----------------------------------------
        err_psi = psi_ref - psi
        self.int_psi += (self.Ts / 2.0) * (err_psi + self.err_psi_d1)
        phi_ref_unsat = self.kp_psi * err_psi + self.ki_psi * self.int_psi - self.kd_psi * self.psi_dot
        phi_ref = saturate(phi_ref_unsat, -self.phi_max, self.phi_max)
        # anti-windup for psi integrator (back-calculation)
        if self.ki_psi != 0.0:
            self.int_psi += (self.Ts / self.ki_psi) * (phi_ref - phi_ref_unsat)

        # ----------------------------------------
        # Inner roll loop: phi_ref -> tau (PD)
        # ----------------------------------------
        err_phi = phi_ref - phi
        tau_unsat = self.kp_phi * err_phi - self.kd_phi * self.phi_dot
        tau = saturate(tau_unsat, -self.torque_max, self.torque_max)

        # ----------------------------------------
        # Pitch loop: theta_ref -> F (PID)
        # ----------------------------------------
        err_theta = theta_ref - theta
        self.int_theta += (self.Ts / 2.0) * (err_theta + self.err_theta_d1)
        F_tilde_unsat = self.kp_theta * err_theta + self.ki_theta * self.int_theta - self.kd_theta * self.theta_dot
        F_tilde = saturate(F_tilde_unsat, -self.force_max, self.force_max)
        # anti-windup for theta integrator
        if self.ki_theta != 0.0:
            self.int_theta += (self.Ts / self.ki_theta) * (F_tilde - F_tilde_unsat)

        # add equilibrium force about hover
        F = P.Fe + F_tilde

        # convert [F; tau] to motor PWMs
        fl_fr = P.mixing @ np.array([[F], [tau]])
        pwm = fl_fr / P.km
        pwm = saturate(pwm, 0.0, 1.0)

        # update delayed vars
        self.theta_d1 = theta
        self.err_theta_d1 = err_theta
        self.phi_d1 = phi
        self.err_psi_d1 = err_psi
        self.psi_d1 = psi

        # return pwm and reference vector [phi_ref, theta_ref, psi_ref]
        return pwm, np.array([[phi_ref], [theta_ref], [psi_ref]])

