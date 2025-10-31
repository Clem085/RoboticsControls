import numpy as np
import hummingbirdParam as P


class CtrlPD:
    """
    Combined PD controller:
      - Longitudinal (theta) PD producing F_cmd
      - Inner roll (phi) PD producing tau_cmd
      - Outer yaw (psi) PD producing phi_d

    update(state, ref, Ts) -> (F_cmd, tau_cmd)
      state: dict or tuple with (theta, phi, psi) in radians
      ref:   dict {'theta_d':..., 'phi_d':..., 'psi_d':...}

    Dirty derivatives are used for theta, phi, psi.
    """

    def __init__(self, params=P,
                 zeta_theta: float = 0.707, tr_theta: float = 0.6,
                 zeta_phi: float = 0.9,   tr_psi: float = 1.5,
                 M_bw: float = 15.0,
                 zeta_psi: float = 0.9,
                 tau_d: float = 0.05):
        self.P = params

        # Longitudinal PD (reuse formulas)
        denom_theta = (
            self.P.m1 * self.P.ell1**2
            + self.P.m2 * self.P.ell2**2
            + self.P.J1y
            + self.P.J2y
        )
        self.b_theta = self.P.ellT / denom_theta
        self.Fe = self.P.Fe
        self.wn_theta = 2.2 / tr_theta
        self.kp_theta = (self.wn_theta**2) / self.b_theta
        self.kd_theta = (2.0 * zeta_theta * self.wn_theta) / self.b_theta

        # Inner roll plant: phi_ddot = (1/J1x) * tau
        self.J1x = self.P.J1x
        self.wn_psi = 2.2 / tr_psi
        self.wn_phi = M_bw * self.wn_psi
        self.kp_phi = (self.wn_phi**2) * self.J1x
        self.kd_phi = (2.0 * zeta_phi * self.wn_phi) * self.J1x

        # Outer yaw plant: psi_ddot = b_psi * phi
        # Linearized about small angles at theta=0, phi=0
        Jzz_equiv = (
            self.P.m1 * self.P.ell1**2
            + self.P.m2 * self.P.ell2**2
            + self.P.J2z
            + self.P.J1z
            + self.P.m3 * (self.P.ell3x**2 + self.P.ell3y**2)
            + self.P.J3z
        )
        self.b_psi = (self.P.ellT * self.Fe) / Jzz_equiv
        self.kp_psi = (self.wn_psi**2) / self.b_psi
        self.kd_psi = (2.0 * zeta_psi * self.wn_psi) / self.b_psi

        # Dirty derivative filters
        self.tau_d = tau_d
        self.theta_prev = 0.0
        self.dtheta_hat = 0.0
        self.phi_prev = 0.0
        self.dphi_hat = 0.0
        self.psi_prev = 0.0
        self.dpsi_hat = 0.0

        # Expose last commanded phi_d for plotting
        self.last_phi_d = 0.0

    def reset(self):
        self.theta_prev = 0.0
        self.dtheta_hat = 0.0
        self.phi_prev = 0.0
        self.dphi_hat = 0.0
        self.psi_prev = 0.0
        self.dpsi_hat = 0.0
        self.last_phi_d = 0.0

    def _dirty_diff(self, x: float, x_prev: float, xdot_hat: float, Ts: float):
        alpha = self.tau_d / (self.tau_d + Ts)
        xdot_hat = alpha * xdot_hat + (1.0 - alpha) * (x - x_prev) / Ts
        return xdot_hat

    def update(self, state, ref, Ts):
        # Unpack state (support dict or tuple-like)
        if isinstance(state, dict):
            theta = state.get('theta', 0.0)
            phi = state.get('phi', 0.0)
            psi = state.get('psi', 0.0)
        else:
            # assume (theta, phi, psi) or (phi, theta, psi)
            try:
                # prefer named-like access order from hummingbirdDynamics.h()
                phi = float(state[0])
                theta = float(state[1])
                psi = float(state[2])
            except Exception:
                theta, phi, psi = state

        theta_d = ref.get('theta_d', 0.0)
        psi_d = ref.get('psi_d', 0.0)
        # phi_d may be provided directly (for testing), otherwise from yaw loop
        phi_d_ref = ref.get('phi_d', None)

        # Dirty derivatives
        self.dtheta_hat = self._dirty_diff(theta, self.theta_prev, self.dtheta_hat, Ts)
        self.theta_prev = theta
        self.dphi_hat = self._dirty_diff(phi, self.phi_prev, self.dphi_hat, Ts)
        self.phi_prev = phi
        self.dpsi_hat = self._dirty_diff(psi, self.psi_prev, self.dpsi_hat, Ts)
        self.psi_prev = psi

        # Longitudinal PD -> F_cmd
        e_theta = theta_d - theta
        F_tilde = self.kp_theta * e_theta + self.kd_theta * (0.0 - self.dtheta_hat)
        F_cmd = self.Fe + F_tilde

        # Outer yaw PD -> phi_d
        if phi_d_ref is None:
            e_psi = psi_d - psi
            phi_d = self.kp_psi * e_psi + self.kd_psi * (0.0 - self.dpsi_hat)
        else:
            phi_d = float(phi_d_ref)
        self.last_phi_d = phi_d

        # Inner roll PD -> tau_cmd
        e_phi = phi_d - phi
        tau_cmd = self.kp_phi * e_phi + self.kd_phi * (0.0 - self.dphi_hat)

        return F_cmd, tau_cmd
