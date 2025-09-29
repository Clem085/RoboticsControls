import numpy as np
import hummingbirdParam as P

def saturate(u: np.ndarray, low: float, high: float):
    out = np.copy(u)
    for i in range(out.shape[0]):
        out[i,0] = max(min(out[i,0], high), low)
    return out

class HummingbirdDynamics:
    def __init__(self, alpha=0.0):
        # state: [phi, theta, psi, phidot, thetadot, psidot]
        self.state = np.array([
            [P.phi0],
            [P.theta0],
            [P.psi0],
            [P.phidot0],
            [P.thetadot0],
            [P.psidot0],
        ])
        # optional motor variability
        self.km = P.km * (1. + alpha * (2. * np.random.rand() - 1.))

    def h(self):
        # measured outputs (angles)
        phi, theta, psi = self.state[0,0], self.state[1,0], self.state[2,0]
        return np.array([[phi],[theta],[psi]])

    # --- Mass matrix M(q): Chapter 2, Eqs. (2.30)–(2.33) ---
    def _M(self, state):
        phi = state[0,0]
        theta = state[1,0]
        sphi, cphi = np.sin(phi), np.cos(phi)
        sthe, cthe = np.sin(theta), np.cos(theta)

        J1x, J1y, J1z = P.J1x, P.J1y, P.J1z
        J2x, J2y, J2z = P.J2x, P.J2y, P.J2z

        M11 = J1x
        M12 = 0.0
        M13 = -J1x * sthe

        M22 = P.m1*P.ell1**2 + P.m2*P.ell2**2 + J2y + (J1y*(cphi**2) + J1z*(sphi**2))
        M23 = (J1y - J1z) * sphi * cphi * cthe

        T1 = (P.m1*P.ell1**2 + P.m2*P.ell2**2 + J2z + J1y*(sphi**2) + J1z*(cphi**2)) * (cthe**2)
        T2 = (J1x + J2x) * (sthe**2)
        M33 = T1 + T2 + P.m3*(P.ell3x**2 + P.ell3y**2) + P.J3z

        M = np.array([[M11, M12, M13],
                      [M12, M22, M23],
                      [M13, M23, M33]])
        return M

    # --- Coriolis/Centrifugal matrix C(q, qdot) so that C(q,qdot)@qdot = c(q,qdot) ---
    def _C(self, state):
        phi = state[0,0]
        theta = state[1,0]
        sphi, cphi = np.sin(phi), np.cos(phi)
        sthe, cthe = np.sin(theta), np.cos(theta)

        J1x, J1y, J1z = P.J1x, P.J1y, P.J1z
        J2x, J2y, J2z = P.J2x, P.J2y, P.J2z

        # ∂M/∂phi
        dM_dphi = np.zeros((3,3))
        dM_dphi[1,1] = 2.0*(J1z - J1y)*sphi*cphi
        dM_dphi[1,2] = (J1y - J1z)*(cphi**2 - sphi**2)*cthe
        dM_dphi[2,1] = dM_dphi[1,2]
        dM_dphi[2,2] = 2.0*(J1y - J1z)*sphi*cphi*(cthe**2)

        # ∂M/∂theta
        dM_dthe = np.zeros((3,3))
        dM_dthe[0,2] = -J1x * cthe
        dM_dthe[2,0] = dM_dthe[0,2]
        dM_dthe[1,2] = -(J1y - J1z)*sphi*cphi*sthe
        dM_dthe[2,1] = dM_dthe[1,2]
        A = (P.m1*P.ell1**2 + P.m2*P.ell2**2 + J2z + J1y*(sphi**2) + J1z*(cphi**2))
        dM_dthe[2,2] = -2.0*A*cthe*sthe + 2.0*(J1x + J2x)*sthe*cthe

        # ∂M/∂psi = 0
        dM_dpsi = np.zeros((3,3))

        # Coriolis matrix via Christoffel symbols:
        # C_ij = 1/2 * sum_k [ (∂M_ij/∂q_k) + (∂M_ik/∂q_j) - (∂M_jk/∂q_i) ] * qdot_k
        qdot = state[3:6,:].reshape(3)
        dM = [dM_dphi, dM_dthe, dM_dpsi]
        C = np.zeros((3,3))
        for i in range(3):
            for j in range(3):
                s = 0.0
                for k in range(3):
                    s += 0.5 * (dM[k][i,j] + dM[j][i,k] - dM[i][j,k]) * qdot[k]
                C[i,j] = s
        return C

    def _B(self):
        # viscous damping
        return np.diag([P.b_phi, P.b_theta, P.b_psi])

    def _partialP(self, state: np.ndarray):
        # @P/@q from (3.10): [0, (m1*ell1 + m2*ell2) g cos(theta), 0]^T
        theta = state[1,0]
        return np.array([[0.0],
                         [(P.m1*P.ell1 + P.m2*P.ell2) * P.g * np.cos(theta)],
                         [0.0]])

    def _tau(self, state: np.ndarray, force: float, torque: float):
        # Generalized forces vector:
        # tau1 = tau
        # tau2 = ellT*F*cos(theta)
        # tau3 = ellT*F*cos(phi)*sin(theta) - tau*sin(phi)
        phi = state[0,0]
        theta = state[1,0]
        tau1 = torque
        tau2 = P.ellT * force * np.cos(theta)
        tau3 = P.ellT * force * np.cos(phi) * np.sin(theta) - torque * np.sin(phi)
        return np.array([[tau1],[tau2],[tau3]])

    def f(self, state, u_pwm):
        # PWM (0..1) -> rotor forces
        fl = self.km * u_pwm[0,0]
        fr = self.km * u_pwm[1,0]
        # [F; tau] = unmixing @ [fl; fr]
        Ft_tau = P.unmixing @ np.array([[fl],[fr]])
        F = Ft_tau[0,0]
        tau = Ft_tau[1,0]

        q = state[0:3,:]
        qdot = state[3:6,:]
        M = self._M(state)
        C = self._C(state)
        B = self._B()
        dPdq = self._partialP(state)
        Tau = self._tau(state, F, tau)

        # Euler-Lagrange: M qdd + C qdot + dP/dq + B qdot = Tau
        qdd = np.linalg.solve(M, Tau - (C + B) @ qdot - dPdq)
        xdot = np.vstack((qdot, qdd))
        return xdot

    def update(self, u_pwm):
        # 4th-order Runge-Kutta
        x = self.state
        F1 = self.f(x, u_pwm)
        F2 = self.f(x + P.Ts/2.0 * F1, u_pwm)
        F3 = self.f(x + P.Ts/2.0 * F2, u_pwm)
        F4 = self.f(x + P.Ts * F3, u_pwm)
        self.state = self.state + (P.Ts/6.0) * (F1 + 2*F2 + 2*F3 + F4)
        return self.state
