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

    # --- Core dynamics in Euler-Lagrange form (simplified consistent model) ---
    def _M(self, state):
        # diagonal inertia
        return np.diag([P.Jphi, P.Jtheta, P.Jpsi])

    def _C(self, state):
        # keep C small/simple (zero) for simulation stability in this lab
        return np.zeros((3,3))

    def _B(self):
        return np.diag([P.b_phi, P.b_theta, P.b_psi])

    def _partialP(self, state: np.ndarray):
        theta = state[1,0]
        dPdphi = 0.0
        dPdtheta = (P.m1*P.ell1 + P.m2*P.ell2) * P.g * np.cos(theta)
        dPdpsi = 0.0
        return np.array([[dPdphi],[dPdtheta],[dPdpsi]])

    def _tau(self, state: np.ndarray, force: float, torque: float):
        phi = state[0,0]
        theta = state[1,0]
        tau1 = torque                                  # roll generalized force
        tau2 = P.ellT * force * np.cos(theta)          # pitch generalized force
        tau3 = P.ellT * force * np.cos(phi) * np.sin(theta) - torque * np.sin(phi)  # yaw
        return np.array([[tau1],[tau2],[tau3]])

    def f(self, state, u_pwm):
        # convert PWM (0..1) to rotor forces
        fl = self.km * u_pwm[0,0]
        fr = self.km * u_pwm[1,0]

        # map to total force F and roll torque tau: [F; tau] = unmixing @ [fl; fr]
        Ft_tau = P.unmixing @ np.array([[fl],[fr]])  # [F; tau]
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
