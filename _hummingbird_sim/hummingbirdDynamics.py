# hummingbirdDynamics.py
import numpy as np
import hummingbirdParam as P

def _saturate(u, lo, hi):
    if isinstance(u, float) or np.isscalar(u):
        return max(min(u, hi), lo)
    u = np.array(u, dtype=float).copy()
    for i in range(u.shape[0]):
        u[i][0] = max(min(u[i][0], hi), lo)
    return u

class HummingbirdDynamics:
    """
    State x = [phi, theta, psi, phidot, thetadot, psidot]^T  (6x1 column)
    Input u = [[pwm_left],[pwm_right]] in [0,1]
    """
    def __init__(self, alpha: float = 0.0, integrator: str = "rk4"):
        self.alpha = float(alpha)
        self.integrator = integrator
        # initialize state from parameter file
        self.state = np.array([[P.phi0],
                               [P.theta0],
                               [P.psi0],
                               [P.phidot0],
                               [P.thetadot0],
                               [P.psidot0]], dtype=float)
        # small viscous damping on generalized rates
        self._B = 1.0e-3 * np.eye(3)
        self._Ts = float(P.Ts)

    # public API expected by your sim
    def update(self, u_pwm: np.ndarray):
        u_pwm = _saturate(u_pwm, 0.0, 1.0)  # clamp PWM to [0,1]
        if self.integrator == "rk4":
            self.state = self._rk4_step(self.state, u_pwm, self._Ts)
        else:
            self.state = self._euler_step(self.state, u_pwm, self._Ts)
        # return measured outputs (angles)
        y = self.state[0:3].copy()
        return y

    # ---------- integrators ----------
    def _euler_step(self, x, u, Ts):
        return x + Ts * self._f(x, u)

    def _rk4_step(self, x, u, Ts):
        k1 = self._f(x, u)
        k2 = self._f(x + 0.5*Ts*k1, u)
        k3 = self._f(x + 0.5*Ts*k2, u)
        k4 = self._f(x + Ts*k3, u)
        return x + (Ts/6.0)*(k1 + 2*k2 + 2*k3 + k4)

    # ---------- continuous dynamics ----------
    def _f(self, x, u_pwm):
        # unpack state
        phi, theta, psi, phid, thetad, psid = [x[i,0] for i in range(6)]
        q    = np.array([phi, theta, psi])
        qdot = np.array([phid, thetad, psid])

        # PWM -> total force F and roll torque tau_phi
        uL = float(u_pwm[0,0]); uR = float(u_pwm[1,0])
        # From your controller scaffold: force = km*(uL+uR), torque = km*d*(uL-uR)
        F_total = P.km * (uL + uR)
        tau_phi = P.km * P.d * (uL - uR)

        # generalized input τ(q,u)
        tau = self._tau_gen(q, F_total, tau_phi)

        # dynamics pieces
        M = self._M(q)
        C = self._C(q, qdot)
        dP_dq = self._dP_dq(q)

        # qddot
        qddot = np.linalg.solve(M, tau - self._B @ qdot - C - dP_dq)

        xdot = np.array([[phid],
                         [thetad],
                         [psid],
                         [qddot[0]],
                         [qddot[1]],
                         [qddot[2]]], dtype=float)
        return xdot

    # ---------- model pieces (Chapter 3) ----------
    def _M(self, q):
        phi, theta, _ = q
        c = np.cos; s = np.sin
        cphi, sphi = c(phi), s(phi)
        cth,  sth  = c(theta), s(theta)

        M22 = (P.m1*P.ell1**2 + P.m2*P.ell2**2 + P.J2y
               + P.J1y*cphi**2 + P.J1z*sphi**2)
        M23 = (P.J1y - P.J1z)*sphi*cphi*cth
        M33 = ((P.m1*P.ell1**2 + P.m2*P.ell2**2 + P.J2z
                + P.J1y*sphi**2 + P.J1z*cphi**2)*(cth**2)
               + (P.J1x + P.J2x)*(sth**2)
               + P.m3*(P.ell3x**2 + P.ell3y**2) + P.J3z)

        M = np.array([[P.J1x,          0.0,        -P.J1x*sth],
                      [0.0,            M22,         M23       ],
                      [-P.J1x*sth,     M23,         M33       ]], dtype=float)
        return M

    def _C(self, q, qdot):
        phi, theta, _      = q
        phid, thetad, psid = qdot
        c = np.cos; s = np.sin
        cphi, sphi = c(phi), s(phi)
        cth,  sth  = c(theta), s(theta)
        c2phi, s2phi = cphi**2, sphi**2

        A = (P.J1y - P.J1z)

        # Row-grouped compact form matching the manual’s structure
        term1 = A*sphi*cphi*(thetad**2 - (cth**2)*(psid**2)) \
                + (A*(c2phi - s2phi) - P.J1x)*cth*thetad*psid

        term2 = -2*(P.J1z - P.J1y)*sphi*cphi*phid*thetad \
                + (A*(c2phi - s2phi) + P.J1x)*cth*phid*psid \
                - ( -P.m1*P.ell1**2 - P.m2*P.ell2**2 - P.J2z + P.J1x + P.J2x
                    - P.J1y*s2phi - P.J1z*c2phi )*2*sth*cth*(psid**2)/1.0

        term3 = (P.J1z - P.J1y)*sphi*cphi*sth*(thetad**2) \
                + (A*(c2phi - s2phi) - P.J1x)*cth*phid*thetad \
                + 2*A*sphi*cphi*phid*psid \
                + 2*( -P.m1*P.ell1**2 - P.m2*P.ell2**2 - P.J2z + P.J1x + P.J2x
                      + P.J1y*s2phi + P.J1z*s2phi )*sth*cth*thetad*psid

        return np.array([term1, term2, term3], dtype=float)

    def _dP_dq(self, q):
        _, theta, _ = q
        return np.array([
            0.0,
            (P.m1*P.ell1 + P.m2*P.ell2) * P.g * np.sin(theta),
            0.0
        ], dtype=float)


    def _tau_gen(self, q, F_total, tau_phi):
        phi, theta, _ = q
        c = np.cos; s = np.sin
        # roll input from differential thrust; other entries from geometry
        tau_theta = P.ellT * F_total * c(phi)
        tau_psi   = P.ellT * F_total * c(theta)*s(phi) - tau_phi * s(theta)
        return np.array([tau_phi, tau_theta, tau_psi], dtype=float)
