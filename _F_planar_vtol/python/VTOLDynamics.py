import numpy as np
import VTOLParam as P

class Dynamics:
    """
    Nonlinear VTOL used elsewhere (kept intact so testDynamics.py still works).
    Inputs: u = [fr, fl]
    State:  x = [z, h, th, zdot, hdot, thdot]
    """
    def __init__(self, alpha: float = 0.0):
        self.state = np.array([[P.z0],[P.h0],[P.theta0],[P.zdot0],[P.hdot0],[P.thetadot0]], dtype=float)
        self.Ts = float(P.Ts)
        urand = lambda: (1 + 2 * alpha * np.random.rand() - alpha)
        self.mc = P.mc * urand()
        self.mr = P.mr * urand()
        self.Jc = P.Jc * urand()
        self.d  = P.d  * urand()
        self.mu = P.mu * urand()
        self.F_wind = getattr(P, "F_wind", 0.0) * urand()

    @staticmethod
    def _as_col(x: np.ndarray, n: int) -> np.ndarray:
        x = np.asarray(x, dtype=float)
        if x.shape == (n,):  return x.reshape(n, 1)
        if x.shape == (n,1): return x
        raise ValueError(f"x must have shape {(n,)} or {(n,1)}, got {x.shape}")

    @staticmethod
    def _match_shape(vec: np.ndarray, like: np.ndarray) -> np.ndarray:
        return vec.flatten() if like.ndim == 1 else vec.reshape(like.shape)

    def f(self, state: np.ndarray, u: np.ndarray) -> np.ndarray:
        x = self._as_col(state, 6)
        uu = self._as_col(u, 2)
        z, h, th, zdot, hdot, thdot = x[:,0]
        fr, fl = uu[:,0]
        mT = self.mc + 2.0*self.mr
        JT = self.Jc + 2.0*self.mr*(self.d**2)
        F   = fr + fl
        tau = self.d*(fr - fl)
        zddot  = (-F*np.sin(th) - self.mu*zdot + self.F_wind)/mT
        hddot  = ( F*np.cos(th) - mT*P.g)/mT
        thddot = tau/JT
        xdot = np.array([zdot, hdot, thdot, zddot, hddot, thddot], float)
        return self._match_shape(xdot, state)

    def h(self) -> np.ndarray:
        return self.state[:3].copy()

    def rk4_step(self, u: np.ndarray) -> None:
        F1 = self.f(self.state, u)
        F2 = self.f(self.state + self.Ts/2*F1, u)
        F3 = self.f(self.state + self.Ts/2*F2, u)
        F4 = self.f(self.state + self.Ts*F3, u)
        self.state = self.state + self.Ts/6*(F1 + 2*F2 + 2*F3 + F4)

    def update(self, u: np.ndarray) -> np.ndarray:
        self.rk4_step(u)
        return self.h()


# =======================
# NEW: Linear VTOL (F.5/F.6)
# =======================
class LinearVTOL:
    """
    Hover-linearized planar VTOL used for F.5/F.6.
      State: x = [ z, h, theta, zdot, hdot, thetadot ]^T
      Input: u = [ F, tau ]^T           (total force, body torque)
      Output: y = [ z, h, theta ]^T     (measured positions/attitude)

    Linearized equations (about theta=0, F_e = m*g):
        z_ddot = -(mu/m)*z_dot - (F_e/m)*theta
        h_ddot =  (1/m)*F
        th_ddot = (1/J)*tau
    """
    def __init__(self):
        # constants for the linear model
        self.m = P.mc + 2.0*P.mr
        self.J = P.Jc + 2.0*P.mr*(P.d**2)
        self.mu = P.mu
        self.Fe = self.m * P.g

        # sim state (6x1) and Ts
        self.state = np.array([[P.z0],[P.h0],[P.theta0],[P.zdot0],[P.hdot0],[P.thetadot0]], float)
        self.Ts = float(P.Ts)

    # ---------- F.6: state-space ----------
    def A(self) -> np.ndarray:
        m, mu, Fe = self.m, self.mu, self.Fe
        return np.array([
            [0, 0, 0,      1,       0, 0],
            [0, 0, 0,      0,       1, 0],
            [0, 0, 0,      0,       0, 1],
            [0, 0, -Fe/m, -mu/m,    0, 0],
            [0, 0, 0,      0,       0, 0],
            [0, 0, 0,      0,       0, 0],
        ], float)

    def B(self) -> np.ndarray:
        m, J = self.m, self.J
        return np.array([
            [0,   0],
            [0,   0],
            [0,   0],
            [0,   0],
            [1/m, 0],
            [0, 1/J],
        ], float)

    def C(self) -> np.ndarray:
        return np.array([
            [1,0,0,0,0,0],
            [0,1,0,0,0,0],
            [0,0,1,0,0,0],
        ], float)

    def D(self) -> np.ndarray:
        return np.zeros((3,2), float)

    def ss(self):
        return self.A(), self.B(), self.C(), self.D()

    # ---------- F.5: transfer functions ----------
    def tf_vertical(self):
        # H/F = 1/(m s^2)
        return np.array([1.0]), np.array([self.m, 0.0, 0.0])

    def tf_pitch(self):
        # Theta/Tau = 1/(J s^2)
        return np.array([1.0]), np.array([self.J, 0.0, 0.0])

    def tf_lateral_theta2z(self):
        # Z/Theta = -(Fe/m) / (s^2 + (mu/m)s)
        return np.array([-self.Fe/self.m]), np.array([1.0, self.mu/self.m, 0.0])

    def tf_lateral_tau2z(self):
        # cascade: (Z/Theta) * (Theta/Tau)
        num1, den1 = self.tf_lateral_theta2z()
        num2, den2 = self.tf_pitch()
        # simple convolution for series TFs
        num = np.convolve(num1, num2)
        den = np.convolve(den1, den2)
        return num, den

    # ---------- simulation ----------
    @staticmethod
    def _as_col(x: np.ndarray, n: int) -> np.ndarray:
        x = np.asarray(x, dtype=float)
        if x.shape == (n,):  return x.reshape(n, 1)
        if x.shape == (n,1): return x
        raise ValueError(f"x must have shape {(n,)} or {(n,1)}, got {x.shape}")

    @staticmethod
    def _match(vec: np.ndarray, like: np.ndarray) -> np.ndarray:
        return vec.flatten() if like.ndim == 1 else vec.reshape(like.shape)

    def f(self, state: np.ndarray, u: np.ndarray) -> np.ndarray:
        x = self._as_col(state, 6)
        uu = self._as_col(u, 2)  # [F, tau]
        A, B = self.A(), self.B()
        xdot = A @ x + B @ uu
        return self._match(xdot, state)

    def rk4_step(self, u: np.ndarray) -> None:
        F1 = self.f(self.state, u)
        F2 = self.f(self.state + self.Ts/2*F1, u)
        F3 = self.f(self.state + self.Ts/2*F2, u)
        F4 = self.f(self.state + self.Ts*F3, u)
        self.state = self.state + self.Ts/6*(F1 + 2*F2 + 2*F3 + F4)

    def update(self, u: np.ndarray) -> np.ndarray:
        """Advance one step with u=[F, tau]. Return y=[z,h,theta]."""
        self.rk4_step(u)
        return (self.C() @ self.state)  # 3x1
