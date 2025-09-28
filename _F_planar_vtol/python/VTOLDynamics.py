import numpy as np
import VTOLParam as P


class Dynamics:
    def __init__(self, alpha: float = 0.0):
        # initial state as a column vector (6x1)
        self.state = np.array([
            [P.z0],       # lateral position
            [P.h0],       # altitude
            [P.theta0],   # roll angle
            [P.zdot0],    # lateral velocity
            [P.hdot0],    # climb rate
            [P.thetadot0] # angular velocity
        ], dtype=float)

        # sample time available to RK4
        self.Ts = float(P.Ts)

        # randomized physical params (robustness modeling)
        urand = lambda: (1 + 2 * alpha * np.random.rand() - alpha)
        self.mc = P.mc * urand()
        self.mr = P.mr * urand()
        self.Jc = P.Jc * urand()
        self.d  = P.d  * urand()
        self.mu = P.mu * urand()
        # allow wind force but keep zero if your P has 0
        self.F_wind = getattr(P, "F_wind", 0.0) * urand()

    # ---------- helpers ----------
    @staticmethod
    def _as_col(x: np.ndarray, n: int) -> np.ndarray:
        x = np.asarray(x, dtype=float)
        if x.shape == (n,):
            return x.reshape(n, 1)
        if x.shape == (n, 1):
            return x
        raise ValueError(f"x must have shape {(n,)} or {(n,1)}, got {x.shape}")

    @staticmethod
    def _match_shape(vec: np.ndarray, like: np.ndarray) -> np.ndarray:
        """Return vec with the same 1D/2D (column) shape as like."""
        return vec.flatten() if like.ndim == 1 else vec.reshape(like.shape)

    # ---------- dynamics ----------
    def f(self, state: np.ndarray, u: np.ndarray) -> np.ndarray:
        """
        xdot = f(x,u) for planar VTOL
        state: (6,) or (6,1) -> [z, h, th, zdot, hdot, thdot]
        u: (2,) or (2,1) -> [fr, fl]
        returns derivative in the SAME shape as `state`
        """
        x = self._as_col(state, 6)
        uu = self._as_col(u, 2)

        z, h, th, zdot, hdot, thdot = x[:, 0]
        fr, fl = uu[:, 0]

        # total mass/inertia (textbook F.3)
        mT = self.mc + 2.0 * self.mr
        JT = self.Jc + 2.0 * self.mr * (self.d ** 2)

        F = fr + fl
        tau = self.d * (fr - fl)

        zddot  = ( -F * np.sin(th) - self.mu * zdot + self.F_wind ) / mT
        hddot  = (  F * np.cos(th) - mT * P.g ) / mT
        thddot =   tau / JT

        xdot = np.array([zdot, hdot, thdot, zddot, hddot, thddot], dtype=float)
        return self._match_shape(xdot, state)

    def h(self) -> np.ndarray:
        """measurement model y = [z, h, theta]^T"""
        z = float(self.state[0, 0])
        h = float(self.state[1, 0])
        theta = float(self.state[2, 0])
        return np.array([[z], [h], [theta]], dtype=float)

    def rk4_step(self, u: np.ndarray) -> None:
        """one RK4 step on internal state"""
        F1 = self.f(self.state, u)
        F2 = self.f(self.state + self.Ts / 2 * F1, u)
        F3 = self.f(self.state + self.Ts / 2 * F2, u)
        F4 = self.f(self.state + self.Ts * F3, u)
        self.state = self.state + self.Ts / 6 * (F1 + 2 * F2 + 2 * F3 + F4)

    def update(self, u: np.ndarray) -> np.ndarray:
        """external API: advance state and return measurement"""
        self.rk4_step(u)
        return self.h()
