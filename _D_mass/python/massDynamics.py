import numpy as np
from typing import Tuple, Optional
import massParam as P


class massDynamics:
    """
    Mass–spring–damper (1D):
      x = [z, zdot]
      zddot = (u - b*zdot - k*z)/m
    """

    def __init__(self,
                 m: Optional[float] = None,
                 b: Optional[float] = None,
                 k: Optional[float] = None,
                 Ts: Optional[float] = None,
                 z0: Optional[float] = None,
                 zdot0: Optional[float] = None,
                 integrator: str = "rk4"):
        self.m  = float(P.m if m is None else m)
        self.b  = float(P.b if b is None else b)
        self.k  = float(P.k if k is None else k)
        self.Ts = float(P.Ts if Ts is None else Ts)

        z_init    = float(getattr(P, "z0", 0.0) if z0 is None else z0)
        zdot_init = float(getattr(P, "zdot0", 0.0) if zdot0 is None else zdot0)
        self.state = np.array([[z_init], [zdot_init]], dtype=float)

        if integrator.lower() not in {"rk4", "euler"}:
            raise ValueError("integrator must be 'rk4' or 'euler'")
        self._integrator = integrator.lower()

    # ---------- helpers ----------
    @staticmethod
    def _as_col2(x: np.ndarray) -> np.ndarray:
        x = np.asarray(x, dtype=float)
        if x.shape == (2,):
            return x.reshape(2, 1)
        if x.shape == (2, 1):
            return x
        raise ValueError(f"x must have shape (2,) or (2,1); got {x.shape}")

    @staticmethod
    def _match_shape(vec: np.ndarray, like: np.ndarray) -> np.ndarray:
        return vec.flatten() if like.ndim == 1 else vec.reshape(like.shape)

    # ---------- dynamics ----------
    def f(self, x: np.ndarray, u: float | np.ndarray) -> np.ndarray:
        """
        xdot = f(x,u)
        x: (2,) or (2,1) -> [z, zdot]
        u: scalar or shape compatible with scalar
        returns derivative in the SAME shape as `x`
        """
        xcol = self._as_col2(x)
        z, zdot = xcol[:, 0]
        F = float(np.asarray(u).squeeze())

        zddot = (F - self.b * zdot - self.k * z) / self.m
        xdot = np.array([zdot, zddot], dtype=float)
        return self._match_shape(xdot, x)

    def update(self, u: float | np.ndarray) -> np.ndarray:
        if self._integrator == "rk4":
            self.state = self._rk4(self.state, u, self.Ts)
        else:
            self.state = self._euler(self.state, u, self.Ts)
        return self.state

    # ---------- integrators ----------
    def _rk4(self, x: np.ndarray, u: float | np.ndarray, Ts: float) -> np.ndarray:
        k1 = self.f(x, u)
        k2 = self.f(x + 0.5 * Ts * k1, u)
        k3 = self.f(x + 0.5 * Ts * k2, u)
        k4 = self.f(x + Ts * k3, u)
        return x + (Ts / 6.0) * (k1 + 2 * k2 + 2 * k3 + k4)

    def _euler(self, x: np.ndarray, u: float | np.ndarray, Ts: float) -> np.ndarray:
        return x + Ts * self.f(x, u)

    # ---------- utilities ----------
    def set_state(self, z: float, zdot: float) -> None:
        self.state = np.array([[float(z)], [float(zdot)]], dtype=float)

    def reset(self) -> None:
        self.set_state(float(getattr(P, "z0", 0.0)), float(getattr(P, "zdot0", 0.0)))

    def set_params(self, *, m: Optional[float] = None, b: Optional[float] = None,
                   k: Optional[float] = None, Ts: Optional[float] = None) -> None:
        if m  is not None: self.m  = float(m)
        if b  is not None: self.b  = float(b)
        if k  is not None: self.k  = float(k)
        if Ts is not None: self.Ts = float(Ts)

    # ---------- energetics ----------
    def energies(self) -> Tuple[float, float, float]:
        z, zdot = float(self.state[0, 0]), float(self.state[1, 0])
        T = 0.5 * self.m * zdot * zdot
        V = 0.5 * self.k * z * z
        return T, V, T + V

    @property
    def z(self) -> float:
        return float(self.state[0, 0])

    @property
    def zdot(self) -> float:
        return float(self.state[1, 0])
