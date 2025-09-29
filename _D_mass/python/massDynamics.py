import numpy as np
from typing import Tuple, Optional
import massParam as P

class massDynamics:
    """
    Mass–spring–damper (1D):
      x = [z, zdot]^T
      zddot = (F - b*zdot - k*z)/m
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

    # ---------- D.6: state-space matrices ----------
    def A(self) -> np.ndarray:
        m, b, k = self.m, self.b, self.k
        return np.array([[0.0,      1.0],
                         [-k/m,  -b/m]])

    def B(self) -> np.ndarray:
        m = self.m
        return np.array([[0.0],
                         [1.0/m]])

    def C(self) -> np.ndarray:
        # measured output y = z
        return np.array([[1.0, 0.0]])

    def D(self) -> np.ndarray:
        return np.array([[0.0]])

    def ss(self) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        """Convenience: return (A,B,C,D)."""
        return self.A(), self.B(), self.C(), self.D()

    # ---------- D.5: transfer function Z(s)/F(s) ----------
    def tf(self) -> Tuple[np.ndarray, np.ndarray]:
        """
        Return TF numerator/denominator arrays for G_{zF}(s) = Z(s)/F(s)
        = 1 / (m s^2 + b s + k).
        """
        m, b, k = self.m, self.b, self.k
        num = np.array([1.0])          # numerator = 1
        den = np.array([m, b, k], float)
        return num, den

    # ---------- dynamics ----------
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

    def f(self, x: np.ndarray, u: float | np.ndarray) -> np.ndarray:
        """
        xdot = f(x,u)
        x: [z, zdot], shape (2,) or (2,1)
        u: scalar force F
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

    # output equation y = Cx + Du
    def output(self, u: float | np.ndarray = 0.0) -> float:
        C, D = self.C(), self.D()
        x = self.state
        F = float(np.asarray(u).squeeze())
        y = (C @ x + D * F).item()
        return y

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
