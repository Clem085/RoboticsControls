import numpy as np
from typing import Tuple, Optional
import massParam as P


class massDynamics:
    """
    Mass–spring–damper (1D):
      state x = [z; zdot]
      zddot = (u - b*zdot - k*z) / m

    Features:
    - RK4 integrator (default), optional Euler for quick tests
    - Safe scalar/array input handling
    - Parameter setters & reset
    - Energy helpers (kinetic, potential, total)
    """

    def __init__(self,
                 m: Optional[float] = None,
                 b: Optional[float] = None,
                 k: Optional[float] = None,
                 Ts: Optional[float] = None,
                 z0: Optional[float] = None,
                 zdot0: Optional[float] = None,
                 integrator: str = "rk4"):
        # Pull defaults from massParam P if not provided
        self.m = float(P.m if m is None else m)
        self.b = float(P.b if b is None else b)
        self.k = float(P.k if k is None else k)
        self.Ts = float(P.Ts if Ts is None else Ts)

        z_init = float(getattr(P, "z0", 0.0) if z0 is None else z0)
        zdot_init = float(getattr(P, "zdot0", 0.0) if zdot0 is None else zdot0)
        self.state = np.array([[z_init], [zdot_init]], dtype=float)

        if integrator.lower() not in {"rk4", "euler"}:
            raise ValueError("integrator must be 'rk4' or 'euler'")
        self._integrator = integrator.lower()

    # ---------- Core dynamics ----------
    def f(self, x: np.ndarray, u: float | np.ndarray) -> np.ndarray:
        """
        Continuous-time dynamics: xdot = f(x, u)
        x: shape (2,1) or (2,)
        u: scalar or shape (1,1)/(1,)
        """
        x = self._as_col2(x)
        u = float(np.asarray(u).squeeze())
        z, zdot = x[0, 0], x[1, 0]
        zddot = (u - self.b * zdot - self.k * z) / self.m
        return np.array([[zdot], [zddot]], dtype=float)

    def update(self, u: float | np.ndarray) -> np.ndarray:
        """
        Advance one step with input u. Returns new state (2,1).
        """
        if self._integrator == "rk4":
            self.state = self._rk4(self.state, u, self.Ts)
        else:
            self.state = self._euler(self.state, u, self.Ts)
        return self.state

    # ---------- Integrators ----------
    def _rk4(self, x: np.ndarray, u: float | np.ndarray, Ts: float) -> np.ndarray:
        k1 = self.f(x, u)
        k2 = self.f(x + 0.5 * Ts * k1, u)
        k3 = self.f(x + 0.5 * Ts * k2, u)
        k4 = self.f(x + Ts * k3, u)
        return x + (Ts / 6.0) * (k1 + 2 * k2 + 2 * k3 + k4)

    def _euler(self, x: np.ndarray, u: float | np.ndarray, Ts: float) -> np.ndarray:
        return x + Ts * self.f(x, u)

    # ---------- Utilities ----------
    @staticmethod
    def _as_col2(x: np.ndarray) -> np.ndarray:
        x = np.asarray(x, dtype=float)
        if x.shape == (2,):
            x = x.reshape(2, 1)
        if x.shape != (2, 1):
            raise ValueError(f"x must have shape (2,1) or (2,), got {x.shape}")
        return x

    def set_state(self, z: float, zdot: float) -> None:
        self.state = np.array([[float(z)], [float(zdot)]], dtype=float)

    def reset(self) -> None:
        """Reset to initial conditions from massParam P (or current params if you prefer)."""
        z0 = float(getattr(P, "z0", 0.0))
        zdot0 = float(getattr(P, "zdot0", 0.0))
        self.set_state(z0, zdot0)

    def set_params(self, *, m: Optional[float] = None, b: Optional[float] = None,
                   k: Optional[float] = None, Ts: Optional[float] = None) -> None:
        if m is not None: self.m = float(m)
        if b is not None: self.b = float(b)
        if k is not None: self.k = float(k)
        if Ts is not None: self.Ts = float(Ts)

    # ---------- Energetics ----------
    def energies(self) -> Tuple[float, float, float]:
        """
        Returns (T, V, E):
        T = (1/2) m * zdot^2
        V = (1/2) k * z^2
        E = T + V
        """
        z, zdot = float(self.state[0, 0]), float(self.state[1, 0])
        T = 0.5 * self.m * zdot * zdot
        V = 0.5 * self.k * z * z
        return T, V, T + V

    # ---------- Convenience getters ----------
    @property
    def z(self) -> float:
        return float(self.state[0, 0])

    @property
    def zdot(self) -> float:
        return float(self.state[1, 0])
