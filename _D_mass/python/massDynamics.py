import numpy as np
import massParam as P

class massDynamics:
    """
    Mass–spring–damper:
      x = [z; zdot]
      zddot = (u - b*zdot - k*z)/m
    """
    def __init__(self):
        z0    = float(getattr(P, "z0", 0.0))
        zdot0 = float(getattr(P, "zdot0", 0.0))
        self.state = np.array([[z0], [zdot0]], dtype=float)

        self.m = float(getattr(P, "m", 5.0))
        self.b = float(getattr(P, "b", 0.5))
        self.k = float(getattr(P, "k", 3.0))
        self.Ts = float(getattr(P, "Ts", 0.01))

    def f(self, x, u):
        z, zdot = x[0,0], x[1,0]
        zddot = (u - self.b*zdot - self.k*z)/self.m
        return np.array([[zdot], [zddot]], dtype=float)

    def update(self, u):
        Ts = self.Ts
        k1 = self.f(self.state, u)
        k2 = self.f(self.state + 0.5*Ts*k1, u)
        k3 = self.f(self.state + 0.5*Ts*k2, u)
        k4 = self.f(self.state + Ts*k3, u)
        self.state = self.state + (Ts/6.0)*(k1 + 2*k2 + 2*k3 + k4)
        return self.state
