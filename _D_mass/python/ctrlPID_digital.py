"""
Digital PID controller (measurement-only) for the mass–spring–damper.

Implements position-form PID with a filtered ("dirty") derivative and two
discretizations for the integrator (Backward-Euler and Tustin).

Math reference matches PartIV_Homework_D9a_D10_F9_F10.md:
  C(s) = kP + kI/s + kD * s/(sigma s + 1),  sigma = 0.05

Difference equations (Ts = sample time):
  e[k]   = r[k] - y[k]
  I[k]   = I[k-1] + kI*Ts*e[k]                      # Backward-Euler
         = I[k-1] + kI*Ts/2 * (e[k] + e[k-1])       # Tustin (trapezoidal)
  e_dot_f[k] = (e_dot_f[k-1] + (e[k]-e[k-1])/Ts) / (1 + sigma/Ts)
  u[k]   = kP*e[k] + I[k] + kD*e_dot_f[k]
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

import numpy as np
import massParam as P


@dataclass
class PIDGains:
    kP: float
    kI: float
    kD: float


class CtrlPIDDigital:
    def __init__(
        self,
        gains: PIDGains,
        sigma: float = 0.05,
        Ts: Optional[float] = None,
        integral_method: str = "backward_euler",  # or "tustin"
        use_anti_windup: bool = True,
        u_min: float = -P.F_max,
        u_max: float = P.F_max,
    ) -> None:
        self.g = gains
        self.sigma = float(sigma)
        self.Ts = float(P.Ts if Ts is None else Ts)
        self.integral_method = integral_method.lower()
        self.use_anti_windup = bool(use_anti_windup)
        self.u_min = float(u_min)
        self.u_max = float(u_max)

        # Internal states
        self._e_prev = 0.0
        self._I = 0.0
        self._edot_f_prev = 0.0

    def reset(self) -> None:
        self._e_prev = 0.0
        self._I = 0.0
        self._edot_f_prev = 0.0

    def _integrate(self, e_now: float) -> None:
        if self.integral_method == "tustin":
            self._I += self.g.kI * self.Ts * 0.5 * (e_now + self._e_prev)
        elif self.integral_method == "backward_euler":
            self._I += self.g.kI * self.Ts * e_now
        else:
            raise ValueError("integral_method must be 'backward_euler' or 'tustin'")

    def _dirty_derivative(self, e_now: float) -> float:
        # filtered difference per spec: (prev + delta/Ts)/(1 + sigma/Ts)
        edot_f = (self._edot_f_prev + (e_now - self._e_prev) / self.Ts) / (1.0 + self.sigma / self.Ts)
        self._edot_f_prev = edot_f
        return edot_f

    def update(self, r: float, y: float) -> float:
        """
        Position-form PID update using measurement-only (y) and reference (r).
        Returns a saturated control input u, and applies simple anti-windup by
        clamping the integrator when saturation occurs.
        """
        # Error
        e_now = float(r - y)

        # Integral
        self._integrate(e_now)

        # Derivative (filtered)
        edot_f_now = self._dirty_derivative(e_now)

        # Unsaturated control
        u_unsat = self.g.kP * e_now + self._I + self.g.kD * edot_f_now

        # Saturate
        u_cmd = max(self.u_min, min(self.u_max, u_unsat))

        # Anti-windup (clamp integrator if saturated and sign would drive further)
        if self.use_anti_windup and self.g.kI != 0.0:
            if (u_cmd >= self.u_max and u_unsat > self.u_max) or (
                u_cmd <= self.u_min and u_unsat < self.u_min
            ):
                # Undo most recent integral contribution (freeze)
                if self.integral_method == "tustin":
                    self._I -= self.g.kI * self.Ts * 0.5 * (e_now + self._e_prev)
                else:
                    self._I -= self.g.kI * self.Ts * e_now
                # Recompute with frozen I
                u_unsat = self.g.kP * e_now + self._I + self.g.kD * edot_f_now
                u_cmd = max(self.u_min, min(self.u_max, u_unsat))

        # Update historical error
        self._e_prev = e_now
        return u_cmd

    # Optional: velocity-form for completeness
    def update_velocity_form(self, r: float, y: float, u_prev: float) -> float:
        e_now = float(r - y)
        self._integrate(e_now)
        edot_f_now = self._dirty_derivative(e_now)
        u_now = self.g.kP * (e_now - self._e_prev) + self.g.kI * self.Ts * e_now + self.g.kD * edot_f_now + u_prev
        u_cmd = max(self.u_min, min(self.u_max, u_now))
        self._e_prev = e_now
        return u_cmd

