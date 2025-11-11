"""
Digital nested PID for planar VTOL (F.10):
 - Altitude outer loop: PID with filtered derivative
 - Lateral outer loop (z): PID with filtered derivative, outputs theta_cmd
 - Inner attitude loop (theta): PD with filtered derivative

Controller uses only measured outputs (z, h, theta) and references (z_r, h_r).
Implements measurement-only difference equations with sigma=0.05 derivative
filter and selectable integrator discretization (Backward-Euler or Tustin).
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional, Tuple

import numpy as np
import VTOLParam as P


@dataclass
class PIDGains:
    kP: float
    kI: float
    kD: float


@dataclass
class PDGains:
    kP: float
    kD: float


class _PID1D:
    def __init__(self, gains: PIDGains, Ts: float, sigma: float, method: str) -> None:
        self.g = gains
        self.Ts = Ts
        self.sigma = sigma
        self.method = method
        self.e_prev = 0.0
        self.I = 0.0
        self.edot_f_prev = 0.0

    def reset(self) -> None:
        self.e_prev = 0.0
        self.I = 0.0
        self.edot_f_prev = 0.0

    def update(self, r: float, y: float) -> float:
        e_now = float(r - y)
        if self.method == "tustin":
            self.I += self.g.kI * self.Ts * 0.5 * (e_now + self.e_prev)
        else:
            self.I += self.g.kI * self.Ts * e_now
        # dirty derivative
        edot_f = (self.edot_f_prev + (e_now - self.e_prev) / self.Ts) / (1.0 + self.sigma / self.Ts)
        self.edot_f_prev = edot_f
        u = self.g.kP * e_now + self.I + self.g.kD * edot_f
        self.e_prev = e_now
        return u


class _PD1D:
    def __init__(self, gains: PDGains, Ts: float, sigma: float) -> None:
        self.g = gains
        self.Ts = Ts
        self.sigma = sigma
        self.e_prev = 0.0
        self.edot_f_prev = 0.0

    def reset(self) -> None:
        self.e_prev = 0.0
        self.edot_f_prev = 0.0

    def update(self, r: float, y: float) -> float:
        e_now = float(r - y)
        edot_f = (self.edot_f_prev + (e_now - self.e_prev) / self.Ts) / (1.0 + self.sigma / self.Ts)
        self.edot_f_prev = edot_f
        u = self.g.kP * e_now + self.g.kD * edot_f
        self.e_prev = e_now
        return u


class NestedPIDDigital:
    def __init__(
        self,
        gains_h: PIDGains = PIDGains(kP=2.0, kI=0.8, kD=0.4),
        gains_z: PIDGains = PIDGains(kP=2.0, kI=0.8, kD=0.4),
        gains_theta: PDGains = PDGains(kP=8.0, kD=1.5),
        sigma: float = 0.05,
        Ts: Optional[float] = None,
        integral_method: str = "backward_euler",
        motor_limit: float = P.F_max,
    ) -> None:
        self.Ts = float(P.Ts if Ts is None else Ts)
        self.sigma = float(sigma)
        self.integral_method = integral_method.lower()
        self.motor_limit = float(motor_limit)

        # Controllers
        self.altitude = _PID1D(gains_h, self.Ts, self.sigma, self.integral_method)
        self.position = _PID1D(gains_z, self.Ts, self.sigma, self.integral_method)
        self.attitude = _PD1D(gains_theta, self.Ts, self.sigma)

        # Equilibrium force to hover
        self.Fe = (P.mc + 2.0 * P.mr) * P.g

    def reset(self) -> None:
        self.altitude.reset()
        self.position.reset()
        self.attitude.reset()

    def update(self, z_ref: float, h_ref: float, z: float, h: float, theta: float) -> Tuple[float, float]:
        # Outer loops
        # Altitude PID -> total thrust deviation about equilibrium
        F_dev = self.altitude.update(h_ref, h)
        F_cmd = self.Fe + F_dev

        # Lateral position PID -> theta command
        theta_cmd = self.position.update(z_ref, z)

        # Inner attitude PD -> torque command tau
        tau_cmd = self.attitude.update(theta_cmd, theta)

        # Convert (F, tau) to (fr, fl)
        u_FT = np.array([[F_cmd], [tau_cmd]])
        u_LR = P.mixing @ u_FT  # [fr, fl]
        fr = float(np.clip(u_LR[0, 0], 0.0, self.motor_limit))
        fl = float(np.clip(u_LR[1, 0], 0.0, self.motor_limit))
        return fr, fl

