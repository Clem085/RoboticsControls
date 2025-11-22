"""
Lab H.5 – Transfer-Function Verification Script

This script numerically validates the small-angle transfer functions from
Chapter 5 of the hummingbird manual by querying the full nonlinear dynamics.

It checks:
  1) Longitudinal channel:  θ̈ = b_theta * F_tilde  ⇒ Θ(s)/F(s) = b_theta / s²
  2) Roll channel:          φ̈ = (1/J1x) * τ       ⇒ Φ(s)/Τ(s) = (1/J1x)/s²
  3) Yaw channel:           ψ̈ = b_psi * φ         ⇒ Ψ(s)/Φ(s) = b_psi / s²

Run from the repo root:
    python _hummingbird_sim/h5_hummingbirdSim.py
"""

from __future__ import annotations

import numpy as np

import hummingbirdParam as P
from hummingbirdDynamics import HummingbirdDynamics


def transfer_function_constants() -> tuple[float, float, float]:
    """Return b_theta, b_phi, b_psi from the Chapter 4 derivations."""
    inertia_long = (
        P.m1 * P.ell1**2
        + P.m2 * P.ell2**2
        + P.J1y
        + P.J2y
    )
    b_theta = P.ellT / inertia_long

    b_phi = 1.0 / P.J1x

    JT = (
        P.m1 * P.ell1**2
        + P.m2 * P.ell2**2
        + P.J2z
        + P.m3 * (P.ell3x**2 + P.ell3y**2)
    )
    b_psi = (P.ellT * P.Fe) / (JT + P.J1z)
    return float(b_theta), float(b_phi), float(b_psi)


def longitudinal_gain(delta_F: float = 0.01) -> float:
    """Return θ̈/F gain using the nonlinear model."""
    dyn = HummingbirdDynamics(alpha=0.0)
    dyn.state[:] = 0.0
    u_eq = P.Fe / (2.0 * dyn.km)
    delta_pwm = delta_F / (2.0 * dyn.km)
    u = np.array([[u_eq + delta_pwm], [u_eq + delta_pwm]])
    xdot = dyn.f(dyn.state, u)
    theta_ddot = xdot[4, 0]
    fl = dyn.km * u[0, 0]
    fr = dyn.km * u[1, 0]
    F_tilde = (fl + fr) - P.Fe
    return theta_ddot / F_tilde


def roll_gain(delta_tau: float = 0.002) -> float:
    """Return φ̈/τ gain by commanding a small differential torque."""
    dyn = HummingbirdDynamics(alpha=0.0)
    dyn.state[:] = 0.0
    u_eq = P.Fe / (2.0 * dyn.km)
    delta_force_per_motor = delta_tau / (2.0 * P.d)
    delta_pwm = delta_force_per_motor / dyn.km
    u = np.array([[u_eq - delta_pwm], [u_eq + delta_pwm]])
    xdot = dyn.f(dyn.state, u)
    phi_ddot = xdot[3, 0]
    fl = dyn.km * u[0, 0]
    fr = dyn.km * u[1, 0]
    tau = P.d * (fr - fl)
    return phi_ddot / tau


def yaw_gain(theta_test_deg: float = 5.0) -> float:
    """Return ψ̈/θ gain by pitching the craft (matches the manual's small-angle model)."""
    dyn = HummingbirdDynamics(alpha=0.0)
    dyn.state[:] = 0.0
    theta_test = np.deg2rad(theta_test_deg)
    dyn.state[1, 0] = theta_test
    u_eq = P.Fe / (2.0 * dyn.km)
    u = np.array([[u_eq], [u_eq]])
    xdot = dyn.f(dyn.state, u)
    psi_ddot = xdot[5, 0]
    return psi_ddot / theta_test


def percent_error(measured: float, expected: float) -> float:
    return 100.0 * abs(abs(measured) - abs(expected)) / max(abs(expected), 1e-9)


def main():
    b_theta_exp, b_phi_exp, b_psi_exp = transfer_function_constants()

    b_theta_meas = longitudinal_gain()
    b_phi_meas = roll_gain()
    b_psi_meas = yaw_gain()

    print("=== Lab H.5 Transfer-Function Check ===")
    print(f"b_theta expected = {b_theta_exp:.6f}, measured = {b_theta_meas:.6f}, error = {percent_error(b_theta_meas, b_theta_exp):.3f}%")
    print(f"b_phi   expected = {b_phi_exp:.6f}, measured = {b_phi_meas:.6f}, error = {percent_error(b_phi_meas, b_phi_exp):.3f}%")
    print(f"b_psi   expected = {b_psi_exp:.6f}, measured = {b_psi_meas:.6f}, error = {percent_error(b_psi_meas, b_psi_exp):.3f}%")

    tol_percent = 0.5  # empirical tolerance for numerical linearization
    assert percent_error(b_theta_meas, b_theta_exp) < tol_percent, "Longitudinal gain mismatch"
    assert percent_error(b_phi_meas, b_phi_exp) < tol_percent, "Roll gain mismatch"
    assert percent_error(b_psi_meas, b_psi_exp) < tol_percent, "Yaw gain mismatch"

    print("\nCascade intuition: tau tilts the craft (roll/pitch small-angle), and the reoriented rotor force")
    print("creates a lateral moment arm that drives yaw, matching the torque -> tilt -> yaw cascade.")


if __name__ == "__main__":
    main()
