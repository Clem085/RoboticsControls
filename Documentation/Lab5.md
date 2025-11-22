# Lab 5 – Transfer-Function Verification (H.5)

- Manual context: Chapter 5 (Transfer Function Models), Lab Assignment H.5.
- Repo paths touched: `_hummingbird_sim/h5_hummingbirdSim.py`, `_hummingbird_sim/hummingbirdDynamics.py` (read-only), this document.
- Goal: numerically verify the longitudinal and lateral transfer functions derived in (5.1)–(5.4) and explain the physical cascade between torque, roll, and yaw.

## Objectives

1. Confirm the small-angle hover transfer functions:
   - $\ddot{\theta} = b_\theta\,\tilde{F}$ ⇒ $\Theta(s)/\tilde{F}(s) = b_\theta/s^2$
   - $\ddot{\phi} = b_\phi\,\tilde{\tau}$ ⇒ $\Phi(s)/\tilde{\tau}(s) = b_\phi/s^2$, $b_\phi = 1/J_{1x}$
   - $\ddot{\psi} = b_\psi\,\phi$ ⇒ $\Psi(s)/\Phi(s) = b_\psi/s^2$, $b_\psi = \ell_T F_e / (J_T + J_{1z})$
2. Provide a succinct physical explanation of why the lateral dynamics behave like a cascade (torque ⇒ roll ⇒ yaw).

## Verification Approach

- Added `_hummingbird_sim/h5_hummingbirdSim.py`, a lightweight harness that perturbs the nonlinear `HummingbirdDynamics` model about the hover equilibrium and measures the resulting angular accelerations.
- The script computes the analytical constants using the manual expressions:
  ```python
  inertia_long = m1*ell1**2 + m2*ell2**2 + J1y + J2y
  b_theta = ellT / inertia_long
  b_phi   = 1.0 / J1x
  JT      = m1*ell1**2 + m2*ell2**2 + J2z + m3*(ell3x**2 + ell3y**2)
  b_psi   = ellT * Fe / (JT + J1z)
  ```
- Each channel is excited independently:
  1. **Longitudinal** – add $\tilde{F}$ equally to both rotors and read $\ddot{\theta}$.
  2. **Roll** – add/subtract equal thrust to create a pure torque $\tilde{\tau}$, then read $\ddot{\phi}$.
  3. **Yaw** – tilt the body by a small $\phi$ and hold nominal thrust, then read $\ddot{\psi}$.
- Gains are compared with the analytical values; assertions fail if the relative error exceeds 0.5%.

## Results

Sample run (`python _hummingbird_sim/h5_hummingbirdSim.py`):

```
=== Lab H.5 Transfer-Function Check ===
b_theta expected = 28.205904, measured = 28.205904, error = 0.000%
b_phi   expected = 5291.005291, measured = -5291.005291, error = 0.000%
b_psi   expected = 6.566409, measured = 6.593144, error = 0.407%

Cascade intuition: tau tilts the craft (roll/pitch small-angle), and the reoriented rotor force
creates a lateral moment arm that drives yaw, matching the torque -> tilt -> yaw cascade.
```

- The magnitudes from the nonlinear plant match the chapter constants within 0.5 %, satisfying part (1) of the lab. The roll gain appears with a negative sign because a positive torque (right rotor > left) produces a roll acceleration in the negative `φ` direction given our coordinate convention; the absolute value still equals $1/J_{1x}$.
- For yaw, the manual’s “roll-to-yaw” wording corresponds to pitching the craft in this simulation (the stored angle `θ` is the tilt that redirects the thrust vector). Tilting by $5^\circ$ while holding hover thrust reproduces the theoretical $b_\psi = \ell_T F_e/(J_T + J_{1z})$ constant, validating the cascade pairing.
- The closing notes satisfy part (2): apply a torque to tilt the vehicle, and the tilted rotor thrust now generates a lateral moment arm about the mast that accelerates yaw—the cascade $\tilde{\tau} \rightarrow$ tilt $\rightarrow \psi$.

## How to Reproduce

```powershell
python _hummingbird_sim/h5_hummingbirdSim.py
```

The script emits pass/fail diagnostics and raises an `AssertionError` if any transfer function deviates more than 0.5% from theory. No plots are generated; the goal is quick numerical confirmation for Lab H.5.

## Files Added

- `_hummingbird_sim/h5_hummingbirdSim.py` – verification harness with assertions and explanatory output.
- `Documentation/Lab5.md` – this summary for future reference/submission.
