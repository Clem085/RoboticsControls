---
title: Hummingbird Lab 3 - PD Control (Simulation)
date: 2025-10-31
header-includes:
  - \usepackage{amsmath}
  - \usepackage{amssymb}
  - \usepackage{siunitx}
---

Overview

- This note summarizes the PD designs used in Lab 3 for the hummingbird simulation and records the specific formulas used to compute gains from time-domain specs. All math is LaTeX to render correctly when exported to PDF (tested with pandoc and typical VS Code PDF exporters that support MathJax/KaTeX).

Models

- Longitudinal (about $\theta_e=0$):

  \[
  \begin{aligned}
  \tilde\theta''(t) &= b_\theta\,\tilde F(t),\\
  b_\theta &= \dfrac{\ell_T}{m_1\ell_1^2 + m_2\ell_2^2 + J_{1y} + J_{2y}},\\
  F_e &= \dfrac{(m_1\ell_1 + m_2\ell_2)\,g}{\ell_T}.
  \end{aligned}
  \]

- Lateral (successive loop closure, inner roll faster than outer yaw):

  \[
  \begin{aligned}
  \phi''(t) &= \frac{1}{J_{1x}}\,\tau(t),\\
  \psi''(t) &\approx b_\psi\,\phi(t),\quad b_\psi \approx \frac{\ell_T F_e}{J_{zz,\text{eq}}},\\
  J_{zz,\text{eq}} &= m_1\ell_1^2 + m_2\ell_2^2 + J_{2z} + J_{1z} + m_3(\ell_{3x}^2 + \ell_{3y}^2) + J_{3z}.
  \end{aligned}
  \]

Dirty Derivative (First-Order Filtered Difference)

For any signal $x$:

\[
\begin{aligned}
\hat{\dot x}[k] &= \alpha\,\hat{\dot x}[k-1] + (1-\alpha)\,\frac{x[k]-x[k-1]}{T_s},\\
\alpha &= \frac{\tau_d}{\tau_d + T_s}.
\end{aligned}
\]

Gain Mapping from Specs

With desired $\zeta$ and rise time $t_r$:

\[
\omega_n \approx \frac{2.2}{t_r}.
\]

- Longitudinal (pitch):

  \[
  k_{P_\theta} = \frac{\omega_{n\theta}^2}{b_\theta},\quad
  k_{D_\theta} = \frac{2\zeta_\theta\,\omega_{n\theta}}{b_\theta}.
  \]

- Inner roll:

  \[
  k_{P_\phi} = \omega_{n\phi}^2 J_{1x},\quad
  k_{D_\phi} = 2\zeta_\phi\,\omega_{n\phi} J_{1x}.
  \]

- Outer yaw:

  \[
  k_{P_\psi} = \frac{\omega_{n\psi}^2}{b_\psi},\quad
  k_{D_\psi} = \frac{2\zeta_\psi\,\omega_{n\psi}}{b_\psi}.
  \]

Bandwidth Separation

Choose inner-loop roll at least one order faster than yaw:

\[
\omega_{n\phi} = M\,\omega_{n\psi},\quad M \in [10,20].
\]

Control Laws (PD Only)

- Longitudinal PD (force):

  \[
  \begin{aligned}
  e_\theta &= \theta_d - \theta,\\
  \widehat{\dot e}_\theta &= -\,\widehat{\dot\theta},\\
  \tilde F &= k_{P_\theta} e_\theta + k_{D_\theta}\,\widehat{\dot e}_\theta,\\
  F &= F_e + \tilde F.
  \end{aligned}
  \]

- Lateral PD (successive loops):

  Outer yaw (commands roll reference):

  \[
  e_\psi = \psi_d - \psi,\quad \widehat{\dot e}_\psi = -\,\widehat{\dot\psi},\quad
  \phi_d = k_{P_\psi} e_\psi + k_{D_\psi}\,\widehat{\dot e}_\psi.
  \]

  Inner roll (commands torque):

  \[
  e_\phi = \phi_d - \phi,\quad \widehat{\dot e}_\phi = -\,\widehat{\dot\phi},\quad
  \tau = k_{P_\phi} e_\phi + k_{D_\phi}\,\widehat{\dot e}_\phi.
  \]

Simulation Notes

- H.7: Step $\theta_d$ by $\pm5^\circ$ and verify well-damped tracking. The logged force $F$ should equal $F_e$ plus PD action.
- H.8: Step $\psi_d$ (e.g., $30^\circ$). The roll response should be visibly faster; the rise-time ratio should be near $M$.

