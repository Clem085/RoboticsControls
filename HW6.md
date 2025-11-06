## Part VI Homework — D.9(a), D.10, F.9, F.10

Concise summary with clean math, run commands, and result placeholders.

### Run Commands (what to execute)
- Mass PD (D.8, for reference):
  - `python _D_mass/python/hwD8_massSim.py`
- Mass digital PID (D.10):
  - `python _D_mass/python/hwD10_massSim.py`
- VTOL nested PID (F.10):
  - Controller: `hw1_s/F_planar_vtol/ctrlNestedPID_digital.py:1`
  - Ask to add a small VTOL sim runner if needed.

### Paste Your Output Images Here
- D.9(a) type/SSE: [Paste image]
- D.10 mass PID step/disturbance: [Paste image]
- F.9 VTOL type summaries: [Paste image]
- F.10 nested PID step in h and z: [Paste image]

---

### D.9(a) — System Type & Integrators (Mass–Spring–Damper)

Unity feedback error transfer and final value theorem:
$$
E(s)=\frac{1}{1+P(s)C(s)}\,R(s),\qquad e_\infty=\lim_{s\to0}sE(s).
$$

- PD: \(C_{\mathrm{PD}}(s)=k_P+k_D s\) → type 0 (no free integrators).
  - Step: finite SSE; Ramp: \(\infty\); Parabola: \(\infty\).
- Add integrator: \(C_{\mathrm{PID}}(s)=k_P+\tfrac{k_I}{s}+k_D s\) → type 1.
  - Step: 0 SSE; Ramp: finite; Parabola: \(\infty\).

Takeaway: a single integrator guarantees zero SSE to steps (more robust to plant uncertainty).

---

### D.10 — Digital PID (Mass–Spring–Damper)

Continuous‑time form with dirty derivative (\(\sigma=0.05\)):
$$
C(s)=k_P+k_I\,\frac{1}{s}+k_D\,\frac{s}{\sigma s+1}.
$$
Position form: \(u = k_P e + I + k_D\,\dot e_f\), with \(e=r-y\).

Integral discretizations (sample time \(T_s\)):
- Backward‑Euler:
  $$ I[k]=I[k-1]+k_I T_s\,e[k]. $$
- Tustin (trapezoidal):
  $$ I[k]=I[k-1]+\frac{k_I T_s}{2}\,(e[k]+e[k-1]). $$

Filtered derivative (dirty derivative):
$$
\dot e_f[k]=\frac{\dot e_f[k-1]+\dfrac{e[k]-e[k-1]}{T_s}}{1+\dfrac{\sigma}{T_s}}.
$$

Measurement‑only difference equations (position form):
```text
e[k]   = r[k] - y[k]
I[k]   = I[k-1] + k_I*T_s*e[k]          # or trapezoidal form above
dhat[k]= (dhat[k-1] + (e[k]-e[k-1])/T_s) / (1 + sigma/T_s)
u[k]   = k_P*e[k] + I[k] + k_D*dhat[k]
```

Files:
- Controller: `_D_mass/python/ctrlPID_digital.py:1`
- Simulation: `_D_mass/python/hwD10_massSim.py:1`

Checklist:
- Step tracking: SSE ≈ 0 with \(k_I>0\).
- Parameter uncertainty (±20%): PID removes bias that PD leaves.
- Reasonable overshoot/settling; derivative filter tames noise.

---

### F.9 — System Type & Integrators (VTOL)

- Altitude (h), PD → type 0; add I in altitude loop → type 1 (step 0 SSE).
- Attitude (θ) inner loop, PD → type 0 w.r.t. θ command; constant torque disturbance → finite SSE; usually no I here.
- Lateral position (z) outer loop, PD → type 0; add I in outer loop → type 1 (step 0 SSE).

---

### F.10 — Digital Nested PID (VTOL)

Architecture:
- Inner attitude (θ): PD with dirty derivative (fast loop).
- Outer altitude (h): PID with integrator.
- Outer lateral (z): PID producing `θ_cmd` with integrator.

Discrete‑time skeleton:
```text
# altitude (h)
e_h[k]   = h_r[k] - h[k]
I_h[k]   = I_h[k-1] + kI_h*T_s*e_h[k]
dhat_h[k]= (dhat_h[k-1] + (e_h[k]-e_h[k-1])/T_s)/(1+sigma/T_s)
F_dev[k] = kP_h*e_h[k] + I_h[k] + kD_h*dhat_h[k]
F[k]     = Fe + F_dev[k]

# outer lateral (z) → theta_cmd
e_z[k]   = z_r[k] - z[k]
I_z[k]   = I_z[k-1] + kI_z*T_s*e_z[k]
dhat_z[k]= (dhat_z[k-1] + (e_z[k]-e_z[k-1])/T_s)/(1+sigma/T_s)
theta_cmd[k] = kP_z*e_z[k] + I_z[k] + kD_z*dhat_z[k]

# inner attitude (theta)
e_th[k]  = theta_cmd[k] - theta[k]
dhat_th  = (dhat_th[k-1] + (e_th[k]-e_th[k-1])/T_s)/(1+sigma/T_s)
tau[k]   = kP_th*e_th[k] + kD_th*dhat_th
```

Implementation: `hw1_s/F_planar_vtol/ctrlNestedPID_digital.py:1` (includes hover force `Fe` and mixing to motor forces).

Checklist:
- Step in `h` and `z`: SSE → 0.
- Inner θ loop faster than outer loops.
- Works under ±20% variation in `(m_c,J_c,d,μ)` when integrators are active.

