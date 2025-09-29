import numpy as np

# Physical constants
g = 9.81         # gravity [m/s^2]

# Geometry and masses (plausible placeholder values)
m1 = 0.06        # left rotor mass-equivalent [kg]
m2 = 0.06        # right rotor mass-equivalent [kg]
ell1 = 0.08      # left arm length [m]
ell2 = 0.08      # right arm length [m]
ellT = 0.10      # vertical offset (thrust arm) [m]
d = 0.12         # half-rotor separation for roll moment [m]

# Inertias about body axes (diagonal)
Jphi = 1.8e-3    # roll inertia [kg m^2]
Jtheta = 2.0e-3  # pitch inertia [kg m^2]
Jpsi = 1.5e-3    # yaw inertia [kg m^2]

# Damping (viscous)
b_phi = 1.0e-3
b_theta = 1.0e-3
b_psi = 1.0e-3

# Motor -> force gain
km = 0.8         # [N per unit PWM] (PWM in [0,1])

# Integration
Ts = 0.002       # sample time [s]

# Initial conditions (H.4 #1 requires zeros)
phi0 = 0.0
theta0 = 0.0
psi0 = 0.0
phidot0 = 0.0
thetadot0 = 0.0
psidot0 = 0.0

# Equilibrium thrust for hover at theta_e = 0
Fe = ((m1*ell1 + m2*ell2) * g) / ellT

# Mixing: [fl; fr] = mixing @ [F; tau]
# F = fl + fr, tau = d(fl - fr)  => mixing = [[1/2, 1/(2d)], [1/2, -1/(2d)]]
mixing = np.array([[0.5,  1.0/(2.0*d)],
                   [0.5, -1.0/(2.0*d)]])

# Unmixing: [F; tau] = unmixing @ [fl; fr]  => [[1,1],[d,-d]]
unmixing = np.array([[1.0, 1.0],
                     [d,   -d]])
