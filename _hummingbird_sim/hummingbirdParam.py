import numpy as np

g = 9.81

# Geometry (Appendix B)
ell1 = 0.247      # m
ell2 = -0.039     # m
ell3x = -0.007    # m
ell3y = -0.007    # m
ell3z = 0.018     # m
ellT = 0.355      # m
d = 0.12          # m

# Masses
m1 = 0.108862     # kg
m2 = 0.4717       # kg
m3 = 0.1905       # kg

# Inertias (diagonal) kg-m^2
J1x, J1y, J1z = 0.000189, 0.001953, 0.001894
J2x, J2y, J2z = 0.000231, 0.003274, 0.003416
J3x, J3y, J3z = 0.0002222, 0.0001956, 0.000027

# Damping (manual sets phi=0.001 on each axis)
b_phi = 0.001
b_theta = 0.001
b_psi = 0.001

# Motor gain (km is identified later; use a placeholder for sim)
km = 0.8

# Integration step
Ts = 0.002

# Simulation timing
t_start = 0.0     # start time (s)
t_end = 20.0      # end time (s)
t_plot = 0.1      # plotting interval (s)

# Initial conditions
phi0 = 0.0
theta0 = 0.0
psi0 = 0.0
phidot0 = 0.0
thetadot0 = 0.0
psidot0 = 0.0

# Hover equilibrium (theta_e = 0) from the manual
Fe = ((m1*ell1 + m2*ell2) * g) / ellT

# Mixing / Unmixing
# [fl; fr] = mixing @ [F; tau],  with  F = fl + fr,  tau = d(fl - fr)
mixing = np.array([[0.5,  1.0/(2.0*d)],
                   [0.5, -1.0/(2.0*d)]])
# [F; tau] = unmixing @ [fl; fr]
unmixing = np.array([[1.0, 1.0],
                     [d,   -d]])
