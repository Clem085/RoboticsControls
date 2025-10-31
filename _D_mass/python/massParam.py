# mass-spring-damper Parameter File
import numpy as np

# Physical parameters
m = 5.0      # kg
k = 3.0      # N/m
b = 0.5      # N*s/m

# animation
length = 5.0
width  = 1.0

# Initial Conditions
z0    = 0.0   # m
zdot0 = 0.0   # m/s

# Simulation Parameters
t_start = 0.0
t_end   = 10.0
Ts      = 0.01
t_plot  = 0.1

# limits (not used here)
F_max = 5.0
