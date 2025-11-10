import numpy as np
import matplotlib.pyplot as plt

import massParam as P
from signalGenerator import signalGenerator
from massAnimation import massAnimation
from dataPlotter import dataPlotter
from massDynamics import massDynamics

# ------------------------------
# Signals (inputs & references)
# ------------------------------
# External force F(t): feel free to swap sin/step/square
F_sig   = signalGenerator(amplitude=2.0, frequency=0.5, y_offset=0.0)
z_ref_s = signalGenerator(amplitude=0.0)  # keep reference at 0 for now

# ------------------------------
# Plant (D.6) + visuals
# ------------------------------
plant = massDynamics(integrator="rk4")
animation = massAnimation()
logger = dataPlotter()

# Print D.5/D.6 objects for the report
num, den = plant.tf()
A, B, C, D = plant.ss()
print("D.5 transfer function Z/F: num =", num, " den =", den)
print("D.6 state space:")
print("A=\n", A); print("B=\n", B); print("C=\n", C); print("D=\n", D)

# ------------------------------
# Simulation loop
# ------------------------------
t = P.t_start
while t < P.t_end:
    F = F_sig.sin(t)             # input force at time t
    x = plant.update(F)          # integrate dynamics with input F
    z_ref = z_ref_s.step(1.0)    # constant 0 (or change to track)
    y = plant.output(F)          # measured output y = z

    # update animation (expects state = [z, zdot])
    animation.update(x)

    # update plots: time, state, control (and optional reference)
    logger.update(t, x, F, reference=z_ref)

    # advance time and draw
    t += P.t_plot
    plt.pause(0.001)

print("Close the figure window to end.")
plt.show(block=True)
