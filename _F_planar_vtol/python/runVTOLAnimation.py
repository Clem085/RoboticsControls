#!/usr/bin/env python3
"""
run_VTOLAnimation.py
Runs BOTH the VTOL animation and the 5-panel data plot using the exact
signal setup from the provided "solution code".

- z, h, theta are driven by simple reference signals (no plant dynamics).
- Force is centered at hover equilibrium Fe.
- Torque is a small, faster sinusoid.
- Motor thrusts are computed via the provided mixing matrix.

If you want the green reference lines (z_r, h_r) to be visible at zero,
leave z_ref and h_ref as 0.0 in the call to dataPlot.update().
"""

import numpy as np
import matplotlib.pyplot as plt

import VTOLParam as P
from signalGenerator import signalGenerator
from VTOLAnimation import VTOLAnimation
from dataPlotter import dataPlotter

# ------------------------------
# Reference / input signals
# ------------------------------
z_plot = signalGenerator(amplitude=4.0, frequency=0.1, y_offset=5.0)
h_plot = signalGenerator(amplitude=2.0, frequency=0.1, y_offset=2.0)
theta_plot = signalGenerator(amplitude=np.pi/8.0, frequency=0.5, y_offset=0.0)

# Center total thrust around hover equilibrium Fe
# (Fe = (mc + 2*mr)*g is defined in VTOLParam.py)
force_plot = signalGenerator(amplitude=5.0, frequency=0.5, y_offset=P.Fe)

# Small torque; relatively fast
torque_plot = signalGenerator(amplitude=0.1, frequency=10.0, y_offset=0.0)

# ------------------------------
# Plotter & Animation
# ------------------------------
dataPlot = dataPlotter()
animation = VTOLAnimation()

# ------------------------------
# Main loop
# ------------------------------
t = P.t_start
while t < P.t_end:
    # Generate “reference” values (no dynamics)
    z = z_plot.sin(t)
    h = h_plot.sin(t)
    theta = theta_plot.sin(t)

    # Generate force/torque and convert to motor thrusts
    f = force_plot.sin(t)
    tau = torque_plot.sin(t)
    motor_thrusts = P.mixing @ np.array([[f], [tau]])  # [f_left, f_right]^T

    # Assemble state vector expected by the visualizers
    state = np.array([[z], [h], [theta], [0.0], [0.0], [0.0]])

    # Update animation and data plots
    animation.update(state)  # target marker uses default (0.0)
    # Pass z_ref and h_ref as 0.0 so green baselines appear at zero
    dataPlot.update(t=t, states=state, motor_thrusts=motor_thrusts, z_ref=0.0, h_ref=0.0)

    # Advance time and draw
    t += P.t_plot
    plt.pause(0.02)

print("Press any key in the plot window to close.")
plt.waitforbuttonpress()
plt.close()
