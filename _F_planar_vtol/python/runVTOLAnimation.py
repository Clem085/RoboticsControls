#!/usr/bin/env python3
"""
run_VTOLAnimation.py
Runs BOTH the VTOL animation and the 5-panel data plot with scripted signals
(no plant dynamics). Matches the “solution” signal setup.
"""

# ---- pick a stable backend BEFORE importing pyplot (mirrors your mass demo) ---
import matplotlib
matplotlib.use("tkagg")  # same as massAnimation; avoids Qt-related freezes

import numpy as np
import matplotlib.pyplot as plt

import VTOLParam as P
from signalGenerator import signalGenerator
from VTOLAnimation import VTOLAnimation
from dataPlotter import dataPlotter

# ------------------------------
# Reference / input signals
# ------------------------------
z_plot     = signalGenerator(amplitude=4.0, frequency=0.10, y_offset=5.0)
h_plot     = signalGenerator(amplitude=2.0, frequency=0.10, y_offset=2.0)
theta_plot = signalGenerator(amplitude=np.pi/8.0, frequency=0.50, y_offset=0.0)

# Center total thrust around hover equilibrium Fe
force_plot = signalGenerator(amplitude=5.0, frequency=0.50, y_offset=P.Fe)

# Keep torque modest and not ultra-fast to reduce redraw load
torque_plot = signalGenerator(amplitude=0.10, frequency=0.50, y_offset=0.0)

# ------------------------------
# Plotter & Animation
# ------------------------------
logger    = dataPlotter()     # 5 stacked plots
animation = VTOLAnimation()   # vehicle view

# ------------------------------
# Main loop
# ------------------------------
t = P.t_start
while t < P.t_end:
    # Scripted “state” (no dynamics)
    z     = z_plot.sin(t)
    h     = h_plot.sin(t)
    theta = theta_plot.sin(t)

    # Total force/torque -> motor thrusts
    F   = force_plot.sin(t)
    tau = torque_plot.sin(t)
    motor_thrusts = P.mixing @ np.array([[F], [tau]])  # [f_left; f_right]

    # State vector for visualizers: [z, h, theta, zdot, hdot, thetadot]
    state = np.array([[z], [h], [theta], [0.0], [0.0], [0.0]])

    # Draw animation and plots
    animation.update(state)  # target defaults to 0.0
    logger.update(t=t, states=state, motor_thrusts=motor_thrusts, z_ref=0.0, h_ref=0.0)

    # Match pacing to your mass demo to keep UI responsive
    t += P.t_plot
    plt.pause(0.1)  # same pause duration you used with the mass script

print("Press any key in the plot window to close.")
plt.waitforbuttonpress()
plt.close()
