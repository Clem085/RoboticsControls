#!/usr/bin/env python3
"""
runVTOLAnimation.py
Simulates the F.5/F.6 linear VTOL about hover using total inputs [F, tau].
Drives the same animation/plots, converting [F, tau] to [fr, fl] only for logging.
"""

import matplotlib
matplotlib.use("tkagg")

import numpy as np
import matplotlib.pyplot as plt

import VTOLParam as P
from signalGenerator import signalGenerator
from VTOLAnimation import VTOLAnimation
from dataPlotter import dataPlotter
from VTOLDynamics import LinearVTOL

# ------------------------------
# Reference / input signals (about hover)
# ------------------------------
# Keep F near Fe to stay in the linear regime; small oscillation added
F_sig   = signalGenerator(amplitude=2.0,  frequency=0.40, y_offset=P.Fe)
tau_sig = signalGenerator(amplitude=0.10, frequency=0.25, y_offset=0.0)

# ------------------------------
# Plant (linear F.6) + visuals
# ------------------------------
plant = LinearVTOL()
animation = VTOLAnimation()
logger    = dataPlotter()

# Print F.5/F.6 objects for the report
(num_h, den_h)     = plant.tf_vertical()
(num_th, den_th)   = plant.tf_pitch()
(num_zth, den_zth) = plant.tf_lateral_theta2z()
(num_ztau, den_ztau) = plant.tf_lateral_tau2z()
A,B,C,D = plant.ss()

print("F.5 TFs:")
print("  H/F     :", num_h,   "/", den_h)
print("  Theta/Tau:", num_th,  "/", den_th)
print("  Z/Theta :", num_zth, "/", den_zth)
print("  Z/Tau   :", num_ztau,"/", den_ztau)
print("\nF.6 State-space:")
print("A=\n", A); print("B=\n", B); print("C=\n", C); print("D=\n", D)

# ------------------------------
# Main loop (simulate linear model)
# ------------------------------
t = P.t_start
while t < P.t_end:
    # total inputs
    F   = F_sig.sin(t)
    tau = tau_sig.sin(t)
    u = np.array([[F], [tau]])

    # step the linear plant
    y = plant.update(u)     # y = [z, h, theta]^T (3x1)
    # assemble a full state for animation: [z, h, theta, zdot, hdot, thetadot]
    x = plant.state

    # convert total inputs to rotor thrusts FOR PLOTTING ONLY
    motor_thrusts = P.mixing @ u  # [fr; fl] = mixing @ [F; tau]

    # draw
    animation.update(x)  # z,h,theta used internally
    logger.update(t=t, states=x, motor_thrusts=motor_thrusts, z_ref=0.0, h_ref=0.0)

    t += P.t_plot
    plt.pause(0.001)

print("Close the figure window to end.")
plt.show(block=True)
