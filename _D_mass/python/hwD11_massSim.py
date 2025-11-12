"""
Homework D.11: Full-State Feedback (Mass–Spring–Damper)

Runs the mass plant with a full-state feedback controller (using a dirty-
derivative to estimate z_dot from the measured position).
"""

import matplotlib.pyplot as plt
import numpy as np
import massParam as P
from massDynamics import massDynamics
from ctrlFSF import ctrlFSF
from signalGenerator import signalGenerator
from massAnimation import massAnimation
from dataPlotter import dataPlotter


# instantiate plant (no uncertainty for D.11)
mass = massDynamics(alpha=0.0)
controller = ctrlFSF()  # uses dirty derivative internally
reference = signalGenerator(amplitude=0.5, frequency=0.04)

# instantiate the simulation plots and animation
dataPlot = dataPlotter()
animation = massAnimation()
t = P.t_start
y = np.array([[mass.outputs()]])  # measured position z

while t < P.t_end:
    t_next_plot = t + P.t_plot

    while t < t_next_plot:
        r = reference.square(t)  # scalar reference for position
        d = 0.0                  # no disturbance
        n = 0.0                  # no sensor noise
        u = controller.update(r, y + n)
        mass.update(u + d)
        y = np.array([[mass.outputs()]])
        t += P.Ts

    animation.update(mass.state)
    dataPlot.update(t, mass.state, u, r)
    plt.pause(0.0001)

print('Press key to close')
plt.waitforbuttonpress()
plt.close('all')

