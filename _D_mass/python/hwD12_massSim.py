"""
Homework D.12: Full-State Feedback with Integrator (Mass-Spring-Damper)

Adds an integral state with anti-windup, introduces parameter uncertainty (20%),
and injects a constant 0.25 N disturbance at the plant input.
"""

import matplotlib.pyplot as plt
import numpy as np
import massParam as P
from massDynamics import massDynamics
from ctrlFSFInt import ctrlFSFInt
from signalGenerator import signalGenerator
from massAnimation import massAnimation
from dataPlotter import dataPlotter

mass = massDynamics(alpha=0.2)  # allow ±20% parameter variation
controller = ctrlFSFInt(p_int=-3.0)
reference = signalGenerator(amplitude=0.5, frequency=0.04)

dataPlot = dataPlotter()
animation = massAnimation()
t = P.t_start
y = np.array([[mass.outputs()]])
disturbance = 0.25  # constant input disturbance (N)

while t < P.t_end:
    t_next_plot = t + P.t_plot

    while t < t_next_plot:
        z_ref = reference.square(t)
        n = 0.0  # measurement noise
        u = controller.update(z_ref, y + n)
        mass.update(u + disturbance)
        y = np.array([[mass.outputs()]])
        t += P.Ts

    animation.update(mass.state)
    dataPlot.update(t, mass.state, u, z_ref)
    plt.pause(0.0001)

print('Press key to close')
plt.waitforbuttonpress()
plt.close('all')

