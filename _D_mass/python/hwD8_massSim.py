"""
Homework D.8: Mass-Spring-Damper PD Control Simulation
Outcome aligned with provided solution: single PD with saturation,
tracking a square reference.
"""

import matplotlib.pyplot as plt
import numpy as np
import massParam as P
from signalGenerator import signalGenerator
from massAnimation import massAnimation
from dataPlotter import dataPlotter
from massDynamics import massDynamics
from ctrlPD import ctrlPD

# instantiate plant, controller, and reference classes
mass = massDynamics()
controller = ctrlPD()
reference = signalGenerator(amplitude=1.0, frequency=0.04)

# instantiate the simulation plots and animation
dataPlot = dataPlotter()
animation = massAnimation()
t = P.t_start
y = mass.outputs()

while t < P.t_end:
    t_next_plot = t + P.t_plot
    while t < t_next_plot:
        r = reference.square(t)
        u = controller.update(r, mass.state)
        y = mass.update(u)
        t += P.Ts

    animation.update(mass.state)
    dataPlot.update(t, mass.state, u, r)
    plt.pause(0.0001)

print('Press key to close')
plt.waitforbuttonpress()
plt.close('all')
