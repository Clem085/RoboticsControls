"""
Homework F.12: VTOL FSF with Integrators

Simulates the planar VTOL with full-state feedback plus integral action on both
altitude and lateral position. Includes ±20% parameter variation and a constant
0.1 N lateral wind disturbance.
"""

import matplotlib.pyplot as plt
import numpy as np
import VTOLParam as P
from VTOLDynamics import Dynamics
from ctrlFSFInt import ctrlFSFInt_VTOL
from signalGenerator import signalGenerator
from VTOLAnimation import VTOLAnimation
from dataPlotter import dataPlotter

VTOL = Dynamics(alpha=0.2)
VTOL.F_wind = 0.1  # constant lateral disturbance (N)
controller = ctrlFSFInt_VTOL()
z_reference = signalGenerator(amplitude=4.0, frequency=0.05, y_offset=0.0)
h_reference = signalGenerator(amplitude=3.0, frequency=0.03, y_offset=5.0)

dataPlot = dataPlotter()
animation = VTOLAnimation()

t = P.t_start
while t < P.t_end:
    t_next_plot = t + P.t_plot

    while t < t_next_plot:
        z_ref = z_reference.square(t)
        h_ref = h_reference.square(t)
        r = np.array([[z_ref], [h_ref]])
        motor_cmd = controller.update(r, VTOL.state)
        VTOL.update(motor_cmd)
        t += P.Ts

    animation.update(VTOL.state, z_ref)
    dataPlot.update(t, VTOL.state, motor_cmd, z_ref, h_ref)
    plt.pause(0.0001)

print('Press key to close')
plt.waitforbuttonpress()
plt.close('all')

