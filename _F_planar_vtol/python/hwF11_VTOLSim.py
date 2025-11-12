# Homework F.11: Full-State Feedback (Planar VTOL)
# Runs the nonlinear VTOL with a full-state feedback controller designed from
# the hover-linearized model. Controller outputs motor thrusts [fr; fl].

import matplotlib.pyplot as plt
import numpy as np
import VTOLParam as P
from VTOLDynamics import Dynamics
from ctrlFSF import ctrlFSF_VTOL
from signalGenerator import signalGenerator
from VTOLAnimation import VTOLAnimation
from dataPlotter import dataPlotter

VTOL = Dynamics(alpha=0.0)  # nominal parameters for F.11
controller = ctrlFSF_VTOL()
z_reference = signalGenerator(amplitude=4.0, frequency=0.05, y_offset=0.0)
h_reference = signalGenerator(amplitude=3.0, frequency=0.03, y_offset=5.0)

dataPlot = dataPlotter()
animation = VTOLAnimation()

t = P.t_start
y = VTOL.h()

while t < P.t_end:
    t_next_plot = t + P.t_plot

    while t < t_next_plot:
        h_ref = h_reference.square(t)
        z_ref = z_reference.square(t)
        r = np.array([[z_ref], [h_ref]])
        d = np.array([[0.0], [0.0]])  # no external disturbances here
        u = controller.update(r, VTOL.state)
        y = VTOL.update(u + d)
        t += P.Ts

    animation.update(VTOL.state, z_ref)
    dataPlot.update(t, VTOL.state, u, z_ref, h_ref)
    plt.pause(0.0001)

print('Press key to close')
plt.waitforbuttonpress()
plt.close('all')

