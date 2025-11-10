"""
Homework F.8: VTOL PD Control Simulation
Adjusted to match solution behavior and interfaces.
"""

import numpy as np
import VTOLParam as P
from ctrlPD import ctrlPD
from VTOLDynamics import Dynamics
from VTOLAnimation import VTOLAnimation
from dataPlotter import dataPlotter
from signalGenerator import signalGenerator

# instantiate VTOL, controller, and reference classes
VTOL = Dynamics()
controller = ctrlPD()
z_reference = signalGenerator(amplitude=4.0, frequency=0.05, y_offset=5.0)
h_reference = signalGenerator(amplitude=3.0, frequency=0.03, y_offset=5.0)

# instantiate the simulation plots and animation
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
        u = controller.update(r, VTOL.state)  # motor thrusts [fr; fl]
        y = VTOL.update(u)
        t += P.Ts

    animation.update(VTOL.state, z_ref)
    dataPlot.update(t, VTOL.state, u, z_ref, h_ref)
    plt_pause = False
    try:
        import matplotlib.pyplot as plt
        plt.pause(0.0001)
        plt_pause = True
    except Exception:
        pass

print('Press key to close')
try:
    import matplotlib.pyplot as plt
    plt.waitforbuttonpress()
    plt.close()
except Exception:
    pass
