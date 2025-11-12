import matplotlib.pyplot as plt
import numpy as np
import massParam as P
from massDynamics import massDynamics
from ctrlPID import ctrlPID
from signalGenerator import signalGenerator
from massAnimation import massAnimation
from dataPlotter import dataPlotter

# instantiate plant, controller, and reference classes
mass = massDynamics(alpha=0.2)
controller = ctrlPID()
reference = signalGenerator(amplitude=0.5, frequency=0.04)
disturbance = signalGenerator(amplitude=0.25)

# instantiate the simulation plots and animation
dataPlot = dataPlotter()
animation = massAnimation()
t = P.t_start  # time starts at t_start
# student's massDynamics has outputs(); provide y as scalar wrapped to 2D for controller compatibility
y = np.array([[mass.outputs()]])

while t < P.t_end:  # main simulation loop
    # Propagate dynamics in between plot samples
    t_next_plot = t + P.t_plot

    while t < t_next_plot:  # updates control and dynamics at faster simulation rate
        r = reference.square(t)  # reference input
        d = 0.0  # disturbance disabled for HW6
        n = 0.0  # sensor noise not used in HW6
        u = controller.update(r, y + n)  # update controller
        mass.update(u + d)  # propagate system
        # refresh measured output for next step
        y = np.array([[mass.outputs()]])
        t += P.Ts  # advance time by Ts

    # update animation and data plots
    animation.update(mass.state)
    dataPlot.update(t, mass.state, u, r)
    plt.pause(0.0001)  # the pause causes the figure to be displayed during the simulation

# Keeps the program from closing until the user presses a button.
print('Press key to close')
plt.waitforbuttonpress()
plt.close()
