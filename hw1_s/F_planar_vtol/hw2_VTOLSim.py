import matplotlib.pyplot as plt
import numpy as np
import VTOLParam as P
from signalGenerator import signalGenerator
from VTOLAnimation import VTOLAnimation
from dataPlotter import dataPlotter


# instantiate reference input classes
z_plot = signalGenerator(amplitude=4.0, frequency=0.1, y_offset=5.0)
h_plot = signalGenerator(amplitude=2.0, frequency=0.1, y_offset=2.0)
theta_plot = signalGenerator(amplitude=np.pi/8.0, frequency=0.5, y_offset=0.0)
force_plot = signalGenerator(amplitude=5, frequency=0.5, y_offset=P.g * (P.mc + 2.0 * P.mr))
torque_plot = signalGenerator(amplitude=.1, frequency=10.)

# instantiate the simulation plots and animation
dataPlot = dataPlotter()
animation = VTOLAnimation()

t = P.t_start  # time starts at t_start
while t < P.t_end:  # main simulation loop
    # set variables
    z = z_plot.sin(t)
    h = h_plot.sin(t)
    theta = theta_plot.sin(t)
    f = force_plot.sin(t)
    tau = torque_plot.sin(t)
    motor_thrusts = P.mixing @ np.array([[f], [tau]])

    # update animation
    state = np.array([[z], [h], [theta], [0.0], [0.0], [0.0]])
    animation.update(state) # values for z_r, h_r, f, and tau could be zero in hw 2 as well
    dataPlot.update(t, state, motor_thrusts)

    t += P.t_plot  # advance time by t_plot
    plt.pause(0.02)

# Keeps the program from closing until the user presses a button.
print('Press key to close')
plt.waitforbuttonpress()
plt.close()
