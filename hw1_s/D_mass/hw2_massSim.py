import matplotlib.pyplot as plt
import numpy as np
import massParam as P
from signalGenerator import signalGenerator
from massAnimation import massAnimation
from dataPlotter import dataPlotter

# instantiate reference input classes
z_plot = signalGenerator(amplitude=1, frequency=0.5, y_offset=0.2)
f_plot = signalGenerator(amplitude=2, frequency=0.5)

# instantiate the simulation plots and animation
dataPlot = dataPlotter()
animation = massAnimation()

t = P.t_start  # time starts at t_start
while t < P.t_end:  # main simulation loop
    # set variables
    z = z_plot.sin(t)
    f = f_plot.sawtooth(t)
    # update animation
    state = np.array([[z], [0.0]])
    animation.update(state)
    dataPlot.update(t, state, f)

    t += P.t_plot  # advance time by t_plot
    plt.pause(0.1)

# Keeps the program from closing until the user presses a button.
print('Press key to close')
plt.waitforbuttonpress()
plt.close()
