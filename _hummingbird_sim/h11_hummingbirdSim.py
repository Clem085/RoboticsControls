# LAB 5: HUMMINGBIRD SIMULATION
# Hummingbird simulation
# Connor Savugot

import matplotlib.pyplot as plt
import numpy as np
import hummingbirdParam as P
from signalGenerator import SignalGenerator
from hummingbirdAnimation import HummingbirdAnimation
from dataPlotter import DataPlotter
from hummingbirdDynamics import HummingbirdDynamics
from ctrlStateFeedbackIntegrator import ctrlStateFeedbackIntegrator

hummingbird = HummingbirdDynamics(alpha=0.05)
controller = ctrlStateFeedbackIntegrator()
psi_ref = SignalGenerator(amplitude=10.*np.pi/180., frequency=0.01)   # gentle yaw command
theta_ref = SignalGenerator(amplitude=8.*np.pi/180., frequency=0.015) # gentle pitch command

# PLOTS AND ANIMATION
dataPlot = DataPlotter()
animation = HummingbirdAnimation()

t = P.t_start  
y = hummingbird.h()
while t < P.t_end: 

    
    t_next_plot = t + P.t_plot
    while t < t_next_plot:
        # use square for pitch, sine for yaw to avoid large roll commands
        r = np.array([[theta_ref.square(t)], [psi_ref.sin(t)]])
        pwms, y_ref = controller.update(r, y)
        y = hummingbird.update(pwms)  # Propagate the dynamics
        t += P.Ts  # advance time by Ts


    animation.update(t, hummingbird.state)
    dataPlot.update(t, hummingbird.state, pwms, y_ref)

    # Maybe add pause
    plt.pause(0.0001)


print('Press key to close')
plt.waitforbuttonpress()
plt.close()
