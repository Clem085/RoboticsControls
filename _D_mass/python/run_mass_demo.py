
import numpy as np
import matplotlib.pyplot as plt

import massParam as P
from signalGenerator import signalGenerator
from massAnimation import massAnimation
from dataPlotter import dataPlotter

# ------------------------------
# Signals (match the solution)
# ------------------------------
z_plot = signalGenerator(amplitude=1.0, frequency=0.5, y_offset=0.2)
f_plot = signalGenerator(amplitude=2.0, frequency=0.5)

# ------------------------------
# Visuals
# ------------------------------
animation = massAnimation()
logger = dataPlotter()

# ------------------------------
# Main loop
# ------------------------------
t = P.t_start
while t < P.t_end:
    # scripted “state” (no dynamics): state = [z; zdot]
    z = z_plot.sin(t)
    f = f_plot.sawtooth(t)
    state = np.array([[z], [0.0]])

    # update animation
    animation.update(state)

    # update data logger
    # Most mass homework templates use update(t, state, f).
    # If your dataPlotter variant uses a slightly different signature,
    # we try a compatible call automatically.
    try:
        logger.update(t, state, f)                           # common mass signature
    except TypeError:
        try:
            logger.update(t=t, states=state, ctrl=f)         # some variants use keywords
        except TypeError:
            # As a last resort, just call positional again (helps if kwargs differ)
            logger.update(t, state, f)

    # advance time and draw
    t += P.t_plot
    plt.pause(0.1)

print("Press any key in the plot window to close.")
plt.waitforbuttonpress()
plt.close()