# run_vtol_animation.py
import numpy as np
import matplotlib.pyplot as plt
import VTOLParam as P
from VTOLAnimation import VTOLAnimation
from signalGenerator import signalGenerator

# Signals (all valid in your signalGenerator)
zv_sig    = signalGenerator(amplitude=3.0,  frequency=0.05, y_offset=0.0)  # lateral
h_sig     = signalGenerator(amplitude=2.0,  frequency=0.04, y_offset=6.0)  # altitude
theta_sig = signalGenerator(amplitude=0.25, frequency=0.20, y_offset=0.0)  # pitch (rad)
zt_sig    = signalGenerator(amplitude=4.0,  frequency=0.03, y_offset=0.0)  # ground target

anim = VTOLAnimation()

t = P.t_start
while t < P.t_end:
    zv    = zv_sig.sin(t)
    h     = h_sig.sin(t + 0.8)       # phase shift so it doesn't move in lockstep
    theta = theta_sig.sin(t + 0.4)
    zt    = zt_sig.square(t)

    x = np.array([[zv], [h], [theta]])   # VTOLAnimation expects [zv, h, theta]^T
    anim.update(x, target=zt)            # and a separate ground target zt
    plt.pause(P.t_plot)
    t += P.t_plot

plt.savefig("vtol_screenshot.png", dpi=200)
print("Saved frame to vtol_screenshot.png")
