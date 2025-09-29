import matplotlib.pyplot as plt
import numpy as np
import hummingbirdParam as P
from hummingbirdAnimation import HummingbirdAnimation
from dataPlotter import DataPlotter
from hummingbirdDynamics import HummingbirdDynamics
from ctrlEquilibrium import ctrlEquilibrium

# dynamics
hummingbird = HummingbirdDynamics(alpha=0.0)

# tiny PD trim (for the “poke → settle” demo)
Kp = 0.05   # per rad
Kd = 0.02   # per rad/s

# equilibrium controller: F = Fe, tau = 0
ctrl = ctrlEquilibrium()

# plots/animation
dataPlot = DataPlotter()
animation = HummingbirdAnimation()

t = P.t_start
while t < P.t_end:
    t_next_plot = t + P.t_plot
    while t < t_next_plot:
        # base open-loop equilibrium command
        u_pwm, _ = ctrl.update(hummingbird.state)

        # # optional excite: +5% thrust for first 0.5 s
        # if t < 0.5:
        #     u_pwm = 1.05 * u_pwm

        # PD trim around equilibrium on pitch (negative feedback)
        theta  = float(hummingbird.state[1,0])
        thetad = float(hummingbird.state[4,0])
        corr = -Kp*theta - Kd*thetad
        u_pwm = u_pwm + np.array([[corr],[corr]], dtype=float)

        # clamp to [0,1]
        u_pwm = np.clip(u_pwm, 0.0, 1.0)

        # step dynamics
        y = hummingbird.update(u_pwm)
        t += P.Ts

    # update visuals (pass u_pwm, not 'u')
    animation.update(t, hummingbird.state)
    dataPlot.update(t, hummingbird.state, u_pwm)

    plt.pause(0.05)

print('Press key to close')
plt.waitforbuttonpress()
plt.close()
