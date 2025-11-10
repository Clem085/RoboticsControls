"""
Homework D.10: Digital PID for Mass–Spring–Damper

Implements measurement-only PID with filtered derivative (sigma=0.05)
and either Backward-Euler or Tustin integrator. Includes parameter
uncertainty (±20%) and simple anti-windup via integrator clamping.
"""

import matplotlib.pyplot as plt
import numpy as np
import massParam as P
from signalGenerator import signalGenerator
from massAnimation import massAnimation
from dataPlotter import dataPlotter
from massDynamics import massDynamics
from ctrlPID_digital import CtrlPIDDigital, PIDGains


def main():
    # Plant with parameter uncertainty alpha=0.2 (±20%)
    plant = massDynamics(alpha=0.2)

    # Choose PID gains. Numerical values depend on D.8; set a reasonable baseline.
    # Start with PD from D.8 and add a small KI, then tune as needed.
    # You can adjust these interactively to achieve desired dynamics.
    gains = PIDGains(
        kP=plant.k,        # rough proportional gain baseline
        kI=1.0,            # start small, increase to remove SSE
        kD=plant.b,        # rough derivative baseline
    )

    controller = CtrlPIDDigital(
        gains=gains,
        sigma=0.05,
        Ts=P.Ts,
        integral_method="backward_euler",  # or "tustin"
        use_anti_windup=True,
        u_min=-P.F_max,
        u_max=P.F_max,
    )

    # Reference: step in position (to test zero SSE with integral action)
    reference = signalGenerator(amplitude=0.5, frequency=0.05)

    # Plots and animation
    plotter = dataPlotter()
    animation = massAnimation()

    t = P.t_start
    state_output = plant.outputs()
    u_cmd = 0.0

    while t < P.t_end:
        t_next_plot = t + P.t_plot
        while t < t_next_plot:
            r = reference.step(t)
            y = plant.outputs()
            u_cmd = controller.update(r, y)
            state_output = plant.update(u_cmd)
            t += P.Ts

        animation.update(plant.state)
        plotter.update(t, plant.state, u_cmd, r)
        plt.pause(0.0001)

    print("Press key to close")
    plt.waitforbuttonpress()
    plt.close("all")


if __name__ == "__main__":
    main()

