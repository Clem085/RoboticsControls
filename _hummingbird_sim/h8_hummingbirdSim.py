import matplotlib.pyplot as plt
import numpy as np
import hummingbirdParam as P
from signalGenerator import SignalGenerator
from hummingbirdAnimation import HummingbirdAnimation
from dataPlotter import DataPlotter
from hummingbirdDynamics import HummingbirdDynamics
from ctrlPD import CtrlPD


def saturate(u: np.ndarray, low: float, high: float):
    out = np.copy(u)
    for i in range(out.shape[0]):
        out[i, 0] = max(min(out[i, 0], high), low)
    return out


def run_h8():
    # instantiate plant, controller, and reference classes
    hummingbird = HummingbirdDynamics(alpha=0.0)
    controller = CtrlPD(P)
    psi_ref = SignalGenerator(amplitude=30.0 * np.pi / 180.0, frequency=0.02)
    theta_ref = SignalGenerator(amplitude=0.0, frequency=0.05)

    # instantiate the simulation plots and animation
    dataPlot = DataPlotter()
    animation = HummingbirdAnimation()

    t = 0.0
    t_end = 12.0
    Ts = P.Ts

    while t < t_end:
        # references and measurements
        psi_d = psi_ref.square(t)
        theta_d = theta_ref.square(t)
        y = hummingbird.h()
        phi = y[0, 0]
        theta = y[1, 0]
        psi = y[2, 0]

        # controller update -> (F, tau)
        state = (phi, theta, psi)
        ref = {'theta_d': theta_d, 'psi_d': psi_d}
        F_cmd, tau_cmd = controller.update(state, ref, Ts)

        # Convert [F; tau] -> [fl; fr] -> PWM
        ft = np.array([[F_cmd], [tau_cmd]])
        fl_fr = P.mixing @ ft
        pwm = fl_fr / P.km
        pwm = saturate(pwm, 0.0, 1.0)

        # Propagate dynamics
        hummingbird.update(pwm)

        # update animation and data plots
        refs = np.array([[controller.last_phi_d], [theta_d], [psi_d]])
        animation.update(t, hummingbird.state)
        dataPlot.update(t, hummingbird.state, pwm, refs)

        t += Ts
        plt.pause(0.0001)

    print('Press key to close')
    plt.waitforbuttonpress()
    plt.close()


if __name__ == "__main__":
    run_h8()
