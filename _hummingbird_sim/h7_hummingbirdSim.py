import matplotlib.pyplot as plt
import numpy as np
import hummingbirdParam as P
from signalGenerator import SignalGenerator
from hummingbirdAnimation import HummingbirdAnimation
from dataPlotter import DataPlotter
from hummingbirdDynamics import HummingbirdDynamics
from ctrlLonPD import CtrlLonPD


def saturate(u: np.ndarray, low: float, high: float):
    out = np.copy(u)
    for i in range(out.shape[0]):
        out[i, 0] = max(min(out[i, 0], high), low)
    return out


def run_h7():
    # Instantiate plant, controller, references
    hummingbird = HummingbirdDynamics(alpha=0.0)
    ctrl = CtrlLonPD(P)
    theta_ref = SignalGenerator(amplitude=5.0 * np.pi / 180.0, frequency=0.05)

    dataPlot = DataPlotter()
    animation = HummingbirdAnimation()

    t = 0.0
    t_end = 10.0
    Ts = P.Ts

    while t < t_end:
        # references and measurements
        theta_d = theta_ref.square(t)
        y = hummingbird.h()
        theta = y[1, 0]

        # PD force command
        F_cmd = ctrl.update(theta, theta_d, Ts)

        # Convert [F; tau=0] -> [fl; fr] -> PWM
        ft = np.array([[F_cmd], [0.0]])
        fl_fr = P.mixing @ ft
        pwm = fl_fr / P.km
        pwm = saturate(pwm, 0.0, 1.0)

        # Propagate dynamics
        hummingbird.update(pwm)

        # Log and animate
        refs = np.array([[0.0], [theta_d], [0.0]])
        animation.update(t, hummingbird.state)
        dataPlot.update(t, hummingbird.state, pwm, refs)

        t += Ts
        plt.pause(0.0001)

    print('Press key to close')
    plt.waitforbuttonpress()
    plt.close()


if __name__ == "__main__":
    run_h7()
