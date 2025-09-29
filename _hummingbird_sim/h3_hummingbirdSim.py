import numpy as np
import hummingbirdParam as P
from hummingbirdDynamics import HummingbirdDynamics
from dataPlotter import DataPlotter
from hummingbirdAnimation import HummingbirdAnimation
from ctrlEquilibrium import ctrlEquilibrium

def _anim_update(anim, t, state):
    """
    Call the animation update with whatever signature your GUI expects:
      - update(t, state)
      - update(state)
      - update()
    """
    try:
        # Most GUI versions in this course expect (t, state)
        anim.update(t, state)
        return
    except TypeError:
        pass
    try:
        # Some versions expect only (state)
        anim.update(state)
        return
    except TypeError:
        pass
    # Last resort: no-arg update
    anim.update()

def run_h3():
    """
    H.3: simulate with arbitrary PWM so it 'freaks out' (as instructed).
    This is purely to exercise the physics; the motion is not meaningful.
    """
    hb = HummingbirdDynamics(alpha=0.0)
    plotter = DataPlotter()
    anim = HummingbirdAnimation()

    t = 0.0
    t_end = 2.0
    Ts = P.Ts

    while t < t_end:
        # random PWM in [0, 1]
        u = np.array([[np.random.rand()], [np.random.rand()]])
        hb.update(u)
        plotter.update(t, hb.state, u)
        _anim_update(anim, t, hb.state)
        t += Ts

    try:
        anim.close()
    except Exception:
        pass
    plotter.close()

def run_h4_equilibrium():
    """
    H.4 #1: verify equilibrium at hover by setting F=Fe and tau=0 via ctrlEquilibrium.
    Angles should remain near zero for the provided parameters.
    """
    hb = HummingbirdDynamics(alpha=0.0)
    ctrl = ctrlEquilibrium()
    plotter = DataPlotter()
    anim = HummingbirdAnimation()

    t = 0.0
    t_end = 2.0
    Ts = P.Ts

    while t < t_end:
        u = ctrl.update(hb.state)  # PWM for F=Fe, tau=0
        hb.update(u)
        plotter.update(t, hb.state, u)
        _anim_update(anim, t, hb.state)
        t += Ts

    try:
        anim.close()
    except Exception:
        pass
    plotter.close()

if __name__ == "__main__":
    # Run both parts back-to-back. Your grader can run either or both.
    run_h3()
    run_h4_equilibrium()
