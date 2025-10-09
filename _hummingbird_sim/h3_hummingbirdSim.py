import numpy as np
import hummingbirdParam as P
from hummingbirdDynamics import HummingbirdDynamics
from dataPlotter import DataPlotter
from hummingbirdAnimation import HummingbirdAnimation
from ctrlEquilibrium import ctrlEquilibrium

def _anim_update(anim, t, state):
    """Call animation update with whatever signature your GUI expects."""
    try:
        anim.update(t, state)   # common signature
        return
    except TypeError:
        pass
    try:
        anim.update(state)      # alternative signature
        return
    except TypeError:
        pass
    anim.update()               # fallback (no-arg)

def run_h3():
    # H.3: "freak out" sim with arbitrary PWM
    hb = HummingbirdDynamics(alpha=0.0)
    plotter = DataPlotter()
    anim = HummingbirdAnimation()

    t = 0.0
    t_end = 2.0
    Ts = P.Ts

    while t < t_end:
        u = np.array([[np.random.rand()], [np.random.rand()]])  # PWM in [0,1]
        hb.update(u)
        plotter.update(t, hb.state, u)
        _anim_update(anim, t, hb.state)
        t += Ts

    try: anim.close()
    except Exception: pass
    plotter.close()

def run_h4_equilibrium():
    # H.4 #1: hover equilibrium (F=Fe, tau=0)
    hb = HummingbirdDynamics(alpha=0.0)
    ctrl = ctrlEquilibrium()
    plotter = DataPlotter()
    anim = HummingbirdAnimation()

    t = 0.0
    t_end = 2.0
    Ts = P.Ts

    while t < t_end:
        u = ctrl.update(hb.state)
        hb.update(u)
        plotter.update(t, hb.state, u)
        _anim_update(anim, t, hb.state)
        t += Ts

    try: anim.close()
    except Exception: pass
    plotter.close()

if __name__ == "__main__":
    run_h3()
    run_h4_equilibrium()
