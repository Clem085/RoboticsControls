import numpy as np
import hummingbirdParam as P

def saturate(u, low, high):
    if isinstance(u, float):
        return max(min(u, high), low)
    out = np.copy(u)
    for i in range(out.shape[0]):
        out[i,0] = max(min(out[i,0], high), low)
    return out

class ctrlEquilibrium:
    def __init__(self):
        pass

    def update(self, x):
        # Equilibrium inputs: F=Fe, tau=0  -> [fl, fr] via mixing, then PWM via km
        ft = np.array([[P.Fe], [0.0]])               # [F; tau]
        fl_fr = P.mixing @ ft                        # [fl; fr]
        pwm = fl_fr / P.km
        pwm = saturate(pwm, 0.0, 1.0)
        return pwm
