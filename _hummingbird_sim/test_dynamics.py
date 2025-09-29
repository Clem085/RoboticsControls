# test_dynamics.py
import numpy as np
from hummingbirdDynamics import HummingbirdDynamics

def check_symmetric(M, tol=1e-9):
    return np.allclose(M, M.T, atol=tol)

def main():
    dyn = HummingbirdDynamics()

    # Pick a pose and probe the mass matrix symmetry (sanity check for H.3)
    q = np.array([0.2, -0.1, 0.3])  # [phi, theta, psi]
    M = dyn._M(q)
    assert check_symmetric(M), "Mass matrix must be symmetric"

    # Try one forward step using whichever API your class exposes
    if hasattr(dyn, "step"):
        # Force-space API: u is [fL, fR], needs Ts
        u = np.array([0.0, 0.0])
        x_next = dyn.step(u, Ts=0.002)
    else:
        # PWM-space API: u is 2x1 [[uL],[uR]] in [0,1]
        u = np.array([[0.0], [0.0]])
        y = dyn.update(u)
        x_next = getattr(dyn, "state", None)

    print("OK: dynamics stepped once.")
    if x_next is not None:
        print("state sample:", np.squeeze(x_next))

if __name__ == "__main__":
    main()
