import numpy as np
import matplotlib.pyplot as plt
from massAnimation import massAnimation

# -----------------------------
# Simulation parameters
# -----------------------------
t_start = 0.0      # start time (s)
t_end   = 15.0     # end time (s)
dt      = 0.02     # time step (s)

# Motion parameters
amplitude = 0.8    # amplitude of oscillation
frequency = 0.5    # frequency (Hz) of oscillation

# -----------------------------
# Initialize animation
# -----------------------------
anim = massAnimation()
t = t_start

# -----------------------------
# Simulation loop
# -----------------------------
while t <= t_end:
    # Position input: sine wave
    z = amplitude * np.sin(2 * np.pi * frequency * t)
    
    # State vector: [position; velocity]
    state = np.array([[z], [0.0]])
    
    # Update animation
    anim.update(state)
    plt.pause(0.01)
    
    t += dt

plt.savefig("vtol_screenshot.png", dpi=200)
print("Saved frame to vtol_screenshot.png")
