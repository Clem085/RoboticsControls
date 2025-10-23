"""
Homework D.8: Mass-Spring-Damper PD Control Simulation
Part (a): tr=2s, zeta=0.7
Part (b): Tuned for saturation
"""

import matplotlib.pyplot as plt
import numpy as np
import massParam as P
from signalGenerator import signalGenerator
from massAnimation import massAnimation
from dataPlotter import dataPlotter
from massDynamics import massDynamics
from ctrlPD import ctrlPD, ctrlPD_saturated

mass = massDynamics()
controller_a = ctrlPD()
controller_b = ctrlPD_saturated()
reference = signalGenerator(amplitude=1.0, frequency=0.02)
animation = massAnimation()
dataPlot = dataPlotter()

print("\n" + "="*60)
print("RUNNING HOMEWORK D.8 SIMULATIONS")
print("="*60)

print("\n>> Part (a): Testing PD controller with tr=2s, zeta=0.7")
print("   Step input: 1.0 meter")

mass_a = massDynamics()
t = P.t_start
max_force_a = 0.0
saturation_occurred_a = False

time_history_a = []
z_history_a = []
z_ref_history_a = []
force_history_a = []

while t < P.t_end:
    t_next_plot = t + P.t_plot
    
    while t < t_next_plot:
        z_r = reference.step(t)
        F = controller_a.update(z_r, mass_a.state)
        
        if abs(F) > max_force_a:
            max_force_a = abs(F)
        
        if abs(F) > P.F_max:
            saturation_occurred_a = True
        
        time_history_a.append(t)
        z_history_a.append(mass_a.state[0][0])
        z_ref_history_a.append(z_r)
        force_history_a.append(F)
        
        mass_a.update(F)
        t += P.Ts
    
    animation.update(mass_a.state)
    dataPlot.update(t, mass_a.state, F, z_r)
    plt.pause(0.0001)

print("   Maximum force: F_max = {:.4f} N".format(max_force_a))
print("   Force limit: {} N".format(P.F_max))
if saturation_occurred_a:
    print("   Status: SATURATION OCCURRED")
else:
    print("   Status: No saturation")

print('\nPart (a) complete. Press key to continue to Part (b)...')
plt.waitforbuttonpress()
plt.close('all')

print("\n>> Part (b): Testing tuned PD controller (F_max = 6N)")
print("   Step input: 1.0 meter")

mass_b = massDynamics()
animation_b = massAnimation()
dataPlot_b = dataPlotter()
t = P.t_start
max_force_b = 0.0
saturation_occurred_b = False

time_history_b = []
z_history_b = []
z_ref_history_b = []
force_history_b = []

while t < P.t_end:
    t_next_plot = t + P.t_plot
    
    while t < t_next_plot:
        z_r = reference.step(t)
        F = controller_b.update(z_r, mass_b.state)
        
        if abs(F) > max_force_b:
            max_force_b = abs(F)
        
        if abs(F) > P.F_max:
            saturation_occurred_b = True
        
        time_history_b.append(t)
        z_history_b.append(mass_b.state[0][0])
        z_ref_history_b.append(z_r)
        force_history_b.append(F)
        
        mass_b.update(F)
        t += P.Ts
    
    animation_b.update(mass_b.state)
    dataPlot_b.update(t, mass_b.state, F, z_r)
    plt.pause(0.0001)

print("   Maximum force: F_max = {:.4f} N".format(max_force_b))
print("   Force limit: {} N".format(P.F_max))
if saturation_occurred_b:
    print("   Status: SATURATION OCCURRED")
else:
    print("   Status: No saturation - tuning successful")

print("\n>> Generating comparison plot...")

fig, axes = plt.subplots(3, 1, figsize=(10, 8))
fig.suptitle('Homework D.8: PD Controller Comparison', fontsize=14, fontweight='bold')

axes[0].plot(time_history_a, z_ref_history_a, 'k--', label='Reference', linewidth=2)
axes[0].plot(time_history_a, z_history_a, 'b-', label='Part (a): tr=2s', linewidth=1.5)
axes[0].plot(time_history_b, z_history_b, 'r-', label='Part (b): tuned', linewidth=1.5)
axes[0].set_ylabel('Position z (m)')
axes[0].set_title('Position Response')
axes[0].grid(True)
axes[0].legend()

zdot_a = np.gradient(z_history_a, P.Ts)
zdot_b = np.gradient(z_history_b, P.Ts)
axes[1].plot(time_history_a, zdot_a, 'b-', label='Part (a): tr=2s', linewidth=1.5)
axes[1].plot(time_history_b, zdot_b, 'r-', label='Part (b): tuned', linewidth=1.5)
axes[1].set_ylabel('Velocity (m/s)')
axes[1].set_title('Velocity Response')
axes[1].grid(True)
axes[1].legend()

axes[2].plot(time_history_a, force_history_a, 'b-', label='Part (a): tr=2s', linewidth=1.5)
axes[2].plot(time_history_b, force_history_b, 'r-', label='Part (b): tuned', linewidth=1.5)
axes[2].axhline(y=P.F_max, color='g', linestyle='--', label='F_max = {}N'.format(P.F_max), linewidth=2)
axes[2].axhline(y=-P.F_max, color='g', linestyle='--', linewidth=2)
axes[2].set_xlabel('Time (s)')
axes[2].set_ylabel('Force (N)')
axes[2].set_title('Control Force (with saturation limits)')
axes[2].grid(True)
axes[2].legend()

plt.tight_layout()

#==============================================================================
# Summary Report
#==============================================================================
print("\n" + "="*60)
print("HOMEWORK D.8 SUMMARY")
print("="*60)
print("\nPart (a): PD Controller Design (tr=2s, zeta=0.7)")
print("  kP = {:.4f}".format(controller_a.kP))
print("  kD = {:.4f}".format(controller_a.kD))
print("  Poles: {:.4f} +/- j{:.4f}".format(controller_a.pole_real, controller_a.pole_imag))
print("  Max force: {:.4f} N".format(max_force_a))
print("  Saturation: {}".format('YES' if saturation_occurred_a else 'NO'))

print("\nPart (b): Tuned PD Controller (F_max=6N constraint)")
print("  kP = {:.4f}".format(controller_b.kP))
print("  kD = {:.4f}".format(controller_b.kD))
print("  Poles: {:.4f} +/- j{:.4f}".format(controller_b.pole_real, controller_b.pole_imag))
print("  Max force: {:.4f} N".format(max_force_b))
print("  Saturation: {}".format('YES' if saturation_occurred_b else 'NO'))

print("\nConclusion:")
if not saturation_occurred_b and max_force_b <= P.F_max:
    print("  Part (b) tuning successful")
    print("  Control input does NOT saturate for 1m step")
else:
    print("  Further tuning may be needed")
    
print("="*60)

print('\nPress key to close all plots and exit')
plt.waitforbuttonpress()
plt.close('all')
