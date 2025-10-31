"""
Homework F.8: VTOL PD Control Simulation
Successive loop closure for altitude and lateral control
"""

import sys
sys.path.append('..')
import numpy as np
import VTOLParam as P
from ctrlPD import ctrlPD
from VTOLDynamics import LinearVTOL
from VTOLAnimation import VTOLAnimation
from dataPlotter import dataPlotter
from signalGenerator import signalGenerator

print("=" * 60)
print("HOMEWORK F.8: VTOL PD CONTROL WITH SUCCESSIVE LOOP CLOSURE")
print("=" * 60)
print("\nSystem Parameters:")
print("  Total mass (m):    {:.2f} kg".format(P.mc + 2.0*P.mr))
print("  Inertia (J):       {:.6f} kg*m^2".format(P.Jc + 2.0*P.mr*(P.d**2)))
print("  Damping (mu):      {:.2f} kg/s".format(P.mu))
print("  Arm length (d):    {:.2f} m".format(P.d))
print("  Gravity (g):       {:.2f} m/s^2".format(P.g))
print("  Equilibrium thrust: {:.2f} N".format((P.mc + 2.0*P.mr)*P.g))
print("\nControl Limits:")
print("  Max thrust per motor: {:.1f} N".format(P.max_thrust))
print("  Max total thrust:     {:.1f} N".format(2.0*P.max_thrust))
print("  Max torque:           {:.2f} N*m".format(P.max_thrust*P.d))
print("=" * 60)
print()

vtol = LinearVTOL()
ctrl = ctrlPD()
animation = VTOLAnimation()
dataPlot = dataPlotter()

z_reference = signalGenerator(amplitude=3.0, frequency=0.05, y_offset=0.0)
h_reference = signalGenerator(amplitude=2.0, frequency=0.08, y_offset=5.0)

t = P.t_start
t_next_plot = 0

print("\n" + "=" * 60)
print("STARTING SIMULATION")
print("=" * 60)
print("Simulating from t = {:.1f} to t = {:.1f} seconds".format(P.t_start, P.t_end))
print("Time step: {:.4f} seconds".format(P.Ts))
print("Plot update rate: {:.2f} seconds".format(P.t_plot))
print("=" * 60)
print()
print("Running... (Close animation window to end early)")
print()

while t < P.t_end:
    t_next_plot = t + P.t_plot
    
    while t < t_next_plot:
        z_r = z_reference.square(t)
        h_r = h_reference.square(t)
        
        z = vtol.state[0][0]
        h = vtol.state[1][0]
        theta = vtol.state[2][0]
        zdot = vtol.state[3][0]
        hdot = vtol.state[4][0]
        thetadot = vtol.state[5][0]
        
        F, tau = ctrl.update(z_r, z, h_r, h, theta, zdot, hdot, thetadot)
        
        motor_thrusts = P.mixing @ np.array([[F], [tau]])
        
        y = vtol.update(np.array([[F], [tau]]))
        
        t += P.Ts
    
    animation.update(vtol.state)
    dataPlot.update(
        t,
        vtol.state,
        motor_thrusts,
        z_ref=z_r,
        h_ref=h_r
    )

print("\n" + "=" * 60)
print("SIMULATION COMPLETE")
print("=" * 60)
print("Final time: {:.2f} seconds".format(t))
print("\nFinal State:")
print("  Lateral position (z): {:.3f} m".format(vtol.state[0][0]))
print("  Altitude (h):        {:.3f} m".format(vtol.state[1][0]))
print("  Pitch angle (theta): {:.3f} rad ({:.1f} deg)".format(
    vtol.state[2][0], vtol.state[2][0]*180.0/np.pi))
print("=" * 60)
print("\nClose plot windows to exit.")

input("Press Enter to exit...")
