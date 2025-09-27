#%%
# these are libraries and functions that will help us with the calculations for h2
import sympy as sp
from sympy.physics.vector import dynamicsymbols
from sympy.physics.vector.printing import vlatex
from IPython.display import Math, display


#%%
# these functions are defined in our helper functions file in the public repository
from EL_helper_functions import rotx, roty, rotz, calc_omega, find_coeffs

# Defining necessary symbols and variables to use in the calculations
t, ell_1, ell_2, ell_3x, ell_3y, ell_3z, m1, m2, m3= sp.symbols('t, ell_1, ell_2, ell_3x, ell_3y, ell_3z, m1, m2, m3')

theta = dynamicsymbols('theta') # we have to tell SymPy that these are functions of time
psi = dynamicsymbols('psi')
phi = dynamicsymbols('phi')

q = sp.Matrix([[phi], [theta], [psi]])
q_dot = q.diff(t)

# TODO define the position of each mass in body frame, then rotate
# it into the world or inertial frame. 
# p1_in_b = 
# p1_in_w = 

# p2_in_2 = 
# p2_in_w = 

# p3_in_1 = 
# p3_in_w = 


#%% 
# TODO, take the time derivative of the position vectors to get the linear velocity,
# then use the "find_coeffs" function to calculate the "V_i" matrices
# v1_in_w = 
# V1 = 

# v2_in_w = 
# V2 = 

# v3_in_w = 
# V3 = 


# now calculate the rotation matrices for each rigid body
# TODO use the "rotx", "roty", and "rotz" functions to calculate the rotation matrices
# for each rigid body
#R1 =  #rotation to body 1
#R2 =  #rotation to body 2
#R3 =  #rotation to body 3

# we can use the rotation matrices to calculate the angular velocity of each rigid body
# TODO use the "calc_omega" function to calculate the angular velocity of each rigid body 
# omega_1 = 
# omega_2 = 
# omega_3 =

# TODO use the "find_coeffs" function to calculate the "W_i" matrices
# W1 =
# W2 = 
# W3 = 


#%%
# TODO define the inertia tensors for each rigid body
# J1 = sp.zeros(3,3)

# J2 = sp.zeros(3,3)

# J3 = sp.zeros(3,3)



# TODO calculate M using the masses and the V, W, R, and J matrices
M = sp.zeros(3,3)
#M = M + 


# simplifying functions and displaying the result
M = sp.simplify(M)
M = sp.trigsimp(M)
M = sp.expand_trig(M)
display(Math(vlatex(M)))