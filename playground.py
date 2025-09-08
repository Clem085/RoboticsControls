
#%%

import numpy as np
import control as ctrl
from control import *
from control.matlab import *
from sympy import *
import matplotlib.pyplot as plt
from sympy.physics.vector import dynamicsymbols
from sympy.physics.vector.printing import vlatex
from IPython.display import Math, display

init_printing()

def dotprint(expr):
    display(Math(vlatex(expr)))

m, l, J, t = symbols('m l J t')
theta = dynamicsymbols('theta')

q = Matrix([[theta]])
qdot = diff(q, t)

p = Matrix([[l/2*cos(theta)],
            [l/2*sin(theta)],
            [0]])

v = diff(p, t)

dotprint(p)
dotprint(v)

omega = Matrix([[0],
                [0],
                [diff(theta, t)]])
dotprint(omega)

K = 1/2*m*(v.T*v) + 1/2*(omega.T*J*omega)
dotprint(K)
K = simplify(K[0])
dotprint(K)
K = simplify(K.subs('J', 1/12*m*l**2))
dotprint(K)

# %%