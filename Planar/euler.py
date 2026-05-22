import sympy as sy
import numpy as np

from Planar_Fk.rotation import rotation


phi, theta, psi = sy.symbols('phi theta psi', real=True)

Rx = rotation(sy.pi / 2, 1)
Ry = rotation(sy.pi / 2, 2)
Rz = rotation(0, 3)

R = Rx * Ry * Rz
sy.pprint(R)