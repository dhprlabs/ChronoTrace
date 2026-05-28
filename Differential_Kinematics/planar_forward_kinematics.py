import numpy as np
import sympy as sy
from types import SimpleNamespace


l1 = 1.0
l2 = 1.0
l3 = 0.25

params = np.array([l1, l2, l3])

def planar_forward_kinematics(angles, params=params):
    l1, l2, l3 = params
    t1 = angles[0]
    t2 = angles[1]
    t3 = angles[2]

    c1 = np.cos(t1)
    s1 = np.sin(t1)
    c2 = np.cos(t2)
    s2 = np.sin(t2)
    c3 = np.cos(t3)
    s3 = np.sin(t3)


    H01 = np.array([
        [c1, -s1, 0, 0],
        [s1, c1, 0, 0],
        [0, 0, 1, 0],
        [0, 0, 0, 1]
    ])

    H12 = np.array([
        [c2, -s2, 0, l1],
        [s2, c2, 0, 0],
        [0, 0, 1, 0],
        [0, 0, 0, 1]
    ])

    H23 = np.array([
        [c3, -s3, 0, l2],
        [s3, c3, 0, 0],
        [0, 0, 1, 0],
        [0, 0, 0, 1]
    ])

    H02 = H01 @ H12 @ np.array([0, 0, 0, 1])
    H03 = H01 @ H12 @ H23
    
    Q0 = H03 @ np.array([0, 0, 0, 1])
    E3 = np.array([l3, 0, 0, 1])
    E0 = H03 @ E3

    o = np.array([0.0, 0.0])
    p = np.array((H02[0], H02[1]))
    q = np.array((Q0[0], Q0[1]))
    e = np.array((E0[0], E0[1]))

    sol = SimpleNamespace(
        o=o, p=p, q=q, e=e,
        H01=H01, H12=H12, H23=H23
    )

    return sol
