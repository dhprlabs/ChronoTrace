import numpy as np
from planar_forward_kinematics import planar_forward_kinematics
import utility
from types import SimpleNamespace


l1 = 1.0
l2 = 1.0
l3 = 0.25
params = np.array([l1, l2, l3])


def planar_jacobian(q_dot, params=params):
    k = np.array([0, 0, 1])
    sol = planar_forward_kinematics(angles=q_dot)

    R00 = np.eye(3)
    R01 = sol.H01[:3,:3]
    R02 = sol.H02[:3,:3]
    R03 = sol.H03[:3,:3]
    
    o00 = sol.H01[:3,3]
    o01 = sol.H02[:3,3]
    o02 = sol.H03[:3,3]
    
    e0 = sol.e
    
    j_v = np.column_stack([
        utility.skew_matrix_3d(R00 @ k) @ (e0 - o00), \
        utility.skew_matrix_3d(R01 @ k) @ (e0 - o01), \
        utility.skew_matrix_3d(R02 @ k) @ (e0 - o02)
    ])
    
    j_w = np.column_stack([R00 @ k, R01 @ k, R02 @ k])
    
    j_e = np.vstack([j_v, j_w])
    
    j_sol = SimpleNamespace(
       j_e=j_e
    )
    
    return j_sol