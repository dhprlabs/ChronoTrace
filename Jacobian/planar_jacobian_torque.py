import numpy as np
from planar_forward_kinematics import planar_forward_kinematics
import utility
from types import SimpleNamespace


l1 = 1.0
l2 = 1.0
l3 = 0.25
params = np.array([l1, l2, l3])


def planar_jacobian_torque(q_dot, params=params):
    k = np.array([0, 0, 1])
    sol = planar_forward_kinematics(angles=q_dot)

    R00 = np.eye(3)
    R01 = sol.H01[:3,:3]
    R02 = sol.H02[:3,:3]
    R03 = sol.H03[:3,:3]
    
    o00 = sol.H01[:3,3]
    o01 = sol.H02[:3,3]
    o02 = sol.H03[:3,3]
    
    g2 = sol.H02 @ np.array([0.5 * l2, 0, 0, 1])

    jg2_v = np.column_stack([
        utility.skew_matrix_3d(R00 @ k) @ (g2[:3] - o00), \
        utility.skew_matrix_3d(R01 @ k) @ (g2[:3] - o01), \
        np.zeros(shape=(3,1))
    ])
    
    jg2_w = np.column_stack([R00 @ k, R01 @ k, np.zeros(shape=(3,1))])
    
    jg2 = np.vstack([jg2_v, jg2_w])
    
    jg_sol = SimpleNamespace(
        jg2=jg2
    )
    
    return jg_sol