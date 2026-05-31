import numpy as np
import sympy as sy
import utility
from types import SimpleNamespace
from spatial_forward_kinematics import spatial_forward_kinematics


def spatial_jacobian(q):
    
    sol, robot = spatial_forward_kinematics(joint_angles=q)
    
    je_v = []
    je_w = []
    
    for i in range(0, len(robot.body)):  
        if i == 0:
            k = robot.body[i+1].joint_axis
            o = robot.body[i+1].o_local
            R = np.eye(3)
            je_v.append(np.array(utility.skew_matrix_3d(R @ k) @ (sol.end_eff_pos - o)))
            je_w.append(np.array(R @ k))
        else:                                    
            k = robot.body[i+1].joint_axis
            o = robot.body[i+1].o_local
            R = robot.body[i].R_local
            je_v.append(np.array(utility.skew_matrix_3d(R @ k) @ (sol.end_eff_pos - o)))
            je_w.append(np.array(R @ k))

    je_v = np.column_stack(je_v)
    je_w = np.column_stack(je_w)
    
    je = np.vstack([je_v, je_w])

    return je        
        