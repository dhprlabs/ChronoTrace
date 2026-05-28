import numpy as np
from planar_differential_kinematics import planar_differential_kinematics
import utility
from types import SimpleNamespace


def planar_differential_kinematics(q, u):
    k = np.array([0, 0, 1])
    sol = planar_differential_kinematics(angles=q)

    w00 = np.array([0, 0, 0])
    v00 = np.array([0, 0, 0])

    R01 = sol.H01[:3, :3]
    r01 = sol.H01[:3, 3]  
    w11 = R01.T @ w00 + u[0] * k
    v11 = R01.T @ (v00 + utility.skew_matrix_2d(w00) @ r01)

    R12 = sol.H12[:3, :3]
    r12 = sol.H12[:3, 3]  
    w22 = R12.T @ w11 + u[1] * k
    v22 = R12.T @ (v11 + utility.skew_matrix_2d(w11) @ r12)

    R23 = sol.H23[:3, :3]
    r23 = sol.H23[:3, 3]  
    w33 = R23.T @ w22 + u[2] * k
    v33 = R23.T @ (v22 + utility.skew_matrix_2d(w22) @ r23)
    
    R02 = R01 @ R12
    R03 = R01 @ R12 @ R23
    
    w1 = R01 @ w11
    w2 = R02 @ w22
    w3 = R03 @ w33
    v1 = R01 @ v11
    v2 = R02 @ v22
    v3 = R03 @ v33
    
    e3 = np.array([0.25, 0, 0])
    ve33 = v33 + utility.skew_matrix_2d(w33) @ e3
    we33 = w33
    
    ve3 = R03 @ ve33
    we3 = R03 @ we33
    
    d_sol = SimpleNamespace(
        v1=v1, v2=v2, v3=v3, 
        w1=w1, w2=w2, w3=w3,
        ve=ve3, we=we3
    )
    
    return d_sol