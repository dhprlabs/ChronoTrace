import numpy as np
import sympy as sy
import utility
from types import SimpleNamespace
from robot_data import robot
from spatial_forward_kinematics import spatial_forward_kinematics


def spatial_differential_kinematics(q, u):
    _, robot = spatial_forward_kinematics(joint_angles=q)
    
    for i in range(1, len(robot.body)+1):                                   
        joint_axis = robot.body[i].joint_axis
        o = robot.body[i].o_local
        r = robot.body[i].R_local
        R = robot.body[i].H_global[:3,:3]
        
        if (i == 1):
            w00 = np.array([0, 0, 0])
            v00 = np.array([0, 0, 0])
            robot.body[i].w_local = r.T @ w00 + u[i-1] * joint_axis
            robot.body[i].v_local = r.T @ (v00 + utility.skew_matrix_3d(w00) @ o)
        else:
            robot.body[i].w_local = r.T @ robot.body[i-1].w_local + u[i-1] * joint_axis
            robot.body[i].v_local = r.T @ (robot.body[i-1].v_local + utility.skew_matrix_3d(robot.body[i-1].w_local) @ o)

        robot.body[i].w_global = R @ robot.body[i].w_local
        robot.body[i].v_global = R @ robot.body[i].v_local 
         
    ee_pos_local = robot.params.end_eff_pos_local
    ee_v_local = robot.body[6].v_local + (utility.skew_matrix_3d(robot.body[6].w_local) @ ee_pos_local)
    ee_v_global = robot.body[6].H_global[:3, :3] @ ee_v_local
    ee_w_global = robot.body[6].w_global

    d_sol = SimpleNamespace(
        ee_v_global=np.array([ee_v_global]),
        ee_w_global=np.array([ee_w_global])
    )

    return d_sol