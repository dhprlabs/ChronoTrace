import numpy as np
import utility
from robot_data import rd


def differential_kinematics(q, u):
    n_bodies = len(rd.bodies)
    
    w_local = [np.zeros(3) for _ in range(n_bodies)]
    v_local = [np.zeros(3) for _ in range(n_bodies)]
    
    base_pos = rd.base[0]["pos"]
    base_quat = rd.base[0]["quat"]
    base_R = utility.quaternion_to_rotation(base_quat)
    
    H_global = np.eye(4)
    H_global[:3, :3] = base_R
    H_global[:3, 3] = base_pos

    for i in range(n_bodies):
        pos = rd.bodies[i]["pos"]
        quat = rd.bodies[i]["quat"]
        j_axis = rd.bodies[i]["joint_axis"]
        
        j_angle = q[i]
        j_vel = u[i]
        j_axis_id = np.argmax(j_axis)

        R_body = utility.quaternion_to_rotation(quat) @ utility.rotation_matrix(j_axis_id, j_angle)
        O_body = rd.bodies[i]["pos"].reshape(3, 1)

        H_body = np.block([
            [R_body, O_body],
            [np.zeros((1, 3)), 1]
        ])

        R_local = H_body[:3, :3]
        H_global = H_global @ H_body

        w_parent = w_local[i-1] if i > 0 else np.zeros(3)
        v_parent = v_local[i-1] if i > 0 else np.zeros(3)

        w_parent_cross_o = np.cross(w_parent, pos)
        w_local_propagated = R_local.T @ w_parent
        v_local_propagated = R_local.T @ (v_parent + w_parent_cross_o)

        w_local[i] = w_local_propagated + (j_vel * np.array(j_axis))
        v_local[i] = v_local_propagated 

    R_global_ee = H_global[:3, :3]
    ee_pos_local = rd.end_effector[0]["pos"]
    
    w_ee_local = w_local[-1]
    v_ee_local = v_local[-1] + np.cross(w_local[-1], ee_pos_local)
    
    ve = R_global_ee @ v_ee_local
    we = R_global_ee @ w_ee_local

    return (ve, we)