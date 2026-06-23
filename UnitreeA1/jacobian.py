import numpy as np
import utility
from robot_data import rd

def calculate_jacobian_matrix(q):
    J = np.zeros((6, 6))
    
    H_curr = np.eye(4)
    H_curr[:3, :3] = utility.quaternion_to_rotation(rd.base[0]["quat"])
    H_curr[:3, 3] = rd.base[0]["pos"]
    
    global_joint_positions = []
    global_joint_axes = []
    
    for i in range(len(rd.bodies)):
        pos = rd.bodies[i]["pos"]
        quat = rd.bodies[i]["quat"]
        j_axis = np.array(rd.bodies[i]["joint_axis"])
        j_angle = q[i]
        j_axis_idx = np.argmax(j_axis)
        
        H_static = np.eye(4)
        H_static[:3, :3] = utility.quaternion_to_rotation(quat)
        H_static[:3, 3] = pos
        
        H_joint_base = H_curr @ H_static
        
        z_i_global = H_joint_base[:3, :3] @ j_axis
        p_i_global = H_joint_base[:3, 3]
        
        global_joint_axes.append(z_i_global)
        global_joint_positions.append(p_i_global)
        
        c = np.cos(j_angle)
        s = np.sin(j_angle)
        if j_axis_idx == 0:    # X-Axis
            R_j = np.array([[1, 0, 0], [0, c, -s], [0, s, c]])
        elif j_axis_idx == 1:  # Y-Axis
            R_j = np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])
        else:                  # Z-Axis
            R_j = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
            
        H_dynamic_rotation = np.eye(4)
        H_dynamic_rotation[:3, :3] = R_j
        H_curr = H_joint_base @ H_dynamic_rotation
        
    R_global_ee = H_curr[:3, :3]
    ee_pos_local = rd.end_effector[0]["pos"]
    p_ee_global = (R_global_ee @ ee_pos_local.reshape(3, 1)).flatten() + H_curr[:3, 3]
    
    for i in range(len(rd.bodies)):
        z_i = global_joint_axes[i]
        p_i = global_joint_positions[i]
        r_i = p_ee_global - p_i
        J[3:6, i] = z_i
        J[0:3, i] = np.cross(z_i, r_i)
        

    return J