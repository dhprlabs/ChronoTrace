import numpy as np 
import utility
from forward_kinematics import forward_kinematics


def inverse_kinematics(joint_angles, target_ee):
    # calculating the current pos and rot using fk
    sol, _ = forward_kinematics(joint_angles)
    
    # unpacking target pos and rot
    x_t = target_ee[0]
    y_t = target_ee[1]
    z_t = target_ee[2]
    phi_t = target_ee[3]
    theta_t = target_ee[4]
    psi_t = target_ee[5]
    
    # converting target rot (euler) into quaternion
    r_t = np.array([phi_t, theta_t, psi_t])
    quat_t = utility.euler2quat(r_t)
    
    x = sol.end_eff_pos[0]
    y = sol.end_eff_pos[1]
    z = sol.end_eff_pos[2]

    # converting current rot (rotation matrix) into quaternion
    quat = utility.rotation2quat(sol.end_eff_rot)
    
    # returning the error
    return x-x_t, y-y_t, z-z_t, quat[1]-quat_t[1], quat[2]-quat_t[2], quat[3]-quat_t[3]