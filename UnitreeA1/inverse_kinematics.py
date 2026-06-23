import numpy as np
import utility
from scipy.optimize import fsolve
from forward_kinematics import forward_kinematics


def ik_residual(joint_angles, target_pos, target_quat):
    current_pos, current_quat = forward_kinematics(joint_angles)
    pos_error = target_pos - current_pos

    q_curr_inv = utility.quaternion_inverse(current_quat)
    delta_q = utility.quaternion_multiplication(target_quat, q_curr_inv)
    quat_error = delta_q[1:] 

    residual = np.concatenate([pos_error, quat_error])

    return residual


def inverse_kinematics(target_pos, target_quat, initial_guess):
    optimized_angles = fsolve(
        ik_residual, 
        initial_guess, 
        args=(np.array(target_pos), np.array(target_quat)),
        xtol=1e-6
    )

    return optimized_angles