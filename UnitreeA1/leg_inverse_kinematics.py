import numpy as np
from scipy.optimize import fsolve
from leg_forward_kinematics import leg_forward_kinematics


def leg_ik_residual(joint_angles, target_pos, leg_no):
    current_pos = leg_forward_kinematics(joint_angles, leg_no)
    return target_pos - current_pos


def leg_inverse_kinematics(target_pos, leg_no, initial_guess):
    optimized_angles = fsolve(
        leg_ik_residual,
        initial_guess,
        args=(np.array(target_pos), leg_no),
        xtol=1e-6
    )
    return optimized_angles