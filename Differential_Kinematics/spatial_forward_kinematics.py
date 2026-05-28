import numpy as np
import sympy as sy
from types import SimpleNamespace
from robot_data import robot
import utility


def spatial_forward_kinematics(joint_angles):
    # Compute H_local
    for i in range(1,len(robot.body)+1):                                   
        quat = robot.body[i].quat
        joint_axis = robot.body[i].joint_axis
        axis_id = np.argmax(np.abs(joint_axis))                            
        angle = joint_angles[i-1]                                                    

        R_q = utility.rotation(angle, axis_id)
        robot.body[i].R_local= utility.quat2rotation(quat) @ R_q
        robot.body[i].o_local = robot.body[i].pos

        robot.body[i].H_local = np.block([
            [robot.body[i].R_local, robot.body[i].o_local.reshape(-1, 1)], 
            [np.zeros((1, 3)), 1]
        ])

    # Compute H_global
    base_quat = robot.params.base_quat
    R_base = utility.quat2rotation(base_quat)

    H_base = np.block([
        [R_base, np.zeros((3, 1))],
        [np.zeros((1, 3)), 1]
    ])

    temp = H_base
    
    for i in range(1, len(robot.body)+1):
        robot.body[i].H_global = temp @ robot.body[i].H_local
        temp = robot.body[i].H_global

    # Compute the position of the end-effector
    end_eff_pos_local = robot.params.end_eff_pos_local
    end_eff_quat_local = robot.params.end_eff_quat_local
    R_end_eff = utility.quat2rotation(end_eff_quat_local)

    # end_eff stuff
    end_eff_pos = robot.body[6].H_global @ np.append(end_eff_pos_local, 1)  
    end_eff_pos = end_eff_pos[:3]  
    end_eff_rot = robot.body[6].H_global[:3, :3] @ R_end_eff

    # Define sol
    sol = SimpleNamespace(
        end_eff_pos=end_eff_pos,
        end_eff_rot=end_eff_rot,
    )

    return sol, robot