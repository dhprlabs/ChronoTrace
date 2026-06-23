import numpy as np
import utility
from robot_data import rd


def forward_kinematics(joint_angles):
    base_quat = rd.base[0]["quat"]
    R_base = utility.quaternion_to_rotation(base_quat)

    H_global = np.block([
        [R_base, np.zeros((3, 1))],
        [np.zeros((1, 3)), 1]
    ])

    for i in range(0, len(rd.bodies)):
        j_axis = rd.bodies[i]["joint_axis"]
        j_axis = np.argmax(j_axis)
        j_angle = joint_angles[i]
        body_quat = rd.bodies[i]["quat"]
        R_body = utility.quaternion_to_rotation(body_quat) @ utility.rotation_matrix(j_axis, j_angle)
        O_body = rd.bodies[i]["pos"].reshape(3, 1)

        H_body = np.block([
            [R_body, O_body],
            [np.zeros((1, 3)), 1]
        ])

        H_global = H_global @ H_body

    ee_quat = rd.end_effector[0]["quat"]
    R_ee = utility.quaternion_to_rotation(ee_quat)
    O_ee = rd.end_effector[0]["pos"].reshape(3, 1)

    H_ee = np.block([
        [R_ee, O_ee],
        [np.zeros((1, 3)), 1]
    ])

    H_global = H_global @ H_ee

    pos = H_global[:3, 3]
    quat = utility.rotation_to_quaternion(H_global[:3, :3])

    return pos, quat