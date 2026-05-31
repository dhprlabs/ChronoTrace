import numpy as np
import sympy as sy
import utility
from types import SimpleNamespace
from spatial_forward_kinematics import spatial_forward_kinematics


def spatial_jacobian_torque(q):
    
    sol, robot = spatial_forward_kinematics(joint_angles=q)
    
    # futilitye origin
    o01 = robot.body[1].H_global[0:3,3]
    o02 = robot.body[2].H_global[0:3,3]
    o03 = robot.body[3].H_global[0:3,3]
    o04 = robot.body[4].H_global[0:3,3]
    o05 = robot.body[5].H_global[0:3,3]
    o06 = robot.body[6].H_global[0:3,3]

    # com
    g1 = (robot.body[1].H_global @ np.append(robot.body[1].ipos, 1))[:3]
    g2 = (robot.body[2].H_global @ np.append(robot.body[2].ipos, 1))[:3]
    g3 = (robot.body[3].H_global @ np.append(robot.body[3].ipos, 1))[:3]
    g4 = (robot.body[4].H_global @ np.append(robot.body[4].ipos, 1))[:3]
    g5 = (robot.body[5].H_global @ np.append(robot.body[5].ipos, 1))[:3]
    g6 = (robot.body[6].H_global @ np.append(robot.body[6].ipos, 1))[:3]

    # joint axis
    n1 = robot.body[1].joint_axis
    n2 = robot.body[2].joint_axis
    n3 = robot.body[3].joint_axis
    n4 = robot.body[4].joint_axis
    n5 = robot.body[5].joint_axis
    n6 = robot.body[6].joint_axis

    # rotation matrix
    R00 = np.eye(3);
    R01 = robot.body[1].H_global[0:3,0:3]
    R02 = robot.body[2].H_global[0:3,0:3]
    R03 = robot.body[3].H_global[0:3,0:3]
    R04 = robot.body[4].H_global[0:3,0:3]
    R05 = robot.body[5].H_global[0:3,0:3]
    R06 = robot.body[6].H_global[0:3,0:3]

    # jacobian
    Jv_g6 = np.column_stack([
        utility.skew_matrix_3d(R00 @ n1) @ (g6-o01), \
        utility.skew_matrix_3d(R01 @ n2) @ (g6-o02), \
        utility.skew_matrix_3d(R02 @ n3) @ (g6-o03), \
        utility.skew_matrix_3d(R03 @ n4) @ (g6-o04), \
        utility.skew_matrix_3d(R04 @ n5) @ (g6-o05), \
        utility.skew_matrix_3d(R05 @ n6) @ (g6-o06) 
    ])

    Jw_g6 = np.column_stack([
        R00 @ n1, \
        R01 @ n2, \
        R02 @ n3, \
        R03 @ n4, \
        R04 @ n5, \
        R05 @ n6,
    ])

    J_g6 = np.vstack([Jv_g6, Jw_g6])

    # jacobian
    Jv_g5 = np.column_stack([
        utility.skew_matrix_3d(R00 @ n1) @ (g5-o01), \
        utility.skew_matrix_3d(R01 @ n2) @ (g5-o02), \
        utility.skew_matrix_3d(R02 @ n3) @ (g5-o03), \
        utility.skew_matrix_3d(R03 @ n4) @ (g5-o04), \
        utility.skew_matrix_3d(R04 @ n5) @ (g5-o05), \
        np.zeros((3,1))
    ])

    Jw_g5 = np.column_stack([
        R00 @ n1, \
        R01 @ n2, \
        R02 @ n3, \
        R03 @ n4, \
        R04 @ n5, \
        np.zeros((3,1))
    ])

    J_g5 = np.vstack([Jv_g5, Jw_g5])

    # jacobian
    Jv_g4 = np.column_stack([
        utility.skew_matrix_3d(R00 @ n1) @ (g4-o01), \
        utility.skew_matrix_3d(R01 @ n2) @ (g4-o02), \
        utility.skew_matrix_3d(R02 @ n3) @ (g4-o03), \
        utility.skew_matrix_3d(R03 @ n4) @ (g4-o04), \
        np.zeros((3,1)),
        np.zeros((3,1))
    ])

    Jw_g4 = np.column_stack([
        R00 @ n1,\
        R01 @ n2,\
        R02 @ n3,\
        R03 @ n4,\
        np.zeros((3,1)),
        np.zeros((3,1))
    ])

    J_g4 = np.vstack([Jv_g4, Jw_g4])


    # jacobian
    Jv_g3 = np.column_stack([
        utility.skew_matrix_3d(R00 @ n1) @ (g3-o01), \
        utility.skew_matrix_3d(R01 @ n2) @ (g3-o02), \
        utility.skew_matrix_3d(R02 @ n3) @ (g3-o03), \
        np.zeros((3,1)),
        np.zeros((3,1)),
        np.zeros((3,1))
    ])

    Jw_g3 = np.column_stack([
        R00 @ n1, \
        R01 @ n2, \
        R02 @ n3, \
        np.zeros((3,1)),
        np.zeros((3,1)),
        np.zeros((3,1))
    ])

    J_g3 = np.vstack([Jv_g3, Jw_g3])

    # jacobian
    Jv_g2 = np.column_stack([
        utility.skew_matrix_3d(R00 @ n1) @ (g2-o01), \
        utility.skew_matrix_3d(R01 @ n2) @ (g2-o02), \
        np.zeros((3,1)),
        np.zeros((3,1)),
        np.zeros((3,1)),
        np.zeros((3,1))
    ])

    Jw_g2 = np.column_stack([
        R00 @ n1, \
        R01 @ n2, \
        np.zeros((3,1)),
        np.zeros((3,1)),
        np.zeros((3,1)),
        np.zeros((3,1))
    ])

    J_g2 = np.vstack([Jv_g2, Jw_g2])

    # jacobian
    Jv_g1 = np.column_stack([
        utility.skew_matrix_3d(R00 @ n1) @ (g1-o01), \
        np.zeros((3,1)),
        np.zeros((3,1)),
        np.zeros((3,1)),
        np.zeros((3,1)),
        np.zeros((3,1))
    ])

    Jw_g1 = np.column_stack([
        R00 @ n1,\
        np.zeros((3,1)), 
        np.zeros((3,1)),
        np.zeros((3,1)),
        np.zeros((3,1)),
        np.zeros((3,1))
    ])

    J_g1 = np.vstack([Jv_g1, Jw_g1])

    # F_g6 = robot.body[6].mass * robot.params.gravity
    # F_g5 = robot.body[5].mass * robot.params.gravity
    # F_g4 = robot.body[4].mass * robot.params.gravity
    # F_g3 = robot.body[3].mass * robot.params.gravity
    # F_g2 = robot.body[2].mass * robot.params.gravity
    # F_g1 = robot.body[1].mass * robot.params.gravity

    # tau = np.matmul(Jv_g1.T, F_g1) + np.matmul(Jv_g2.T, F_g2) + np.matmul(Jv_g3.T, F_g3) + \
    #       np.matmul(Jv_g4.T, F_g4) + np.matmul(Jv_g5.T, F_g5) + np.matmul(Jv_g6.T, F_g6)
    
    return J_g1, J_g2, J_g3, J_g4, J_g5, J_g6
