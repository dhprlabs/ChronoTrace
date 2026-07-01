import numpy as np
from leg_forward_kinematics import leg_forward_kinematics


def leg_jacobian(q_leg, leg_no):
    J = np.zeros((6, 3))
    
    if (leg_no == 0 or leg_no == 3):
        w = 0.08505
    elif (leg_no == 1 or leg_no == 2):
        w = -0.08505

    c1, s1 = np.cos(q_leg[0]), np.sin(q_leg[0])
    c2, s2 = np.cos(q_leg[1]), np.sin(q_leg[1])
    l2 = 0.2  

    p_ee = leg_forward_kinematics(q_leg, leg_no)

    p0 = np.array([0.0, 0.0, 0.0])
    z0 = np.array([1.0, 0.0, 0.0])

    p1 = np.array([0.0, w * c1, w * s1])
    z1 = np.array([0.0, c1, s1])

    R_x = np.array([
        [1.0, 0.0, 0.0],
        [0.0, c1, -s1],
        [0.0, s1, c1]
    ])

    p2_relative = np.array([-l2 * s2, 0.0, -l2 * c2])
    p2 = p1 + R_x @ p2_relative
    z2 = z1  

    J[0:3, 0] = np.cross(z0, p_ee - p0)  
    J[3:6, 0] = z0                       

    J[0:3, 1] = np.cross(z1, p_ee - p1)
    J[3:6, 1] = z1

    J[0:3, 2] = np.cross(z2, p_ee - p2)
    J[3:6, 2] = z2

    return J