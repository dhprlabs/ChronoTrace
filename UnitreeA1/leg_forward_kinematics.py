import numpy as np


def leg_forward_kinematics(q, leg_no):
    if (leg_no == 0 or leg_no == 3):
        w = 0.08505
    elif (leg_no == 1 or leg_no == 2):
        w = -0.08505

    c1, s1 = np.cos(q[0]), np.sin(q[0])
    c2, s2 = np.cos(q[1]), np.sin(q[1])
    c3, s3 = np.cos(q[2]), np.sin(q[2])

    l2 = 0.2  
    l3 = 0.2  

    x_limb = -l2 * s2 - l3 * np.sin(q[1] + q[2])
    y_limb = w
    z_limb = -l2 * c2 - l3 * np.cos(q[1] + q[2])

    x = x_limb
    y = y_limb * c1 - z_limb * s1
    z = y_limb * s1 + z_limb * c1

    return np.array([x, y, z])