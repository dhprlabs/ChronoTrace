import numpy as np
import pprint

def rotation_matrix(raxis, rangle):
    s = np.sin(rangle)
    c = np.cos(rangle)

    if (raxis == 0):
        R = np.array([
            [1, 0, 0], 
            [0, c, -s],
            [0, s, c]
        ])
    elif (raxis == 1):
        R = np.array([
            [c, 0, s], 
            [0, 1, 0],
            [-s, 0, c]
        ])
    elif (raxis == 2):
        R = np.array([
            [c, -s, 0], 
            [s, c, 0],
            [0, 0, 1]
        ])

    return R


def euler_xyz(phi, theta, psi):
    s_phi = np.sin(phi)
    c_phi = np.cos(phi)

    s_theta = np.sin(theta)
    c_theta = np.cos(theta)

    s_psi = np.sin(psi)
    c_psi = np.cos(psi)

    Rx = np.array([
        [1, 0, 0], 
        [0, c_phi, -s_phi],
        [0, s_phi, c_phi]
    ])

    Ry = np.array([
        [c_theta, 0, s_theta], 
        [0, 1, 0],
        [-s_theta, 0, c_theta]
    ])

    Rz = np.array([
        [c_psi, -s_psi, 0], 
        [s_psi, c_psi, 0],
        [0, 0, 1]
    ])

    Rt = Rx @ Ry @ Rz

    return Rt


def rotation_to_euler_xyz(R):
    if abs(R[0, 2]) < 1.0 - 1e-7:
        theta = np.arcsin(R[0, 2])
        phi = np.arctan2(-R[1, 2], R[2, 2])
        psi = np.arctan2(-R[0, 1], R[0, 0])
    else:
        phi = 0.0  
        if R[0, 2] > 0:
            theta = np.pi / 2
            psi = np.arctan2(R[1, 0], R[1, 1])
        else:
            theta = -np.pi / 2
            psi = -np.arctan2(R[1, 0], R[1, 1])

    return phi, theta, psi


def quaternion_inverse(q):
    q_inverse = np.array([q[0], -q[1], -q[2], -q[3]])
    norm_q = np.sum(q ** 2)

    return q_inverse / norm_q


def quaternion_normalization(q):
    return q / np.linalg.norm(q)


def quaternion_multiplication(q1, q2):
    dp = np.dot(q1[1:], q2[1:])
    cp = np.cross(q1[1:], q2[1:])
    w = (q1[0] * q2[0]) - (dp)
    vx = (q1[0] * q2[1]) + (q1[1] * q2[0]) + (cp[0])
    vy = (q1[0] * q2[2]) + (q1[2] * q2[0]) + (cp[1])
    vz = (q1[0] * q2[3]) + (q1[3] * q2[0]) + (cp[2])

    return np.array([w, vx, vy, vz])


def quaternion_to_rotation(q):
    q0, q1, q2, q3 = q

    R = np.array([
        [q0**2 + q1**2 - q2**2 - q3**2, 2 * (q1 * q2 - q0 * q3), 2 * (q1 * q3 + q0 * q2)],
        [2 * (q1 * q2 + q0 * q3), q0**2 - q1**2 + q2**2 - q3**2, 2 * (q2 * q3 - q0 * q1)],
        [2 * (q1 * q3 - q0 * q2), 2 * (q2 * q3 + q0 * q1), q0**2 - q1**2 - q2**2 + q3**2]
    ])

    return R


def rotation_to_quaternion(R):
    q0 = np.sqrt(1 + R[0, 0] + R[1, 1] + R[2, 2]) / 2
    q1 = np.sqrt(1 + R[0, 0] - R[1, 1] - R[2, 2]) / 2
    q2 = np.sqrt(1 - R[0, 0] + R[1, 1] - R[2, 2]) / 2
    q3 = np.sqrt(1 - R[0, 0] - R[1, 1] + R[2, 2]) / 2

    if q0 >= max(q1, q2, q3):  # q0 is largest
        q1 = (R[2, 1] - R[1, 2]) / (4 * q0)
        q2 = (R[0, 2] - R[2, 0]) / (4 * q0)
        q3 = (R[1, 0] - R[0, 1]) / (4 * q0)
    elif q1 >= max(q0, q2, q3):  # q1 is largest
        q0 = (R[2, 1] - R[1, 2]) / (4 * q1)
        q2 = (R[1, 0] + R[0, 1]) / (4 * q1)
        q3 = (R[0, 2] + R[2, 0]) / (4 * q1)
    elif q2 >= max(q0, q1, q3):  # q2 is largest
        q0 = (R[0, 2] - R[2, 0]) / (4 * q2)
        q1 = (R[1, 0] + R[0, 1]) / (4 * q2)
        q3 = (R[2, 1] + R[1, 2]) / (4 * q2)
    else:  # q3 is largest
        q0 = (R[1, 0] - R[0, 1]) / (4 * q3)
        q1 = (R[0, 2] + R[2, 0]) / (4 * q3)
        q2 = (R[2, 1] + R[1, 2]) / (4 * q3)

    return np.array([q0, q1, q2, q3])


# H = homogenous_transformations(0, 0, [0, 0, 0])
# pprint.pprint(H)
# print("="*161)

# Rt = euler_xyz(np.pi/2, 0, 0)
# pprint.pprint(Rt)
# print("="*161)

# phi, theta, psi = rotation_to_euler_xyz(Rt)
# pprint.pprint(np.array([phi, theta, psi]))
# print("="*161)

# q1 = np.array([0.7071, 0.7071, 0, 0])
# q2 = np.array([0.7071, 0, 0.7071, 0])

# q1_inverse = quaternion_inverse(q1)
# pprint.pprint(q1_inverse)
# print("="*161)

# qf = quaternion_multiplication(q1, q2)
# pprint.pprint(qf)
# print("="*161)

# R1 = quaternion_to_rotation(q1)
# pprint.pprint(R1)
# print("="*161)

# q1 = rotation_to_quaternion(R1)
# pprint.pprint(q1)
# print("="*161)