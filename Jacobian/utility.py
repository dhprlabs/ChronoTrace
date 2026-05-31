import numpy as np


def rotation(angle, axis):
    c = np.cos(angle) 
    s = np.sin(angle)
    
    if axis == 0:   
        return np.array([
            [1, 0, 0],
            [0, c, -s],
            [0, s, c]
        ])
    elif axis == 1:  
        return np.array([
            [c, 0, s],
            [0, 1, 0],
            [-s, 0, c]
        ])
    elif axis == 2:
        return np.array([
            [c, -s, 0],
            [s, c, 0],
            [0, 0, 1]
        ])
    else:
        raise ValueError("Axis must be 0 (x), 1 (y), or 2 (z).")


def quat2rotation(q):
    q0, q1, q2, q3 = q

    R = np.array([
        [q0**2 + q1**2 - q2**2 - q3**2, 2 * (q1 * q2 - q0 * q3), 2 * (q1 * q3 + q0 * q2)],
        [2 * (q1 * q2 + q0 * q3), q0**2 - q1**2 + q2**2 - q3**2, 2 * (q2 * q3 - q0 * q1)],
        [2 * (q1 * q3 - q0 * q2), 2 * (q2 * q3 + q0 * q1), q0**2 - q1**2 - q2**2 + q3**2]
    ])

    return R


def rotation2quat(R):
    assert R.shape == (3, 3), "Input must be a 3x3 rotation matrix."

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


def quat2axisangle(quat):
    # q0 = cos(angle/2)
    # [qx,qy,qz] = sin(angle/2)*axis
 
    # angle = 2*acos(q0)
    # axis = (1/sin(angle/2))*[qx,qy,qz]

    q0 = quat[0]
    qx = quat[1]
    qy = quat[2]
    qz = quat[3]

    angle = 2*np.arccos(q0)
    sin_half_theta = np.sin(angle/2);

    if (sin_half_theta < 1e-6): 
        axis = np.array([1,0,0]) 
    else:
        axis = (1/sin_half_theta)*np.array([qx,qy,qz])

    return axis,angle


def euler2rotation(euler):
    Rx = rotation(euler[0], 0)
    Ry = rotation(euler[1], 1)
    Rz = rotation(euler[2], 2)
    
    R = Rx @ Ry
    R = R @ Rz
    
    return R


def rotation2euler(R):
    r13 = R[0,2]
    theta = np.arcsin(r13)
    cos_theta = np.cos(theta)

    r12 = R[0,1]
    psi = np.arcsin(-r12/cos_theta) 

    r23 = R[1,2]
    phi = np.arcsin(-r23/cos_theta) 

    return np.array([phi, theta, psi])


def quat2euler(q):
    R = quat2rotation(q);
    euler = rotation2euler(R)

    return euler


def euler2quat(euler):
    R = euler2rotation(euler)
    quat = rotation2quat(R)
    
    return quat


def quat_product(q, p):
    q0 = q[0]
    p0 = p[0]
    p_vec = p[1:4].copy()
    q_vec = q[1:4].copy()
    
    qp0 = q0*p0-np.dot(p_vec, q_vec)
    qp_vec = q0*p_vec + p0*q_vec + np.cross(q_vec, p_vec)
    qp = np.array([qp0, qp_vec[0], qp_vec[1], qp_vec[2]])
    
    return qp


def quat_normalize(q):
    assert q.shape[-1] == 4
    norm = np.linalg.norm(q, axis=-1, keepdims=True)
    
    return q / norm  


def skew_matrix_2d(w):
    w_z = w[2]

    return np.array([[0, -w_z, 0], [w_z, 0, 0], [0, 0, 0]])


def skew_matrix_3d(w):
    w_x = w[0]
    w_y = w[1]
    w_z = w[2]

    return np.array([[0, -w_z, w_y], [w_z, 0, -w_x], [-w_y, w_x, 0]])