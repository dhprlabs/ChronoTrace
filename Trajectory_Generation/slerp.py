import numpy as np 
import sys
from utility import euler2quat
from lerp import lerp


def slerp(s, t1, t2, q1, q2):
    if (t2 - t1 < 0):
        print("t2 cannot be greater than t1")
        sys.exit(1)

    if (np.linalg.norm(t2 - t1) < 0.1):
        print("(t2 - t1) is too small")
        sys.exit(1)

    if (s < t1):
        t = 0
    elif (s > t2):
        t = 1
    else:
        t = (s - t1) / (t2 - t1)

    q1 = q1 / np.linalg.norm(q1)
    q2 = q2 / np.linalg.norm(q2)

    cos_theta = np.dot(q1, q2)

    if (cos_theta < 0.0):
        q2 = -q2
        cos_theta = -cos_theta

    if (cos_theta > 0.995):
        q, q_dot, q_dot_dot = lerp(s, t1, t2, q1, q2)
        return q, q_dot, q_dot_dot

    theta = np.arccos(cos_theta)
    sin_theta = np.sin(theta)

    q = (1/sin_theta) * (np.sin((1-t)*theta) * q1 + np.sin((t*theta)) * q2)
    q_dot = (1/sin_theta) * (-np.cos((1-t)*theta) * q1 + np.cos(t*theta) * q2) * theta
    q_dot_dot = (1/sin_theta) * (np.sin((1-t)*theta) * q1 + np.sin(t*theta) * q2) * theta * theta

    return q, q_dot, q_dot_dot
