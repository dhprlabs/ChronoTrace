import numpy as np 
import sys
from utility import euler2quat


def lerp(s, t1, t2, q1, q2):
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

    q = (1 - t) * q1 + (t * q2) 
    q = q / np.linalg.norm(q)
    q_dot = -q1 + q2
    q_dot_dot = np.array([0, 0, 0, 0])

    return q, q_dot, q_dot_dot


# def lerp(s, pi, pf):
#     d = pf - pi
#     return pi + (s * d)


# s = 0.00

# for i in range(0, 500):
#     if (s > 1.0):
#         break
#     else:
#         p = lerp(s, np.array([3,4]), np.array([-2,6]))
#         print(f'{i} : {p}')
#         s += 0.01