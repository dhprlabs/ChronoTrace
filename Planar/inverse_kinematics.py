import numpy as np
import sympy as sy
import matplotlib.pyplot as plt


L1 = 1.0
L2 = 1.0
L3 = 0.25

THETA1 = np.pi / 4
THETA2 = -np.pi / 6
THETA3 = -np.pi / 6

PARAMS = (L1, L2, L3)
ANGLES = np.array([THETA1, THETA2, THETA3])

LIMITS = np.array([
    [0.0, np.pi/2],
    [-np.pi/2, np.pi/2],
    [0.0, np.pi/2]
])

WORKSPACE_X = []
WORKSPACE_Y = []


def create_fk_workspace_points(limits):
    n = 30
    t1 = np.linspace(limits[0,0], limits[0,1], n)
    t2 = np.linspace(limits[1,0], limits[1,1], n)
    t3 = np.linspace(limits[2,0], limits[2,1], n)

    for i in range(0, len(t1)):
        for j in range(0, len(t2)):
            for k in range(0, len(t3)):
                params = (1.0, 1.0, 0.25)
                angles = np.array([t1[i], t2[j], t3[k]])
                e, *_ = fk_using_homogenous_transformations(params, angles)
                WORKSPACE_X.append(e[0])
                WORKSPACE_Y.append(e[1])

    plt.figure(figsize=(8, 6))
    plt.scatter(WORKSPACE_X, WORKSPACE_Y, marker='o', color='yellow')
    plt.show()

def plot_fk(o, p, q, e):
    plt.figure(figsize=(8, 6))
    
    plt.plot([o[0], p[0]], [o[1], p[1]], linewidth=4, color='red')
    plt.plot([p[0], q[0]], [p[1], q[1]], linewidth=4, color='blue')
    plt.plot([q[0], e[0]], [q[1], e[1]], linewidth=4, color='green')

    plt.xlim(-2.5, 2.5)
    plt.ylim(-2.5, 2.5)
    plt.xlabel("x")
    plt.ylabel("y")

    plt.show()

def fk_using_homogenous_transformations(PARAMS, ANGLES):
    l1, l2, l3 = PARAMS
    t1 = ANGLES[0]
    t2 = ANGLES[1]
    t3 = ANGLES[2]

    c1 = np.cos(t1)
    s1 = np.sin(t1)
    c2 = np.cos(t2)
    s2 = np.sin(t2)
    c3 = np.cos(t3)
    s3 = np.sin(t3)


    H01 = np.array([
        [c1, -s1, 0, 0],
        [s1, c1, 0, 0],
        [0, 0, 1, 0],
        [0, 0, 0, 1]
    ])

    H12 = np.array([
        [c2, -s2, 0, l1],
        [s2, c2, 0, 0],
        [0, 0, 1, 0],
        [0, 0, 0, 1]
    ])

    H23 = np.array([
        [c3, -s3, 0, l2],
        [s3, c3, 0, 0],
        [0, 0, 1, 0],
        [0, 0, 0, 1]
    ])

    H02 = H01 @ H12 @ np.array([0, 0, 0, 1])
    H03 = H01 @ H12 @ H23
    
    Q0 = H03 @ np.array([0, 0, 0, 1])
    E3 = np.array([l3, 0, 0, 1])
    E0 = H03 @ E3

    o = np.array([0.0, 0.0])
    p = np.array((H02[0], H02[1]))
    q = np.array((Q0[0], Q0[1]))
    e = np.array((E0[0], E0[1]))

    return (e, o, p, q)


# e, o, p, q = fk_using_homogenous_transformations(PARAMS, ANGLES)
# plot_fk(o, p, q, e)
# create_fk_workspace_points(LIMITS)