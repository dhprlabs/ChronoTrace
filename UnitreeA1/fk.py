import numpy as np 


def forward_kinematics(q, leg_no):
    w = -0.08505 if leg_no in [0, 2] else 0.08505
    l2 = 0.2
    l3 = 0.2
    
    c0, s0 = np.cos(q[0]), np.sin(q[0])
    c1, s1 = np.cos(q[1]), np.sin(q[1])
    c2, s2 = np.cos(q[2]), np.sin(q[2])
    
    T_01 = np.array([
        [1.0, 0.0, 0.0, 0.0],
        [0.0,  c0, -s0, 0.0],
        [0.0,  s0,  c0, 0.0],
        [0.0, 0.0, 0.0, 1.0]
    ])
    
    T_12 = np.array([
        [ c1, 0.0,  s1, 0.0],
        [0.0, 1.0, 0.0,   w],
        [-s1, 0.0,  c1, 0.0],
        [0.0, 0.0, 0.0, 1.0]
    ])
    
    T_23 = np.array([
        [ c2, 0.0,  s2,  0.0],
        [0.0, 1.0, 0.0,  0.0],
        [-s2, 0.0,  c2,  -l2],
        [0.0, 0.0, 0.0,  1.0]
    ])
    
    T_3E = np.array([
        [1.0, 0.0, 0.0,  0.0],
        [0.0, 1.0, 0.0,  0.0],
        [0.0, 0.0, 1.0,  -l3],
        [0.0, 0.0, 0.0,  1.0]
    ])
    
    T_robot = T_01 @ T_12 @ T_23 @ T_3E
    foot_position = T_robot[0:3, 3]
    
    return foot_position
