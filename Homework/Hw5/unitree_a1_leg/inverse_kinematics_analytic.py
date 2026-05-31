import numpy as np

def inverse_kinematics_analytic(X_ref):

    L = 0.2; #thigh and shank length

    lx = X_ref[0];
    ly = X_ref[1];
    lz = X_ref[2];

    #HINT1: You need to compute q_a, q_h, q_k as a function of lx, ly, lz
    q_a = 0
    q_h = 0.9;
    q_k = -1.8;

    q_leg = np.array([q_a,q_h,q_k])

    return q_leg
