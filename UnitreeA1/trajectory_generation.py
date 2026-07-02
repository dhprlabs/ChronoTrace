import numpy as np 


def quintic_trajectory(q0, qf, tf, t, v0=None, vf=None, a0=None, af=None):
    q0 = np.asarray(q0, dtype=np.float64)
    qf = np.asarray(qf, dtype=np.float64)
    v0 = np.zeros_like(q0) if v0 is None else np.asarray(v0, dtype=np.float64)
    vf = np.zeros_like(qf) if vf is None else np.asarray(vf, dtype=np.float64)
    a0 = np.zeros_like(q0) if a0 is None else np.asarray(a0, dtype=np.float64)
    af = np.zeros_like(qf) if af is None else np.asarray(af, dtype=np.float64)
    
    if t <= 0:
        return q0, v0, a0
    if t >= tf:
        return qf, vf, af
        
    c0 = q0
    c1 = v0
    c2 = 0.5 * a0
    c3 = (20 * (qf - q0) - (8 * vf + 12 * v0) * tf - (3 * a0 - af) * (tf ** 2)) / (2 * (tf ** 3))
    c4 = (30 * (q0 - qf) + (14 * vf + 16 * v0) * tf + (3 * a0 - 2 * af) * (tf ** 2)) / (2 * (tf ** 4))
    c5 = (12 * (qf - q0) - 6 * (vf + v0) * tf - (a0 - af) * (tf ** 2)) / (2 * (tf ** 5))
    
    q = c0 + c1*t + c2*(t**2) + c3*(t**3) + c4*(t**4) + c5*(t**5)
    q_dot = c1 + 2*c2*t + 3*c3*(t**2) + 4*c4*(t**3) + 5*c5*(t**4)
    q_ddot = 2*c2 + 6*c3*t + 12*c4*(t**2) + 20*c5*(t**3)
    
    return q, q_dot, q_ddot



