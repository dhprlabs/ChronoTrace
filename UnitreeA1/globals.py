from parameters import params
import numpy as np


def init():
    global time, fsm, t_fsm
    global lz_i, lz_f, lx_ref, ly_ref, lz_ref
    global q_d, u_d 
    global q, u
    global torque
    global mass
    global gravity

    time = 0.0
    fsm = np.array([params.fsm_stand] * 4)
    t_fsm = np.zeros(4)
    
    lz_i = np.ones(4) * params.lz_0
    lz_f = np.ones(4) * params.lz_0
    
    lx_ref = np.zeros(4)
    ly_ref = np.zeros(4)
    lz_ref = np.ones(4) * params.lz_0

    q_d, u_d = (np.zeros(12) for _ in range(2))
    q, u = (np.zeros(12) for _ in range(2))
    torque = np.zeros(12)

    mass = 4.713
    gravity = 9.81