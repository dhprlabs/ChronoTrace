from parameters import params
import numpy as np


def init():
    global time 
    global t_fsm
    global fsm
    
    
    time = 0.0
    t_fsm = np.zeros(shape=(4), dtype=float)
    fsm = np.array([params.fsm_stand, params.fsm_stand, params.fsm_stand, params.fsm_stand])