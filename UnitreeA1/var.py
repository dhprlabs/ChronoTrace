import numpy as np


def init():
    global lz, h, t_step, t_half
    global v_x, v_y, control_mode
    global fsm_states, t_fsm_start
    global lateral_offsets
   
    lz = -0.249
    h = 0.075
    t_step = 0.4
    t_half = t_step * 0.5

    v_x, v_y = 0.2, 0.1
    control_mode = "X_ONLY"

    fsm_states = np.array([1, 2, 2, 1])
    t_fsm_start = np.zeros(4)

    lateral_offsets = np.array([-0.085, 0.085, -0.085, 0.085])
