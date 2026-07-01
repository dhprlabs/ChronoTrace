import globals


def state_machine():
    time = globals.time
    fsm_stand = globals.params.fsm_stand
    fsm_stance = globals.params.fsm_stance
    fsm_swing = globals.params.fsm_swing
    t_stand = globals.params.t_stand
    t_step = globals.params.t_step

    for leg_no in range(4):
        if (time >= (globals.t_fsm[leg_no] + t_stand) and globals.fsm[leg_no] == fsm_stand):
            globals.t_fsm[leg_no] = time
            
            if leg_no in [0, 3]:  
                globals.fsm[leg_no] = fsm_stance
            elif leg_no in [1, 2]:                 
                globals.fsm[leg_no] = fsm_swing

        elif (time > (globals.t_fsm[leg_no] + t_step) and globals.fsm[leg_no] == fsm_swing):
            globals.fsm[leg_no] = fsm_stance
            globals.t_fsm[leg_no] = time
        
        elif (time > (globals.t_fsm[leg_no] + t_step) and globals.fsm[leg_no] == fsm_stance):
            globals.fsm[leg_no] = fsm_swing
            globals.t_fsm[leg_no] = time