import globals


def state_machine():
    time = globals.time
    t_fsm = globals.t_fsm
    fsm = globals.fsm

    fsm_stand = globals.params.fsm_stand
    fsm_stance = globals.params.fsm_stance
    fsm_swing = globals.params.fsm_swing

    t_stand = globals.params.t_stand
    t_step = globals.params.t_step

    for leg_no in range(4):
        if (time >= (globals.t_fsm[leg_no] + t_stand) and globals.fsm[leg_no] == fsm_stand):
            if (leg_no == 0 or leg_no == 3):
                globals.fsm[leg_no] = fsm_swing
                globals.t_fsm[leg_no] = time
            if (leg_no == 1 or leg_no == 2):
                globals.fsm[leg_no] = fsm_stance
                globals.t_fsm[leg_no] = time
        
        if (time > (globals.t_fsm[leg_no] + t_step) and fsm[leg_no] == fsm_swing):
            globals.fsm[leg_no] = fsm_stance
            globals.t_fsm[leg_no] = time
        
        if (time > (globals.t_fsm[leg_no] + t_step) and fsm[leg_no] == fsm_stance):
            globals.fsm[leg_no] = fsm_swing
            globals.t_fsm[leg_no] = time

    print(globals.fsm)