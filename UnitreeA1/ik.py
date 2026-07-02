import numpy as np 
from scipy.optimize import fsolve
from fk import forward_kinematics
 

def calc_error(current_angles, target_pos, leg_no):
    current_pos = forward_kinematics(current_angles, leg_no)
    return target_pos - current_pos

def inverse_kinematics(target_pos, current_angles, leg_no):
    leg_angles = fsolve(calc_error, current_angles, args=(target_pos, leg_no), xtol=1e-6)
    return leg_angles


