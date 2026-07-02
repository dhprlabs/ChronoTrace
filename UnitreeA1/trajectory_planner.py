import numpy as np
from trajectory_generation import quintic_trajectory
import var

def compute_leg_target(leg_no, sim_time):
    t_start = var.t_fsm_start[leg_no]
    t_step = var.t_step
    t_half = var.t_half
    lz_0 = var.lz
    h_c = var.h
    w_offset = var.lateral_offsets[leg_no]
    
    t_cycle = sim_time % t_step
    phi = np.clip((sim_time - t_start) / t_step, 0.0, 1.0)
    
    # =========================================================================
    # MODE 1: PURE VERTICAL SQUAT (Z-ONLY)
    # =========================================================================
    if var.control_mode == "Z_ONLY":
        x_target = 0.0
        y_target = w_offset
        t_squat_cycle = sim_time % (2 * t_step)
        
        if t_squat_cycle <= t_step:
            s, _, _ = quintic_trajectory(0.0, 1.0, t_step, t_squat_cycle)
            z_target = lz_0 - (s * 0.06)
        else:
            s, _, _ = quintic_trajectory(1.0, 0.0, t_step, t_squat_cycle - t_step)
            z_target = lz_0 - (s * 0.06)
            
        return np.array([x_target, y_target, z_target])

    # =========================================================================
    # MODES 2 & 3: DIRECTIONAL LOCOMOTION (X, Y, OR COMBINED TROT)
    # =========================================================================
    S_x = var.v_x * t_step if var.control_mode in ["X_ONLY", "TROT_3D"] else 0.0
    S_y = var.v_y * t_step if var.control_mode in ["Y_ONLY", "TROT_3D"] else 0.0
    is_swing = (var.fsm_states[leg_no] == 2)
    
    if is_swing:
        
        local_time = sim_time - t_start
        
        if local_time <= t_half:
            s_z, _, _ = quintic_trajectory(0.0, 1.0, t_half, local_time)
            z_target = lz_0 + (s_z * h_c)
        else:
            s_z, _, _ = quintic_trajectory(1.0, 0.0, t_half, local_time - t_half)
            z_target = lz_0 + (s_z * h_c)
            
        s_xy, _, _ = quintic_trajectory(0.0, 1.0, t_step, local_time)
        x_target = -S_x / 2.0 + (s_xy * S_x)
        y_target = w_offset - S_y / 2.0 + (s_xy * S_y)
        
    else:
        z_target = lz_0
        x_target = S_x / 2.0 - (phi * S_x)
        y_target = w_offset + S_y / 2.0 - (phi * S_y)
        
    return np.array([x_target, y_target, z_target])
