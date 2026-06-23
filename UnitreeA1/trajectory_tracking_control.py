import numpy as np
import mujoco as mj


def compute_critically_damped_kv(kp):
    """
    Computes the ideal derivative gains to ensure critical damping.
    Accepts a scalar or an array of Kp values.
    """
    return 2.0 * np.sqrt(kp)

def pd_feedforward_control(model, data, q_d, q_dot_d, q_ddot_d, kp, kv):
    """
    PD + Feedforward Trajectory Controller
    """
    q = data.qpos
    q_dot = data.qvel
    
    M = np.zeros((model.nv, model.nv))
    mj.mj_fullM(model, M, data.qM)
    bias = data.qfrc_bias
    
    tau_feedforward = M @ q_ddot_d + bias
    tau_feedback = kp * (q_d - q) + kv * (q_dot_d - q_dot)
    
    return tau_feedforward + tau_feedback

def inverse_dynamics_control(model, data, q_d, q_dot_d, q_ddot_d, kp, kv):
    """
    Inverse Dynamics (Computed Torque) Controller
    """
    q = data.qpos
    q_dot = data.qvel
    
    M = np.zeros((model.nv, model.nv))
    mj.mj_fullM(model, M, data.qM)
    bias = data.qfrc_bias
    
    a_q = q_ddot_d + kp * (q_d - q) + kv * (q_dot_d - q_dot)
    tau = M @ a_q + bias

    return tau