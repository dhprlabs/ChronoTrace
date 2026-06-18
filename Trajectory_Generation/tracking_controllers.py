import mujoco
import numpy as np


def pd_feedforward_control(model, data, q_des, qd_des, qdd_des, Kp, Kd):
    """
    Computes torques using PD Feedforward Control.
    tau = M*qdd_des + (C*qd + g) + Kp*e + Kd*e_dot
    """
    # 1. Read current physical states
    q_curr = data.qpos[:6]
    qd_curr = data.qvel[:6]

    # 2. Calculate Tracking Errors
    e = q_des - q_curr
    e_dot = qd_des - qd_curr

    # 3. Extract dense Mass Matrix M(q)
    M = np.zeros((model.nv, model.nv))
    mujoco.mj_fullM(model, M, data.qM)
    M_arm = M[:6, :6]

    # 4. Extract Bias Forces (Coriolis + Gravity)
    # MuJoCo pre-calculates this during mj_forward
    bias_forces = data.qfrc_bias[:6]

    # 5. Compute Control Torques
    tau_ff = M_arm @ qdd_des + bias_forces
    tau_fb = (Kp @ e) + (Kd @ e_dot)
    
    return tau_ff + tau_fb


def inverse_dynamics_control(model, data, q_des, qd_des, qdd_des, Kp, Kd):
    """
    Computes torques using Inverse Dynamics / Computed Torque Control.
    tau = M * (qdd_des + Kp*e + Kd*e_dot) + (C*qd + g)
    """
    # 1. Read current physical states
    q_curr = data.qpos[:6]
    qd_curr = data.qvel[:6]

    # 2. Calculate Tracking Errors
    e = q_des - q_curr
    e_dot = qd_des - qd_curr

    # 3. Extract dense Mass Matrix M(q)
    M = np.zeros((model.nv, model.nv))
    mujoco.mj_fullM(model, M, data.qM)
    M_arm = M[:6, :6]

    # 4. Extract Bias Forces (Coriolis + Gravity)
    bias_forces = data.qfrc_bias[:6]

    # 5. Compute Virtual Acceleration (u)
    u = qdd_des + (Kp @ e) + (Kd @ e_dot)

    # 6. Apply Inverse Dynamics Law
    tau = (M_arm @ u) + bias_forces
    
    return tau