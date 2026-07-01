import numpy as np
from leg_jacobian import leg_jacobian

def compute_leg_torque(q_leg, u_leg, q_d, u_d, leg_no, fsm_state, fsm_swing, p_curr, p_ref, mass, gravity):
    """
    Computes the 3-element joint torque vector for a single leg.
    Uses Joint PD for Swing, and Jacobian Transpose Force control for Stance.
    """
    # 1. Swing Phase Control: Rigid Joint Tracking
    if fsm_state == fsm_swing:
        kp_joint = 25.0
        kd_joint = 1.5
        torque = kp_joint * (q_d - q_leg) + kd_joint * (u_d - u_leg)
        return torque

    # 2. Stance / Stand Phase Control: Force-Based Projection
    else:
        # Calculate the 6x3 Geometric Jacobian for this leg configuration
        J_full = leg_jacobian(q_leg, leg_no)
        J = J_full[0:3, :] # Extract only the top 3x3 linear velocity rows
        
        # Virtual Cartesian spring stiffness to track horizontal movements
        kp_cart = 150.0
        kd_cart = 5.0
        
        # Feedforward force to balance the trunk mass evenly across stance pairs
        f_gravity = 0.5 * mass * gravity
        
        # Compute the 3D Cartesian force vector
        Fx = kp_cart * (p_ref[0] - p_curr[0]) - kd_cart * (0.0 - p_curr[0])
        Fy = kp_cart * (p_ref[1] - p_curr[1]) - kd_cart * (0.0 - p_curr[1])
        Fz = f_gravity + 80.0 * (p_ref[2] - p_curr[2]) # Added a virtual spring along Z for height stability
        
        F_cart = np.array([Fx, Fy, Fz])
        
        # Map Cartesian forces directly to joint torques: tau = J^T * F
        torque = J.T @ F_cart
        return torque