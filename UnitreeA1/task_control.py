import numpy as np
import scipy.linalg


def _time_scaling(tf, t, profile='quintic'):
    """
    Helper function to generate normalized time scaling factors s(t), s_dot(t), s_ddot(t).
    Maps t in [0, tf] to s in [0, 1].
    """
    if t <= 0:
        return 0.0, 0.0, 0.0
    if t >= tf:
        return 1.0, 0.0, 0.0
        
    tau = t / tf
    
    if profile == 'linear':
        s = tau
        s_dot = 1.0 / tf
        s_ddot = 0.0
    elif profile == 'cubic':
        s = 3 * (tau ** 2) - 2 * (tau ** 3)
        s_dot = (6 * tau - 6 * (tau ** 2)) / tf
        s_ddot = (6 - 12 * tau) / (tf ** 2)
    elif profile == 'quintic':
        s = 10 * (tau ** 3) - 15 * (tau ** 4) + 6 * (tau ** 5)
        s_dot = (30 * (tau ** 2) - 60 * (tau ** 3) + 30 * (tau ** 4)) / tf
        s_ddot = (60 * tau - 180 * (tau ** 2) + 120 * (tau ** 3)) / (tf ** 2)
    else:
        raise ValueError("Invalid profile type. Choose 'linear', 'cubic', or 'quintic'.")
        
    return s, s_dot, s_ddot

def position_trajectory(p0, pf, tf, t, profile='quintic'):
    """
    Generates a 3D Cartesian position trajectory step (x, y, z).
    
    Parameters:
        p0 (np.ndarray): Initial position [shape: (3,)]
        pf (np.ndarray): Final position [shape: (3,)]
        tf (float): Total trajectory duration
        t (float): Current execution time
        profile (str): Interpolation polynomial profile ('linear', 'cubic', 'quintic')
        
    Returns:
        p, p_dot, p_ddot: Interpolated 3D position, velocity, and acceleration vectors.
    """
    p0 = np.asarray(p0, dtype=np.float64)
    pf = np.asarray(pf, dtype=np.float64)
    
    s, s_dot, s_ddot = _time_scaling(tf, t, profile)
    
    p = p0 + s * (pf - p0)
    p_dot = s_dot * (pf - p0)
    p_ddot = s_ddot * (pf - p0)
    
    return p, p_dot, p_ddot

def orientation_euler_trajectory(euler0, eulerf, tf, t, profile='quintic'):
    """
    Method 1: Euler Angle Interpolation.
    Directly interpolates three independent rotation coordinates (e.g., Roll, Pitch, Yaw).
    Warning: Subject to representation singularities/gimbal lock during path execution.
    """
    e0 = np.asarray(euler0, dtype=np.float64)
    ef = np.asarray(eulerf, dtype=np.float64)
    
    s, s_dot, s_ddot = _time_scaling(tf, t, profile)
    
    euler = e0 + s * (ef - e0)
    euler_dot = s_dot * (ef - e0)
    euler_ddot = s_ddot * (ef - e0)
    
    return euler, euler_dot, euler_ddot

def orientation_matrix_trajectory(R0, Rf, tf, t, profile='quintic'):
    """
    Method 2: Rotation Matrix Interpolation (Axis-Angle Exponential Mapping).
    Interpolates orientation strictly on the SO(3) group manifold.
    Uses the relative rotation matrix to isolate a constant physical spatial rotation axis.
    """
    R0 = np.asarray(R0, dtype=np.float64)
    Rf = np.asarray(Rf, dtype=np.float64)
    
    s, s_dot, _ = _time_scaling(tf, t, profile)
    
    if t <= 0:
        return R0, np.zeros(3)
    if t >= tf:
        return Rf, np.zeros(3)
        
    # Relative rotation matrix from initial to target orientation
    R_rel = R0.T @ Rf
    
    # Map from rotation matrix group to axis-angle skew Lie algebra representation via matrix log
    # R_rel = e^( skew(u)*theta )
    skew_matrix = scipy.linalg.logm(R_rel)
    
    # Extract structural physical angular velocity representation in the local frame
    # S(omega_local) = skew_matrix * s_dot
    R_t_local = scipy.linalg.expm(skew_matrix * s)
    R = R0 @ R_t_local
    
    # Convert local velocity representation to absolute global space angular velocity vector
    omega_local = np.array([skew_matrix[2, 1], skew_matrix[0, 2], skew_matrix[1, 0]])
    omega_global = R0 @ omega_local * s_dot
    
    return R, omega_global

def orientation_quaternion_nlerp(q0, qf, tf, t, profile='quintic'):
    """
    Method 3: Normalized Linear Interpolation (NLERP) for Quaternions.
    Fast and computationally light, but angular velocity is not perfectly constant.
    """
    q0 = np.asarray(q0, dtype=np.float64) / np.linalg.norm(q0)
    qf = np.asarray(qf, dtype=np.float64) / np.linalg.norm(qf)
    
    s, _, _ = _time_scaling(tf, t, profile)
    
    # Shortest path check (flip signs if dot product is negative)
    dot = np.dot(q0, qf)
    if dot < 0.0:
        qf = -qf
        dot = -dot
        
    # Linear blend followed by explicit geometric projection scaling normalization
    q = (1.0 - s) * q0 + s * qf
    q = q / np.linalg.norm(q)
    
    return q

def orientation_quaternion_slerp(q0, qf, tf, t, profile='quintic'):
    """
    Method 4: Spherical Linear Interpolation (SLERP) for Quaternions.
    Guarantees a perfectly constant angular velocity rotation path across the 4D unit hypersphere.
    """
    q0 = np.asarray(q0, dtype=np.float64) / np.linalg.norm(q0)
    qf = np.asarray(qf, dtype=np.float64) / np.linalg.norm(qf)
    
    s, _, _ = _time_scaling(tf, t, profile)
    
    dot = np.dot(q0, qf)
    
    # Shortest path check
    if dot < 0.0:
        qf = -qf
        dot = -dot
        
    # If the quaternions are extremely close, fall back to linear step to prevent numerical division by zero
    if dot > 0.9995:
        q = (1.0 - s) * q0 + s * qf
        return q / np.linalg.norm(q)
        
    # Standard SLERP angular vector projection formulation
    theta_0 = np.arccos(dot)
    theta_t = theta_0 * s
    
    sin_theta_0 = np.sin(theta_0)
    sin_theta_t = np.sin(theta_t)
    
    c0 = np.sin(theta_0 - theta_t) / sin_theta_0
    c1 = sin_theta_t / sin_theta_0
    
    q = c0 * q0 + c1 * qf
    return q / np.linalg.norm(q)