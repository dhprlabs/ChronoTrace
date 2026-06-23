import numpy as np
from scipy.interpolate import CubicSpline

def linear_trajectory(q0, qf, tf, t):
    """
    Generates a multi-DoF linear (constant velocity) trajectory point.
    
    Parameters:
        q0 (np.ndarray): Initial joint positions [shape: (num_joints,)]
        qf (np.ndarray): Final joint positions [shape: (num_joints,)]
        tf (float): Total trajectory duration (seconds)
        t (float): Current execution time (seconds)
        
    Returns:
        q (np.ndarray): Joint positions at time t
        q_dot (np.ndarray): Joint velocities at time t
        q_ddot (np.ndarray): Joint accelerations at time t (always zero)
    """
    q0 = np.asarray(q0, dtype=np.float64)
    qf = np.asarray(qf, dtype=np.float64)
    
    if t <= 0:
        return q0, np.zeros_like(q0), np.zeros_like(q0)
    if t >= tf:
        return qf, np.zeros_like(qf), np.zeros_like(qf)
        
    # Interpolation factor
    alpha = t / tf
    
    q = q0 + alpha * (qf - q0)
    q_dot = (qf - q0) / tf
    q_ddot = np.zeros_like(q0)
    
    return q, q_dot, q_ddot

def cubic_trajectory(q0, qf, tf, t, v0=None, vf=None):
    """
    Generates a multi-DoF Cubic Polynomial Trajectory point.
    Satisfies position boundary conditions and matches initial/final velocities.
    
    Parameters:
        q0 (np.ndarray): Initial joint positions
        qf (np.ndarray): Final joint positions
        tf (float): Total trajectory duration
        t (float): Current execution time
        v0 (np.ndarray, optional): Initial velocities. Defaults to zero vector.
        vf (np.ndarray, optional): Final velocities. Defaults to zero vector.
        
    Returns:
        q, q_dot, q_ddot: Multi-joint position, velocity, and acceleration arrays.
    """
    q0 = np.asarray(q0, dtype=np.float64)
    qf = np.asarray(qf, dtype=np.float64)
    v0 = np.zeros_like(q0) if v0 is None else np.asarray(v0, dtype=np.float64)
    vf = np.zeros_like(qf) if vf is None else np.asarray(vf, dtype=np.float64)
    
    if t <= 0:
        return q0, v0, np.zeros_like(q0)
    if t >= tf:
        return qf, vf, np.zeros_like(qf)
        
    # Solve for polynomial coefficients analytically per joint: q(t) = a0 + a1*t + a2*t^2 + a3*t^3
    a0 = q0
    a1 = v0
    a2 = (3 * (qf - q0) - (2 * v0 + vf) * tf) / (tf ** 2)
    a3 = (2 * (q0 - qf) + (v0 + vf) * tf) / (tf ** 3)
    
    # Evaluate polynomial and derivatives
    q = a0 + a1 * t + a2 * (t ** 2) + a3 * (t ** 3)
    q_dot = a1 + 2 * a2 * t + 3 * a3 * (t ** 2)
    q_ddot = 2 * a2 + 6 * a3 * t
    
    return q, q_dot, q_ddot

def quintic_trajectory(q0, qf, tf, t, v0=None, vf=None, a0=None, af=None):
    """
    Generates a multi-DoF Quintic Polynomial Trajectory point.
    Ensures zero jerk/accelerations at boundary conditions for highly smooth starts/stops.
    
    Parameters:
        q0 (np.ndarray): Initial joint positions
        qf (np.ndarray): Final joint positions
        tf (float): Total trajectory duration
        t (float): Current execution time
        v0, vf (np.ndarray, optional): Boundary velocities. Defaults to zero vectors.
        a0, af (np.ndarray, optional): Boundary accelerations. Defaults to zero vectors.
    """
    q0 = np.asarray(q0, dtype=np.float64)
    qf = np.asarray(qf, dtype=np.float64)
    v0 = np.zeros_like(q0) if v0 is None else np.asarray(v0, dtype=np.float64)
    vf = np.zeros_like(qf) if vf is None else np.asarray(vf, dtype=np.float64)
    a0 = np.zeros_like(q0) if a0 is None else np.asarray(a0, dtype=np.float64)
    af = np.zeros_like(qf) if af is None else np.asarray(af, dtype=np.float64)
    
    if t <= 0:
        return q0, v0, a0
    if t >= tf:
        return qf, vf, af
        
    # Analytical solution coefficients for q(t) = a0 + a1*t + a2*t^2 + a3*t^3 + a4*t^4 + a5*t^5
    c0 = q0
    c1 = v0
    c2 = 0.5 * a0
    c3 = (20 * (qf - q0) - (8 * vf + 12 * v0) * tf - (3 * a0 - af) * (tf ** 2)) / (2 * (tf ** 3))
    c4 = (30 * (q0 - qf) + (14 * vf + 16 * v0) * tf + (3 * a0 - 2 * af) * (tf ** 2)) / (2 * (tf ** 4))
    c5 = (12 * (qf - q0) - 6 * (vf + v0) * tf - (a0 - af) * (tf ** 2)) / (2 * (tf ** 5))
    
    # Evaluate terms
    q = c0 + c1*t + c2*(t**2) + c3*(t**3) + c4*(t**4) + c5*(t**5)
    q_dot = c1 + 2*c2*t + 3*c3*(t**2) + 4*c4*(t**3) + 5*c5*(t**4)
    q_ddot = 2*c2 + 6*c3*t + 12*c4*(t**2) + 20*c5*(t**3)
    
    return q, q_dot, q_ddot


class CubicSplineWaypointPlanner:
    """
    Handles multi-waypoint interpolation using continuous clamped cubic splines.
    Perfect for complex joint trajectories requiring zero velocity at boundaries 
    and smooth acceleration blending across via-points.
    """
    def __init__(self, time_steps, waypoints):
        """
        Parameters:
            time_steps (list or np.ndarray): Monotonically increasing timestamps [shape: (N,)]
            waypoints (list or np.ndarray): Joint position configuration matrix [shape: (N, num_joints)]
        """
        self.time_steps = np.asarray(time_steps, dtype=np.float64)
        self.waypoints = np.asarray(waypoints, dtype=np.float64)
        self.tf = self.time_steps[-1]
        
        # Enforce clamped boundary conditions (zero velocity at initial and final points)
        # scipy CubicSpline expects shapes mapped to axis=0 for time interpolation nodes
        self.spline = CubicSpline(self.time_steps, self.waypoints, axis=0, bc_type='clamped')
        
        # Pre-compute velocity and acceleration derivatives analytically
        self.spline_dot = self.spline.derivative(nu=1)
        self.spline_ddot = self.spline.derivative(nu=2)
        
    def evaluate(self, t):
        """
        Evaluates the vector spline profile state at timestamp t.
        """
        if t <= 0:
            return self.waypoints[0].copy(), np.zeros_like(self.waypoints[0]), np.zeros_like(self.waypoints[0])
        if t >= self.tf:
            return self.waypoints[-1].copy(), np.zeros_like(self.waypoints[-1]), np.zeros_like(self.waypoints[-1])
            
        q = self.spline(t)
        q_dot = self.spline_dot(t)
        q_ddot = self.spline_ddot(t)
        
        return q, q_dot, q_ddot
