import numpy as np
import utility as ram 

class Robot:
    def __init__(self):
        self.body = {}
        self.params = Robot.Params()  

    def add_body(self, body_id, parent, name, pos, quat, ipos, iquat, mass, inertia, joint_axis, joint_range):
        self.body[body_id] = Robot.Body(parent, name, pos, quat, ipos, iquat, mass, inertia, joint_axis, joint_range)

    class Body:
        def __init__(self, parent, name, pos, quat, ipos, iquat, mass, inertia, joint_axis, joint_range):
            self.parent = parent
            self.name = name
            self.pos = np.array(pos)
            self.quat = np.array(quat)
            self.ipos = np.array(ipos)
            self.iquat = np.array(iquat)
            self.mass = mass
            self.inertia = np.array(inertia)
            self.joint_axis = np.array(joint_axis)
            self.joint_range = np.array(joint_range)

    class Params:
        def __init__(self):
            base_quat = np.array([0, 0, 0, -1])
            self.base_quat = ram.quat_normalize(base_quat)
            self.end_eff_pos_local = np.array([0, 0.1, 0])
            end_eff_quat_local = np.array([-1, 1, 0, 0])
            self.end_eff_quat_local = ram.quat_normalize(end_eff_quat_local)


robot = Robot()

# shoulder_link
robot.add_body(
    1, parent='base', name='shoulder_link', pos=[0, 0, 0.163],
    quat=[1, 0, 0, 0], ipos=[0, 0, 0], iquat=[1, 0, 0, 0],
    mass=3.7, inertia=[0.0102675, 0.0102675, 0.00666],
    joint_axis=[0, 0, 1], joint_range=[-6.28319, 6.28319]
)

# upper_arm_link
robot.add_body(
    2, parent='shoulder_link', name='upper_arm_link', pos=[0, 0.138, 0],
    quat=[1, 0, 1, 0], ipos=[0, 0, 0.2125], iquat=[1, 0, 0, 0],
    mass=8.393, inertia=[0.133886, 0.133886, 0.0151074],
    joint_axis=[0, 1, 0], joint_range=[-6.28319, 6.28319]
)

# forearm_link
robot.add_body(
    3, parent='upper_arm_link', name='forearm_link', pos=[0, -0.131, 0.425],
    quat=[1, 0, 0, 0], ipos=[0, 0, 0.196], iquat=[1, 0, 0, 0],
    mass=2.275, inertia=[0.0311796, 0.0311796, 0.004095],
    joint_axis=[0, 1, 0], joint_range=[-6.28319, 6.28319]
)

# wrist_1_link
robot.add_body(
    4, parent='forearm_link', name='wrist_1_link', pos=[0, 0, 0.392],
    quat=[1, 0, 1, 0], ipos=[0, 0.127, 0], iquat=[1, 0, 0, 0],
    mass=1.219, inertia=[0.0025599, 0.0025599, 0.0021942],
    joint_axis=[0, 1, 0], joint_range=[-6.28319, 6.28319]
)

# wrist_2_link
robot.add_body(
    5, parent='wrist_1_link', name='wrist_2_link', pos=[0, 0.127, 0],
    quat=[1, 0, 0, 0], ipos=[0, 0, 0.1], iquat=[1, 0, 0, 0],
    mass=1.219, inertia=[0.0025599, 0.0025599, 0.0021942],
    joint_axis=[0, 0, 1], joint_range=[-6.28319, 6.28319]
)

# wrist_3_link
robot.add_body(
    6, parent='wrist_2_link', name='wrist_3_link', pos=[0, 0, 0.1],
    quat=[1, 0, 0, 0], ipos=[0, 0.0771683, 0], iquat=[1, 0, 0, 1],
    mass=0.1889, inertia=[0.000132134, 9.90863e-05, 9.90863e-05],
    joint_axis=[0, 1, 0], joint_range=[-6.28319, 6.28319]
)

# normalize quaternions using the rotation library
for body_id, body in robot.body.items():
    body.quat = ram.quat_normalize(body.quat)
    body.iquat = ram.quat_normalize(body.iquat)
    