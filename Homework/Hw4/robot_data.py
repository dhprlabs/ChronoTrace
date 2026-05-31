import numpy as np
from import_utility import ram

class Robot:
    class Body:
        """Defines the body (links) of the robot."""
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
        """Class to hold heterogeneous robot-level parameters."""
        def __init__(self):
            base_quat = np.array([1, 0, 0, 0])
            self.base_quat = ram.quat_normalize(base_quat)

            self.end_eff_pos_local = np.array([0.1, 0, 0])
            end_eff_quat_local = np.array([1, 0, 0, 0])
            self.end_eff_quat_local = ram.quat_normalize(end_eff_quat_local)

    def __init__(self):
        self.body = {}
        self.params = Robot.Params()  # Initialize robot parameters

    def add_body(self, body_id, parent, name, pos, quat, ipos, iquat, mass, inertia, joint_axis, joint_range):
        self.body[body_id] = Robot.Body(parent, name, pos, quat, ipos, iquat, mass, inertia, joint_axis, joint_range)


# Initialize the robot
robot = Robot()

# Add bodies
robot.add_body(
    1, parent='ground', name='shoulder_link', pos=[0, 0, 0.072],
    quat=[1, 0, 0, 0], ipos=[2.23482e-05, 4.14609e-05, 0.0066287], iquat=[0.0130352, 0.706387, 0.012996, 0.707586],
    mass=0.480879, inertia=[0.000588946, 0.000555655, 0.000378999],
    joint_axis=[0, 0, 1], joint_range=[-3.14158, 3.14158]
)

      # <body name="wx250s/shoulder_link" pos="0 0 0.072">
      #   <inertial pos="2.23482e-05 4.14609e-05 0.0066287" quat="0.0130352 0.706387 0.012996 0.707586" mass="0.480879"
      #     diaginertia="0.000588946 0.000555655 0.000378999"/>
      #   <joint name="waist" axis="0 0 1" range="-3.14158 3.14158"/>
      #   <geom pos="0 0 -0.003" quat="1 0 0 1" mesh="wx250s_2_shoulder" class="visual"/>
      #   <geom pos="0 0 -0.003" quat="1 0 0 1" mesh="wx250s_2_shoulder" class="collision"/>

robot.add_body(
    2, parent='shoulder_link', name='upper_arm_link', pos=[0, 0, 0.03865],
    quat=[1, 0, 0, 0], ipos=[0.0171605, 2.725e-07, 0.191323], iquat=[0.705539, 0.0470667, -0.0470667, 0.705539],
    mass=0.430811, inertia=[0.00364425, 0.003463, 0.000399348],
    joint_axis=[0, 1, 0], joint_range=[-1.88496, 1.98968]
)
        # <body name="wx250s/upper_arm_link" pos="0 0 0.03865">
        #   <inertial pos="0.0171605 2.725e-07 0.191323" quat="0.705539 0.0470667 -0.0470667 0.705539" mass="0.430811"
        #     diaginertia="0.00364425 0.003463 0.000399348"/>
        #   <joint name="shoulder" range="-1.88496 1.98968"/>
        #   <geom quat="1 0 0 1" mesh="wx250s_3_upper_arm" class="visual"/>
        #   <geom quat="1 0 0 1" mesh="wx250s_3_upper_arm" class="collision"/>
        #   0 1 0

robot.add_body(
    3, parent='upper_arm_link', name='upper_forearm_link', pos=[0.04975, 0, 0.25],
    quat=[1, 0, 0, 0], ipos=[0.107963, 0.000115876, 0], iquat=[0.000980829, 0.707106, -0.000980829, 0.707106],
    mass=0.234589, inertia=[0.000888, 0.000887807, 3.97035e-05],
    joint_axis=[0, 1, 0], joint_range=[-2.14675, 1.6057]
)
          # <body name="wx250s/upper_forearm_link" pos="0.04975 0 0.25">
          #   <inertial pos="0.107963 0.000115876 0" quat="0.000980829 0.707106 -0.000980829 0.707106" mass="0.234589"
          #     diaginertia="0.000888 0.000887807 3.97035e-05"/>
          #   <joint name="elbow" range="-2.14675 1.6057"/>
          #   <geom mesh="wx250s_4_upper_forearm" class="visual"/>
          #   <geom mesh="wx250s_4_upper_forearm" class="collision"/>
          #
robot.add_body(
    4, parent='upper_forearm_link', name='lower_forearm_link', pos=[0.175, 0, 0],
    quat=[1, 0, 0, 0], ipos=[0.0374395, 0.00522252, 0], iquat=[-0.0732511, 0.703302, 0.0732511, 0.703302],
    mass=0.220991, inertia=[0.0001834, 0.000172527, 5.88633e-05],
    joint_axis=[1, 0, 0], joint_range=[-3.14158, 3.14158]
)

            # <body name="wx250s/lower_forearm_link" pos="0.175 0 0">
            #   <inertial pos="0.0374395 0.00522252 0" quat="-0.0732511 0.703302 0.0732511 0.703302" mass="0.220991"
            #     diaginertia="0.0001834 0.000172527 5.88633e-05"/>
            #   <joint name="forearm_roll" axis="1 0 0" range="-3.14158 3.14158"/>
            #   <geom quat="0 1 0 0" mesh="wx250s_5_lower_forearm" class="visual"/>
            #   <geom quat="0 1 0 0" mesh="wx250s_5_lower_forearm" class="collision"/>

robot.add_body(
    5, parent='lower_forearm_link', name='wrist_link', pos=[0.075, 0, 0],
    quat=[1, 0, 0, 0], ipos=[0.04236, -1.0663e-05, 0.010577], iquat=[0.608721, 0.363497, -0.359175, 0.606895],
    mass=0.084957, inertia=[3.29057e-05, 3.082e-05, 2.68343e-05],
    joint_axis=[0, 1, 0], joint_range=[-1.74533, 2.14675]
)
              # <body name="wx250s/wrist_link" pos="0.075 0 0">
              #   <inertial pos="0.04236 -1.0663e-05 0.010577" quat="0.608721 0.363497 -0.359175 0.606895" mass="0.084957"
              #     diaginertia="3.29057e-05 3.082e-05 2.68343e-05"/>
              #   <joint name="wrist_angle" axis="0 1 0" range="-1.74533 2.14675"/>
              #   <geom quat="1 0 0 1" mesh="wx250s_6_wrist" class="visual"/>
              #   <geom quat="1 0 0 1" mesh="wx250s_6_wrist" class="collision"/>

robot.add_body(
    6, parent='wrist_link', name='gripper_link', pos=[0.065, 0, 0],
    quat=[1, 0, 0, 0], ipos=[0.0325296, 4.2061e-07, 0.0090959], iquat=[0.546081, 0.419626, 0.62801, 0.362371],
    mass=0.110084, inertia=[0.00307592, 0.00307326, 0.0030332],
    joint_axis=[1, 0, 0], joint_range=[-3.14158, 3.14158]
)

                # <body name="wx250s/gripper_link" pos="0.065 0 0">
                #   <inertial pos="0.0325296 4.2061e-07 0.0090959" quat="0.546081 0.419626 0.62801 0.362371"
                #     mass="0.110084" diaginertia="0.00307592 0.00307326 0.0030332"/>
                #   <joint name="wrist_rotate" axis="1 0 0" range="-3.14158 3.14158"/>
                #   <geom pos="-0.02 0 0" quat="1 0 0 1" mesh="wx250s_7_gripper" class="visual"/>
                #   <geom pos="-0.02 0 0" quat="1 0 0 1" mesh="wx250s_7_gripper" class="collision"/>
                #   <geom pos="-0.02 0 0" quat="1 0 0 1" mesh="wx250s_9_gripper_bar" class="visual"/>
                #   <geom pos="-0.02 0 0" quat="1 0 0 1" mesh="wx250s_9_gripper_bar" class="collision"/>
                #   <site name="attachment_site" size="0.001" pos="0.1 0 0" quat="1 0 0 0" rgba="1 0 0 1" group="1"/>
                #

#                 <body name="wx250s/left_finger_link" pos="0.066 0 0">
# <inertial pos="0.013816 0 0" quat="0.705384 0.705384 -0.0493271 -0.0493271" mass="0.016246"
#   diaginertia="4.79509e-06 3.7467e-06 1.48651e-06"/>
# <joint name="left_finger" axis="0 1 0" type="slide" range="0.015 0.037"/>
# <geom pos="0 0.005 0" quat="0 0 0 -1" mesh="wx250s_10_gripper_finger" class="visual"/>
# <geom pos="0 0.005 0" quat="0 0 0 -1" mesh="wx250s_10_gripper_finger" class="collision"/>
# <geom name="left/left_g0" pos="0.042 -0.009 0.012" class="sphere_collision"/>
# <geom name="left/left_g1" pos="0.042 -0.009 -0.012" class="sphere_collision"/>
# </body>

                #   <body name="wx250s/right_finger_link" pos="0.066 0 0">
                #     <inertial pos="0.013816 0 0" quat="0.705384 0.705384 0.0493271 0.0493271" mass="0.016246"
                #       diaginertia="4.79509e-06 3.7467e-06 1.48651e-06"/>
                #     <joint name="right_finger" axis="0 1 0" type="slide" range="-0.037 -0.015"/>
                #     <geom pos="0 -0.005 0" quat="0 0 1 0" mesh="wx250s_10_gripper_finger" class="visual"/>
                #     <geom pos="0 -0.005 0" quat="0 0 1 0" mesh="wx250s_10_gripper_finger" class="collision"/>
                #     <geom name="right/right_g0" pos="0.042 0.009 0.012" class="sphere_collision"/>
                #     <geom name="right/right_g1" pos="0.042 0.009 -0.012" class="sphere_collision"/>
                #   </body>
                # </body>

# Normalize quaternions using the rotation library
for body_id, body in robot.body.items():
    body.quat = ram.quat_normalize(body.quat)
    body.iquat = ram.quat_normalize(body.iquat)

# Example of parameter usage
#print(robot.params.end_eff_pos_local)  # Access robot-level parameter
#print(robot[1])  # Access body
