import numpy as np 
import utility
import pprint


class RobotData:
    def __init__(self):
        self.bodies = []
        self.base = []
        self.end_effector = []

    def add_body(self, id, name, parent, pos, quat, mass, inertia, ipos, joint_axis, joint_range):
        quat_np = utility.quaternion_normalization(quat)

        data = {
            "id": id,
            "name": name,
            "parent": parent,
            "pos": np.array(pos),
            "quat": np.array(quat_np),
            "mass": mass,
            "inertia": np.array(inertia),
            "ipos": np.array(ipos),
            "joint_axis": np.array(joint_axis),
            "joint_range": np.array(joint_range), 
        }

        self.bodies.append(data)

    def add_base(self, name, pos, quat, mass, inertia, ipos):
        quat_np = utility.quaternion_normalization(quat)
        
        data = {
            "name": name,
            "pos": np.array(pos),
            "quat": np.array(quat_np),
            "mass": mass,
            "inertia": np.array(inertia),
            "ipos": np.array(ipos),
        }

        self.base.append(data)

    def add_end_effector(self, name, parent, pos, quat):
        quat_np = utility.quaternion_normalization(quat)
        
        data = {
            "name": name,
            "parent": parent,
            "pos": np.array(pos),
            "quat": np.array(quat_np),
        }

        self.end_effector.append(data)

    def print_data(self):
        for i in range(0, len(self.base)):
            pprint.pprint(self.base[i])
            print("="*161)

        for i in range(0, len(self.end_effector)):
            pprint.pprint(self.end_effector[i])
            print("="*161)

        for i in range(0, len(self.bodies)):
            pprint.pprint(self.bodies[i])
            print("="*161)


rd = RobotData()

rd.add_base("base", [0, 0, 0], [0, 0, 0, -1], 4.0, [0.00443333156, 0.00443333156, 0.0072], [0, 0, 0])

rd.add_end_effector("attachment_site", "wrist_3_link", [0, 0.1, 0], [-1, 1, 0, 0])

rd.add_body(1, "shoulder_link", "base", [0, 0, 0.163], [1, 0, 0, 0], 3.7, [0.0102675, 0.0102675, 0.00666], [0, 0, 0], [0, 0, 1], [-6.28319, 6.28319])

rd.add_body(2, "upper_arm_link", "shoulder_link", [0, 0.138, 0], [1, 0, 1, 0], 8.393, [0.133886, 0.133886, 0.0151074], [0, 0, 0.2125], [0, 1, 0], [-6.28319, 6.28319])

rd.add_body(3, "forearm_link", "upper_arm_link", [0, -0.131, 0.425], [1, 0, 0, 0], 2.275, [0.0311796, 0.0311796, 0.004095], [0, 0, 0.196], [0, 1, 0], [-3.1415, 3.1415])

rd.add_body(4, "wrist_1_link", "forearm_link", [0, 0, 0.392], [1, 0, 1, 0], 1.219, [0.0025599, 0.0025599, 0.0021942], [0, 0.127, 0], [0, 1, 0], [-6.28319, 6.28319])

rd.add_body(5, "wrist_2_link", "wrist_1_link", [0, 0.127, 0], [1, 0, 0, 0], 1.219, [0.0025599, 0.0025599, 0.0021942], [0, 0, 0.1], [0, 0, 1], [-6.28319, 6.28319])

rd.add_body(6, "wrist_3_link", "wrist_2_link", [0, 0, 0.1], [1, 0, 0, 0], 0.1889, [0.000132134, 9.90863e-05, 9.90863e-05], [0, 0.0771683, 0], [0, 1, 0], [-6.28319, 6.28319])
    