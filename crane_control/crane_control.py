import numpy as np
from scipy.spatial.transform import Rotation
from ikpy.chain import Chain
from ikpy.link import URDFLink
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import xml.etree.ElementTree as ET

np.set_printoptions(precision=3, suppress=True)

class Joint:
    def __init__(self, joint_type, initial_position, velocity_limit):
        self.joint_type = joint_type  # 'revolute' or 'prismatic'
        self.position = initial_position
        self.target = None
        self.velocity_limit = velocity_limit

    def set_target(self, target):
        self.target = target

    def update_position(self, dt):
        if self.target is not None:
            error = self.target - self.position
            velocity = np.clip(error / dt, -self.velocity_limit, self.velocity_limit)
            self.position += velocity * dt
        else:
            raise AttributeError('Target not set')

class Crane:
    def __init__(self):
        self.joints = None

    def initialise_crane_model(self, joints):
        for joint in joints:
            self.joints.append(Joint(joint[0], joint[1], joint[2]))

    def set_target(self, target):
        for i, joint in enumerate(self.joints):
            joint.set_target(target[i])

    def update(self, dt):
        for joint in self.joints:
            joint.update_position(dt)

    def get_positions(self):
        return [joint.position for joint in self.joints]
    
    # def run_sim(self):
    #     # Simulation parameters
    #     dt = 1.0  # Time step
    #     simulation_time = 10.0  # Total simulation time

    #     # Run simulation
    #     time = 0.0
    #     while time < simulation_time:
    #         self.update(dt)
    #         positions = self.get_positions()
    #         print(f"Time: {time:.2f}, Positions: {positions}")
    #         time += dt
    
class CraneControl():
    def __init__(self):
        # Read DH parameters from URDF
        self.urdf_file = "config/awesome_crane.urdf"
        self.crane = Chain.from_urdf_file(self.urdf_file, active_links_mask=[False, True, True, True, True, False], last_link_vector=np.array([0.5, 0.0, 0.0]))
        self.joint_velocity_limits = self.load_velocity_limits()

        self.current_ee_position = None
        self.current_ee_orientation = None 
        self.current_joint_positions = None 

        self.initialise_crane()


        self.crane_model = Crane()
        joints = []
        for i, joint in enumerate(self.crane.links):
            if i == 0 or i == (len(self.crane.links)- 1):
                continue
            joints.append([joint.joint_type, self.current_joint_positions[i], self.joint_velocity_limits[i]])
        self.crane_model.initialise_crane_model(joints)

        self.origin_shift_transformation = np.eye(4, 4)

        self.shift_origin(np.array([0, 0, 0, 0]))

    def load_velocity_limits(self):
        # Parse the XML data
        robot = ET.parse(self.urdf_file)
        root = robot.getroot()

        # Find all joints and extract their velocity limits
        velocity_limits = []
        for joint in root.findall(".//joint"):
            velocity_limit = joint.find("limit").get("velocity")
            velocity_limits.append(float(velocity_limit))
        velocity_limits.insert(0, 0)
        velocity_limits.append(0)
        print(f"Loaded Velocity limits: {velocity_limits}")
        return velocity_limits

    def augment_joint_angles(self, joint_angles: list):
        return np.array([0, np.deg2rad(joint_angles[0]), joint_angles[1], np.deg2rad(joint_angles[2]), np.deg2rad(joint_angles[3])])
    
    def forward_kinematics(self, joint_angles):
        print("==================")
        # Compute forward kinematics
        print("Forward Kinematics")
        print(f"Joint angles: {joint_angles}")

        # ikpy shenanigans
        joint_angles.insert(0, 0)
        joint_angles.append(0)

        F = self.crane.forward_kinematics(joint_angles)

        print(f"End-Effector position: {F[:3, 3]}")
        ee_pose = F[:3, 3]
        rot_matrix = F[:3, :3]
        r = Rotation.from_matrix(rot_matrix)
        ee_orientation = r.as_euler('xyz')
        print(f"End-Effector orientation: {ee_orientation}")

        return ee_pose, ee_orientation
    
    def initialise_crane(self):

        joint_angles = self.augment_joint_angles([0, 0, 0, 0])
        self.current_ee_position, self.current_ee_orientation = self.forward_kinematics(joint_angles)
        self.current_joint_positions = joint_angles
    
    def shift_origin(self, array: np.ndarray):
        incoming_origin_shift_position = array[:3]
        incoming_origin_shift_yaw = np.deg2rad(np.array([0.0, 0.0, array[3]]))

        self.origin_shift_transformation[:3, 3] = incoming_origin_shift_position
        r = Rotation.from_euler("xyz", incoming_origin_shift_yaw)
        rot_mat = r.as_matrix()
        self.origin_shift_transformation[:3, :3] = rot_mat
        # print(origin_shift_transformation)

        print("==================")
        print("world->base_link transform after transformation")

        self.crane.links[1] = URDFLink(
                    name=self.crane.links[1].name,
                    bounds=self.crane.links[1].bounds,
                    origin_translation=incoming_origin_shift_position,
                    origin_orientation=np.deg2rad(incoming_origin_shift_yaw),
                    rotation=self.crane.links[1].rotation,
                    translation=self.crane.links[1].translation,
                    use_symbolic_matrix=self.crane.links[1].use_symbolic_matrix,
                    joint_type=self.crane.links[1].joint_type
                )

        print(f"translation: {self.crane.links[1].origin_translation}")
        print(f"orientation: {np.rad2deg(self.crane.links[1].origin_orientation)}")

    def goto_point(self, target_ee_pos):

        print("==================")
        # Compute inverse kinematics
        print("Inverse Kinematics")

        print(f"Start End-Effector position: {self.current_ee_position}")
        print(f"Start End-Effector orientation: {self.current_ee_orientation}")

        print(f"Target End-effector position: {target_ee_pos}")

        initial_joint_positions = self.crane.inverse_kinematics(target_position=self.current_ee_position, target_orientation=self.current_ee_orientation)
        final_joint_positions = self.crane.inverse_kinematics(target_position=target_ee_pos)

        resulting_ee_pos, resulting_ee_orientation = self.forward_kinematics(final_joint_positions)

        print(f"initial joint positions: {initial_joint_positions}")
        print(f"final joint positions: {final_joint_positions}")
        print(f"final End-Effector position: {resulting_ee_pos}")
        print(f"Error in End-Effector position: {resulting_ee_pos - target_ee_pos}")
        

        # cannot reach instantaneously, add a slow movement to it (controller?)
        self.crane_model.set_target(final_joint_positions[1:-1])

        self.crane_model.update(1.0)

        updated_joint_angles = self.crane_model.get_positions()
        self.current_joint_positions = self.augment_joint_angles(updated_joint_angles)
