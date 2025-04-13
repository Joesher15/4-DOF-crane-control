import numpy as np
from scipy.spatial.transform import Rotation
from ikpy.chain import Chain
from ikpy.link import URDFLink
import xml.etree.ElementTree as ET
import asyncio
import websockets
import json
import re

np.set_printoptions(precision=3, suppress=True)

def separate_numbers(input_string):
    numbers = re.findall(r'-?\d+\.\d+|-?\d+', input_string)
    return [float(num) for num in numbers]

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
            if self.joint_type == 'revolute':
                error = self.target - self.position
                # print(error)
                velocity = np.clip(error / dt, -self.velocity_limit, self.velocity_limit)
                self.position += velocity * dt
            elif self.joint_type == 'prismatic':
                error = self.target - self.position
                # print(error)
                velocity = np.clip(error / dt, -self.velocity_limit, self.velocity_limit)
                self.position += velocity * dt
            elif self.joint_type == 'origin_translate':
                error_vec = self.target - self.position
                error = np.linalg.norm(error_vec)
                # print(self.position, self.target)
                # print(error_vec, error)
                if error != 0.0:
                    direction = error_vec / error
                    velocity = np.clip(error / dt, -self.velocity_limit, self.velocity_limit)
                    self.position += velocity * dt * direction
            else:
                raise ValueError(f'joint type unknown: {self.joint_type}')
        else:
            raise AttributeError('Target not set')

class Crane:
    def __init__(self):
        self.joints = []

    def initialise_crane_model(self, joints):
        for joint in joints:
            self.joints.append(Joint(joint[0], joint[1], joint[2]))

    def set_target(self, target):
        for i, joint in enumerate(self.joints):
            # print(i, joint)
            joint.set_target(target[i])

    def crane_moving_status(self):
        reached = True
        for joint in self.joints:
            if(joint.target is not None):
                if joint.joint_type == 'origin_translate':
                    reached = reached and np.all(joint.target == joint.position)
                else:
                    reached = reached and joint.target == joint.position
            else:
                reached = False
        if (reached):
            return "Target Reached"
        else:
            return "Robot moving"

    def update(self, dt):
        for joint in self.joints:
            joint.update_position(dt)

    def get_positions(self):
        return [joint.position for joint in self.joints]

    def get_targets(self):
        return [joint.target for joint in self.joints]
            
    
class CraneControl():
    def __init__(self):
        # Read DH parameters from URDF
        self.urdf_file = "config/public/awesome_crane1.urdf"
        z_offset = self.load_last_link()
        self.crane = Chain.from_urdf_file(self.urdf_file, active_links_mask=[False, True, True, True, True, False], last_link_vector=np.array([0.0, 0.0, z_offset]))
        self.joint_velocity_limits = self.load_velocity_limits()

        self.current_ee_position = None
        self.current_ee_orientation = None 
        self.current_joint_positions = None 

        self.publish = False

        self.crane_model = Crane()
        self.initialise_crane()
        
    def load_last_link(self):
        # Parse the XML data
        robot = ET.parse(self.urdf_file)
        root = robot.getroot()
        links = root.findall(".//link")
        last_link = links[-1]
        oxyz = last_link.find("visual").find("origin").get("xyz")
        oxyz = separate_numbers(oxyz)
        length = last_link.find("visual").find("geometry").find("cylinder").get("length")
        length = separate_numbers(length)[0]
        z_offset =  oxyz[2] - length / 2
        return z_offset

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
        return [0, joint_angles[0], joint_angles[1], joint_angles[2], joint_angles[3], 0]
    
    def forward_kinematics(self, joint_angles):
        F = self.crane.forward_kinematics(joint_angles)
        ee_pose = F[:3, 3]
        rot_matrix = F[:3, :3]
        r = Rotation.from_matrix(rot_matrix)
        ee_orientation = r.as_euler('xyz')

        return ee_pose, ee_orientation
    
    def initialise_crane(self):
        # set initial values
        origin_translate = np.array([0.0, 0.0, 0.0])
        origin_rotation = 0.0
        start_joint_position = [0.0, 1.0, 0.0, 0.0] # TODO: Remove hardcoding and expose to config
        origin_translation_vel_limit = 0. # TODO: Remove hardcoding and expose to config
        origin_rotational_vel_limit = 1.0 # TODO: Remove hardcoding and expose to config

        # update state variables with initial values
        # state pose of crane
        self.origin_translate = origin_translate
        self.origin_rotation = origin_rotation
        self.current_joint_positions = self.augment_joint_angles(start_joint_position)
        self.current_ee_position, self.current_ee_orientation = self.forward_kinematics(self.current_joint_positions)

        # state of targets for controller
        self.current_target_ee_pos = self.current_ee_position
        self.current_target_origin_translate = self.origin_translate
        self.current_target_origin_rotation = self.origin_rotation

        joints = []
        for i, joint in enumerate(self.crane.links):
            if i == 0 or i == (len(self.crane.links)- 1):
                continue
            joints.append([joint.joint_type, self.current_joint_positions[i], self.joint_velocity_limits[i]])
        self.current_joint_positions.insert(1, self.origin_rotation)
        self.current_joint_positions.insert(1, self.origin_translate)
        joints.insert(0, ['revolute', self.origin_rotation, origin_rotational_vel_limit])
        joints.insert(0, ['origin_translate', self.origin_translate, origin_translation_vel_limit])
        self.crane_model.initialise_crane_model(joints)

        self.crane_model.set_target(self.current_joint_positions[1:-1])
    
    def set_origin_target(self, array: np.ndarray):
        current_joint_targets = self.crane_model.get_targets()
        self.current_target_origin_translate = array[:3]
        self.current_target_origin_rotation = array[3]

        current_joint_targets[0] = self.current_target_origin_translate
        current_joint_targets[1] = self.current_target_origin_rotation

        self.crane_model.set_target(current_joint_targets)

    def shift_origin(self):
        origin_shift_transformation = np.eye(4, 4)
        incoming_origin_shift_yaw = np.deg2rad(np.array([0.0, 0.0, self.origin_rotation]))
        origin_shift_transformation[:3, 3] = self.origin_translate
        r = Rotation.from_euler("xyz", incoming_origin_shift_yaw)
        rot_mat = r.as_matrix()
        origin_shift_transformation[:3, :3] = rot_mat

        # print("==================")
        # print("world->base_link transform after transformation")

        self.crane.links[1] = URDFLink(
                    name=self.crane.links[1].name,
                    bounds=self.crane.links[1].bounds,
                    origin_translation=self.origin_translate,
                    origin_orientation=np.deg2rad(incoming_origin_shift_yaw),
                    rotation=self.crane.links[1].rotation,
                    translation=self.crane.links[1].translation,
                    use_symbolic_matrix=self.crane.links[1].use_symbolic_matrix,
                    joint_type=self.crane.links[1].joint_type
                )
        # print(f"link name: {self.crane.links[1].name}")
        # print(f"translation: {self.crane.links[1].origin_translation}")
        # print(f"orientation: {np.rad2deg(self.crane.links[1].origin_orientation)}")

        if(np.linalg.norm(self.origin_translate) > 0.05): # TODO: Remove hardcoding and expose to config
            self.goto_point(self.current_target_ee_pos)

    def add_origin_joints(self, array):
        array.insert(0, self.origin_rotation)
        array.insert(0, self.origin_translate)
        return array
    
    def goto_point(self, target_ee_pos: np.ndarray):

        initial_joint_positions = self.crane.inverse_kinematics(target_position=self.current_ee_position, target_orientation=self.current_ee_orientation)
        final_joint_positions = self.crane.inverse_kinematics(target_position=target_ee_pos)

        resulting_ee_pos, resulting_ee_orientation = self.forward_kinematics(final_joint_positions)

        ee_position_error = resulting_ee_pos - target_ee_pos
        if (np.abs(np.linalg.norm(ee_position_error)) > 0.1): # TODO: Remove hardcoding and expose to config
            return "Not reachable"
        else:
            origin_targets = [self.current_target_origin_translate, self.current_target_origin_rotation]
            self.crane_model.set_target(origin_targets + final_joint_positions[1:-1].tolist())
            self.current_target_ee_pos = resulting_ee_pos
            return "Going to Pose"
        
    def set_joint_angles(self, joint_angles):
            joint_angles = self.add_origin_joints(joint_angles)
            self.crane_model.set_target(joint_angles)
            return "Going to Pose"
        
    async def publish_state(self, websocket, pub_interval):
        while True:
            updated_joint_angles = self.crane_model.get_positions()
            self.origin_translate = updated_joint_angles[0]
            self.origin_rotation = updated_joint_angles[1]
            self.current_joint_positions = self.augment_joint_angles(updated_joint_angles[2:])
            self.current_ee_position, self.current_ee_orientation = self.forward_kinematics(self.current_joint_positions)

            data = {'command': 'update', 'joint_positions': updated_joint_angles[2:],
                    'ee_position': [self.current_ee_position[0], self.current_ee_position[1], 
                                    self.current_ee_position[2]],
                    'robot_motion_status': self.crane_model.crane_moving_status(),
                    'origin_shift': [self.origin_translate[0], self.origin_translate[1], self.origin_translate[2],
                                     np.deg2rad(self.origin_rotation)]}
            await websocket.send(json.dumps(data))

            await asyncio.sleep(pub_interval)


async def main():

    control_update_interval = 0.1 # s # TODO: Remove hardcoding and expose to config
    publish_interval = 0.2 # s # TODO: Remove hardcoding and expose to config

    # Initialize crane model and set targets as needed
    crane_control = CraneControl()

    async def handler(websocket):
        print("Client Connected")
        publish_state_task = asyncio.create_task(crane_control.publish_state(websocket, publish_interval))
        try:
            async for message in websocket:
                data = json.loads(message)
                print(f"Received command: {data['command']}")
                if data['command'] == 'origin_pose':
                    origin = data['data']['origin']
                    print(f"Received origin pose command: {origin}")
                    array = np.array([origin['ox'], origin['oy'], origin['oz'], origin['oyaw']])
                    crane_control.set_origin_target(array)
                if data['command'] == 'ee_position':
                    ee_command = data['data']['ee_command']
                    print(f"Received ee_position command: {ee_command}")
                    go_to_response = crane_control.goto_point(np.array([ee_command['x'], ee_command['y'], ee_command['z']]))
                    response = json.dumps({'command': 'go_to_response', 'res': go_to_response})
                    await websocket.send(response)
                if data['command'] == 'joint_pose':
                    joint_command = data['data']['joint_command']
                    print(f"Received joint positions command: {joint_command}")
                    set_joint_angles_resp = crane_control.set_joint_angles(joint_command)
                    response = json.dumps({'command': 'joint_command_response', 'res': set_joint_angles_resp})
                    await websocket.send(response)
        finally:
            publish_state_task.cancel()

    async def update_crane():
        while True:
            crane_control.shift_origin()
            crane_control.crane_model.update(control_update_interval)
            await asyncio.sleep(control_update_interval)

    print("Attempting to listen on port 8765")
    async with websockets.serve(handler, "localhost", 8765):
        await update_crane()  # Run forever


if __name__ == "__main__":
    asyncio.run(main())