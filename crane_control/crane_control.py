import numpy as np
import xml.etree.ElementTree as ET
import re

np.set_printoptions(precision=3, suppress=True)


def separate_numbers(input_string):
    numbers = re.findall(r'-?\d+\.\d+|-?\d+', input_string)
    return [float(num) for num in numbers]

def read_urdf_parameters(urdf_file):
    """
    Read the URDF file and extract DH parameters.
    """
    xml = ET.parse(urdf_file)
    robot = xml.getroot()

    dh_params = []
    
    for joint in robot.findall('joint'):
        joint_origin = separate_numbers(joint.find('origin').get('xyz'))
        joint_axis = separate_numbers(joint.find('axis').get('xyz'))
        if joint.get('type') == 'revolute':
            theta = 0  # will be replaced by input
            d = joint_origin[2]
            a = joint_origin[0]
            alpha = np.rad2deg(np.arctan2(joint_axis[1], joint_axis[0]))
        elif joint.get('type')== 'prismatic':
            theta = 0
            d = 0  # will be replaced by input
            a = joint_origin[0]
            alpha = np.rad2deg(np.arctan2(joint_axis[1], joint_axis[0]))
        dh_params.append([theta, d, a, alpha])
    return dh_params

# DH parameters for a 4-DOF crane
def dh_matrix(theta, d, a, alpha):
    """
    Create the Denavit-Hartenberg transformation matrix.
    """
    theta = np.deg2rad(theta)
    alpha = np.deg2rad(alpha)
    return np.array([
        [np.cos(theta), -np.sin(theta) * np.cos(alpha), np.sin(theta) * np.sin(alpha), a * np.cos(theta)],
        [np.sin(theta), np.cos(theta) * np.cos(alpha), -np.cos(theta) * np.sin(alpha), a * np.sin(theta)],
        [0, np.sin(alpha), np.cos(alpha), d],
        [0, 0, 0, 1]
    ])

def compute_forward_kinematics(joint_angles, dh_params):
    """
    joint_angles: [swing rotation (degrees), lift elevation (mm), elbow rotation (degrees), wrist rotation (degrees)]
    """
    theta1, d2, theta3, theta4 = joint_angles
    
    # DH parameters for each joint
    print(dh_params[0][0])
    dh_params[0][0] = theta1
    dh_params[1][1] = d2
    dh_params[2][0] = theta3
    dh_params[3][0] = theta4
    
    # Compute the transformation matrix for each joint
    T = np.eye(4)
    for params in dh_params:
        T = T @ dh_matrix(*params)
    
    return T

# # Read DH parameters from URDF
dh_params = read_urdf_parameters('config/awesome_crane.urdf')

# Compute forward kinematics
joint_angles = [45, 1, 0, 0]
T = compute_forward_kinematics(joint_angles, dh_params)
print("Forward Kinematics Transformation Matrix:")
print(T)
print("End-Effector position:")
ee_position = np.array([0, 0, 0, 1])
print(T.dot(ee_position))