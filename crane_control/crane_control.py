import numpy as np
from scipy.spatial.transform import Rotation
from ikpy.chain import Chain


np.set_printoptions(precision=3, suppress=True)

# # Read DH parameters from URDF
crane = Chain.from_urdf_file("config/awesome_crane.urdf", active_links_mask=[False, True, True, True, True])
# print(crane.links)

print("==================")
# Compute forward kinematics
print("Forward Kinematics")
joint_angles = ['', np.deg2rad(45), 1, np.deg2rad(45), np.deg2rad(-45)]
print(f"Joint angles: {joint_angles[1:]}")

ee_position = np.array([0, 0, 0, 1])

F2 = crane.forward_kinematics(joint_angles)

print(f"End-Effector position: {F2[:3, 3]}")

rot_matrix = F2[:3, :3]
r = Rotation.from_matrix(rot_matrix)
yaw, pitch, roll = r.as_euler('zyx')
print(f"End-Effector orientation: {yaw, pitch, roll}")


print("==================")
# Compute inverse kinematics
print("Inverse Kinematics")
end_effector_pos = np.array([1.0, 0.0, 0.0])
end_effector_orientation = [np.deg2rad(0), np.deg2rad(0), np.deg2rad(0)]

print(f"End-effector position: {end_effector_pos}")
print(f"End-effector orientation: {end_effector_orientation}")

calculated_joint_angles = crane.inverse_kinematics_frame(F2, orientation_mode="all")
print(f"Joint angles: {calculated_joint_angles}")
