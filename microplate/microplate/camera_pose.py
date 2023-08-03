import pathlib
import numpy as np
from airo_dataset_tools.data_parsers.pose import Pose
import json 
from airo_spatial_algebra.se3 import SE3Container

json_path = pathlib.Path(__file__).parent / "camera_pose.json"

def get_camera_pose():
    pose = Pose(**json.load(open(json_path)))
    return pose_model_to_homogeneous_matrix(pose)

def pose_model_to_homogeneous_matrix(pose): 
    euler_angles = [pose.rotation_euler_xyz_in_radians.roll,pose.rotation_euler_xyz_in_radians.pitch,pose.rotation_euler_xyz_in_radians.yaw]
    positions = [pose.position_in_meters.x,pose.position_in_meters.y,pose.position_in_meters.z]
    homogeneous_matrix = SE3Container.from_euler_angles_and_translation(euler_angles,positions).homogeneous_matrix
    return homogeneous_matrix
if __name__ == "__main__":
    print(get_camera_pose())