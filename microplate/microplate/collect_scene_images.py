import datetime
import json

import cv2
import numpy as np
from airo_camera_toolkit.cameras.realsense import Realsense
from airo_camera_toolkit.utils import ImageConverter
from airo_dataset_tools.data_parsers.pose import EulerAngles, Pose, Position
from airo_robots.manipulators.hardware.ur_rtde import URrtde
from airo_spatial_algebra.se3 import SE3Container

from microplate.camera_pose import get_camera_pose

if __name__ == "__main__":
    camera_pose_in_tcp = get_camera_pose()
    camera = Realsense(resolution=Realsense.RESOLUTION_720, fps=6)
    robot = URrtde("10.42.0.162", URrtde.UR3E_CONFIG)

    home_pose = SE3Container.from_euler_angles_and_translation([np.pi, 0, 0], [0.3, 0.0, 0.2]).homogeneous_matrix
    robot.move_linear_to_tcp_pose(home_pose, 0.1).wait()

    target_position = np.array([0.34, -0.01, 0.0])  # approximate center of the microplate

    for z in np.arange(0.2, 0.31, 0.05):
        # avoid self-collision while moving from one half to other.
        robot.move_linear_to_tcp_pose(home_pose, 0.1).wait()

        for y in np.arange(-0.15, 0.151, 0.05):
            for x in np.arange(0.3, 0.351, 0.05):
                if z == 0.3 and x == 0.35:
                    # filter unreachable poses.
                    continue

                camera_position = np.array([x, y, z])
                print(camera_position)
                robot_z_vector = target_position - camera_position
                robot_z_vector /= np.linalg.norm(robot_z_vector)
                robot_x_vector = np.cross(robot_z_vector, np.array([0, 0, 1]))
                robot_x_vector /= np.linalg.norm(robot_x_vector)
                robot_y_vector = np.cross(robot_z_vector, robot_x_vector)
                robot_y_vector /= np.linalg.norm(robot_y_vector)
                camera_orientation = np.eye(3)
                camera_orientation[:, 0] = robot_x_vector
                camera_orientation[:, 1] = robot_y_vector
                camera_orientation[:, 2] = robot_z_vector

                camera_pose = SE3Container.from_rotation_matrix_and_translation(
                    camera_orientation, camera_position
                ).homogeneous_matrix
                tcp_pose = camera_pose @ np.linalg.inv(camera_pose_in_tcp)
                print(tcp_pose)
                if not robot.is_tcp_pose_reachable(tcp_pose):
                    print("Not reachable")
                    continue
                robot.move_linear_to_tcp_pose(tcp_pose, 0.05).wait()
                image = camera.get_rgb_image()
                image = ImageConverter.from_numpy_format(image).image_in_opencv_format
                image_name = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")

                cv2.imwrite(f"scene_images/{image_name}.png", image)

                pose_se3 = SE3Container.from_homogeneous_matrix(camera_pose)
                x, y, z = pose_se3.translation
                roll, pitch, yaw = pose_se3.orientation_as_euler_angles

                pose_saveable = Pose(
                    position_in_meters=Position(x=x, y=y, z=z),
                    rotation_euler_xyz_in_radians=EulerAngles(roll=roll, pitch=pitch, yaw=yaw),
                )
                with open(f"scene_images/{image_name}_camera_pose.json", "w") as f:
                    json.dump(pose_saveable.dict(), f, indent=4)
                np.save(f"scene_images/{image_name}_camera_intrinsics.npy", camera.intrinsics_matrix())
