if __name__ == "__main__":

    import numpy as np
    from airo_camera_toolkit.cameras.realsense import Realsense
    from airo_camera_toolkit.reprojection import reproject_to_frame_z_plane
    from airo_robots.manipulators.hardware.ur_rtde import URrtde
    from airo_spatial_algebra.se3 import SE3Container

    from microplate.camera_pose import get_camera_pose
    from microplate.opencv_annotation import Annotation, get_manual_annotations

    camera = Realsense(resolution=Realsense.RESOLUTION_720, fps=10)
    robot = URrtde("10.42.0.162", URrtde.UR3E_CONFIG)

    camera_in_tcp_pose = get_camera_pose()

    start_pose = SE3Container.from_euler_angles_and_translation([np.pi, 0, 0], [0.3, 0.0, 0.2]).homogeneous_matrix
    print(start_pose)

    robot.move_linear_to_tcp_pose(start_pose, 0.1).wait()
    image = camera.get_rgb_image()
    anns = get_manual_annotations(camera, {"keypoint": Annotation.Keypoint})
    keypoint = anns["keypoint"]
    keypoint = np.array(keypoint)
    keypoint = keypoint[np.newaxis, :]
    print(keypoint)

    tcp_pose_in_base = robot.get_tcp_pose()
    camera_pose_in_base = tcp_pose_in_base @ camera_in_tcp_pose

    keypoint_position_in_base = reproject_to_frame_z_plane(keypoint, camera.intrinsics_matrix(), camera_pose_in_base)[
        0
    ]
    print(keypoint_position_in_base)
    input("Press enter to move to the keypoint position")

    target_pose = np.copy(start_pose)
    target_pose[:3, 3] = keypoint_position_in_base
    target_pose[2, 3] += 0.01  # move 2cm up, to avoid collision with the table
    robot.move_linear_to_tcp_pose(target_pose, 0.1).wait()
