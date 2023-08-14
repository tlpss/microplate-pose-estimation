from typing import List

import numpy as np
import torch
from airo_typing import CameraExtrinsicMatrixType, CameraIntrinsicsMatrixType


def multiview_triangulation_midpoint(
    extrinsics_matrices: List[CameraExtrinsicMatrixType],
    intrinsics_matrices: List[CameraIntrinsicsMatrixType],
    image_points,
):
    """cf. https://ieeexplore.ieee.org/stamp/stamp.jsp?tp=&arnumber=8967077"""

    # determine the rays for each camera in the world frame
    rays = []
    for extrinsics_matrix, intrinsics_matrix, image_point in zip(
        extrinsics_matrices, intrinsics_matrices, image_points
    ):
        ray = (
            extrinsics_matrix[:3, :3]
            @ np.linalg.inv(intrinsics_matrix)
            @ np.array([image_point[0], image_point[1], 1])
        )
        ray = ray / np.linalg.norm(ray)
        rays.append(ray)

    lhs = 0
    rhs = 0
    for i, ray in enumerate(rays):
        rhs += (np.eye(3) - ray[:, np.newaxis] @ ray[np.newaxis, :]) @ extrinsics_matrices[i][:3, 3]
        lhs += np.eye(3) - ray[:, np.newaxis] @ ray[np.newaxis, :]

    lhs_inv = np.linalg.inv(lhs)
    midpoint = lhs_inv @ rhs
    return midpoint


def get_triangulation_errors(
    extrinsics_matrices: List[CameraExtrinsicMatrixType],
    intrinsics_matrices: List[CameraIntrinsicsMatrixType],
    image_points,
    midpoint,
):
    errors = []
    for extrinsics_matrix, intrinsics_matrix, image_point in zip(
        extrinsics_matrices, intrinsics_matrices, image_points
    ):
        ray = (
            extrinsics_matrix[:3, :3]
            @ np.linalg.inv(intrinsics_matrix)
            @ np.array([image_point[0], image_point[1], 1])
        )
        ray = ray / np.linalg.norm(ray)
        error = np.linalg.norm(
            (np.eye(3) - ray[:, np.newaxis] @ ray[np.newaxis, :]) @ ((extrinsics_matrix[:3, 3]) - midpoint)
        )
        errors.append(error)
    return errors


def multiview_triangulation_gradient(
    extrinsics_matrices: List[CameraExtrinsicMatrixType],
    intrinsics_matrices: List[CameraIntrinsicsMatrixType],
    image_points,
):
    """can be used to get better 3D point, but usually the midpoint is good enough so far"""
    midpoint_guess = multiview_triangulation_midpoint(extrinsics_matrices, intrinsics_matrices, image_points)
    print(midpoint_guess)
    midpoint = torch.tensor(midpoint_guess, requires_grad=True)
    extrinsics_matrices = [
        torch.tensor(extrinsics_matrix, requires_grad=False) for extrinsics_matrix in extrinsics_matrices
    ]
    intrinsics_matrices = [
        torch.tensor(intrinsics_matrix, requires_grad=False) for intrinsics_matrix in intrinsics_matrices
    ]
    image_points = [torch.tensor(image_point, requires_grad=False) for image_point in image_points]
    optimizer = torch.optim.Adam([midpoint], lr=0.0003)
    for i in range(10000):
        loss = 0
        # project midpoint to each camera and take L2 distance

        for image_point, extrinsics_matrix, intrinsics_matrix in zip(
            image_points, extrinsics_matrices, intrinsics_matrices
        ):
            projected_image_point = (
                intrinsics_matrix @ (torch.linalg.inv(extrinsics_matrix) @ torch.cat([midpoint, torch.ones(1)]))[:3]
            )
            projected_image_point = projected_image_point / projected_image_point[2]
            loss += torch.norm(image_point - projected_image_point[:2])
        loss.backward()
        # print(loss)
        optimizer.step()
        optimizer.zero_grad()
        # print(midpoint.detach().numpy())
    return midpoint.detach().numpy()
