# Copyright 2021 Yu-Kai Lin. All rights reserved.
# Use of this source code is governed by a BSD-style
# license that can be found in the LICENSE file.

import os.path as osp
import sys
from functools import partial

__dir__ = osp.dirname(osp.abspath(__file__))

try:
    pykcp_path = osp.join(__dir__, "../../build/python")
    sys.path.insert(0, pykcp_path)
    import pykcp
except ImportError:
    print("Oops, We cannot import pykcp!")
    print("Make sure that you have properly compiled the python binding of KCP.")
    exit(1)

import numpy as np
import open3d as o3d
from scipy.spatial.transform.rotation import Rotation as R

source_point_cloud_filename = osp.join(__dir__, "data/1531883530.949817000.pcd")
target_point_cloud_filename = osp.join(__dir__, "data/1531883530.449377000.pcd")

snapshot_path = "snapshot.png"


def create_line_by_cylinder(start, end, radius, color):
    length = np.linalg.norm(start - end)
    cylinder = o3d.geometry.TriangleMesh.create_cylinder(radius, length, 8)
    translation = (start + end) / 2
    axis = np.cross((start - end) / length, np.array([0, 0, 1]))
    angle = np.arccos(np.dot((start - end) / length, np.array([0, 0, 1])))
    rotation = R.from_rotvec(angle * axis).as_matrix()
    cylinder.rotate(rotation, center=(0, 0, 0))
    cylinder.translate(translation)
    cylinder.paint_uniform_color(color)
    return cylinder


def create_sphere(position, radius, color):
    sphere = o3d.geometry.TriangleMesh.create_sphere(radius)
    sphere.translate(position)
    sphere.paint_uniform_color(color)
    return sphere


def visualize(
    source,
    target,
    transformation,
    initial_correspondences,
    inlier_correspondence_indices,
):
    source_o3d = o3d.geometry.PointCloud()
    source_o3d.points = o3d.utility.Vector3dVector(source)
    source_transformed_o3d = o3d.geometry.PointCloud()
    source_transformed_o3d.points = o3d.utility.Vector3dVector(source)
    target_o3d = o3d.geometry.PointCloud()
    target_o3d.points = o3d.utility.Vector3dVector(target)

    source_transformed_o3d.transform(transformation)

    source_o3d.colors = o3d.utility.Vector3dVector(
        np.array([[1.0, 0.2, 0.2] for _ in range(source.shape[0])], dtype=float)
    )
    source_transformed_o3d.colors = o3d.utility.Vector3dVector(
        np.array([[1.0, 0.2, 0.2] for _ in range(source.shape[0])], dtype=float)
    )
    target_o3d.colors = o3d.utility.Vector3dVector(
        np.array([[0.2, 0.2, 1.0] for _ in range(target.shape[0])], dtype=float)
    )

    size = initial_correspondences.points[0].shape[1]
    points = list(initial_correspondences.points[0].T)
    points += list(initial_correspondences.points[1].T)
    lines = [[i, i + size] for i in range(size)]
    initial_correspondences_o3d = o3d.geometry.LineSet()
    initial_correspondences_o3d.points = o3d.utility.Vector3dVector(np.array(points))
    initial_correspondences_o3d.lines = o3d.utility.Vector2iVector(
        np.array(lines, dtype=int)
    )
    initial_correspondences_o3d.paint_uniform_color(np.array([0.2, 0.2, 0.2]))
    inlier_correspondence_lines = []
    inlier_correspondence_spheres = []
    for idx in inlier_correspondence_indices:
        start = initial_correspondences.points[0].T[idx]
        end = initial_correspondences.points[1].T[idx]
        color = np.array([0.0, 0.9, 0.0])
        inlier_correspondence_lines.append(
            create_line_by_cylinder(start, end, 0.08, color)
        )
        inlier_correspondence_spheres.append(create_sphere(end, 0.2, color))

    def registration_before_raw(vis):
        vis.clear_geometries()
        vis.add_geometry(source_o3d, False)
        vis.add_geometry(target_o3d, False)
        return False

    def registration_before_raw_with_initial_correspondences(vis):
        registration_before_raw(vis)
        vis.add_geometry(initial_correspondences_o3d, False)
        vis.update_renderer()
        return False

    def registration_before_raw_with_inlier_correspondences(vis):
        registration_before_raw(vis)
        vis.add_geometry(initial_correspondences_o3d, False)
        for line in inlier_correspondence_lines:
            vis.add_geometry(line, False)
        vis.update_renderer()
        return False

    def registration_result_with_inlier_correspondences(vis):
        vis.clear_geometries()
        vis.add_geometry(source_transformed_o3d, False)
        vis.add_geometry(target_o3d, False)
        for sphere in inlier_correspondence_spheres:
            vis.add_geometry(sphere, False)
        return False

    def registration_result_final(vis):
        vis.clear_geometries()
        vis.add_geometry(source_transformed_o3d, False)
        vis.add_geometry(target_o3d, False)
        return False

    def save_snapshot(vis):
        vis.capture_screen_image(snapshot_path)
        return False

    vis = o3d.visualization.VisualizerWithKeyCallback()

    vis.create_window()
    vis.get_render_option().load_from_json("render_option.json")
    vis.register_key_callback(ord("S"), partial(save_snapshot))
    vis.register_key_callback(ord("1"), partial(registration_before_raw))
    vis.register_key_callback(
        ord("2"), partial(registration_before_raw_with_initial_correspondences)
    )
    vis.register_key_callback(
        ord("3"), partial(registration_before_raw_with_inlier_correspondences)
    )
    vis.register_key_callback(
        ord("4"), partial(registration_result_with_inlier_correspondences)
    )
    vis.register_key_callback(ord("5"), partial(registration_result_final))
    vis.add_geometry(source_transformed_o3d)
    vis.add_geometry(target_o3d)

    print()
    print("[Keymap] Key '1'-'5' to see different views, 's' to snapshot, 'q' to quit")
    print("  1: initial clouds")
    print("  2: initial k closet points correspondences")
    print("  3: inlier correspondences by the maximum clique pruning method")
    print("  4: registration result with inlier correspondences")
    print("  5: registration result")

    vis.run()
    vis.destroy_window()

    return vis


def main():
    # 1. Load point clouds.
    source_o3d = o3d.io.read_point_cloud(source_point_cloud_filename)
    target_o3d = o3d.io.read_point_cloud(target_point_cloud_filename)

    source = np.asarray(source_o3d.points)
    target = np.asarray(target_o3d.points)

    # 2. Extract corner points with multi-scale curvature.
    source_corner_points = pykcp.MultiScaleCurvature(source).get_corner_points()
    target_corner_points = pykcp.MultiScaleCurvature(target).get_corner_points()

    # 3. Estimate the transformation of two clouds with KCP-TEASER.
    params = pykcp.KCPParams()
    params.k = 2
    params.teaser.noise_bound = 0.06

    solver = pykcp.KCP(params)
    solver.solve(
        source_corner_points,  # source point clouds
        target_corner_points,  # target point clouds
        source_corner_points,  # source feature clouds
        target_corner_points,  # target feature clouds
    )
    solution = solver.get_solution()
    initial_correspondences = solver.get_initial_correspondences()
    inlier_correspondence_indices = solver.get_inlier_correspondence_indices()

    print(solution)
    visualize(
        source,
        target,
        solution,
        initial_correspondences,
        inlier_correspondence_indices,
    )


if __name__ == "__main__":
    main()
