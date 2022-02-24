// Copyright 2021 Yu-Kai Lin. All rights reserved.
// Use of this source code is governed by a BSD-style
// license that can be found in the LICENSE file.

#include <kcp/keypoint.hpp>
#include <kcp/solver.hpp>

#include <pcl/io/pcd_io.h>

#include <iostream>

/**
 * @brief Convert ``pcl::PointCloud`` to ``Eigen::MatrixX3d``.
 * 
 * @param cloud The point cloud of type ``pcl::PointCloud``
 * @param matrix The point cloud of type ``Eigen::MatrixX3d``.
 */
void convert_pcl_point_cloud_to_eigen_matrix(pcl::PointCloud<pcl::PointXYZI>& cloud,
                                             Eigen::MatrixX3d& matrix) {
  matrix.setZero(cloud.size(), 3);
  for (int idx = 0; idx < cloud.size(); ++idx) {
    const auto& point = cloud[idx];
    matrix(idx, 0)    = point.x;
    matrix(idx, 1)    = point.y;
    matrix(idx, 2)    = point.z;
  }
}

/**
 * @brief Point cloud preprocessing for the nuScenes data.
 *
 * @details This preprocessing removes ground points (using a very simple
 * condition that z-axis <= -1.5m) and points coming from the inspector.
 *
 * @param cloud The point cloud.
 */
void preprocessing(pcl::PointCloud<pcl::PointXYZI>* cloud) {
  pcl::PointCloud<pcl::PointXYZI> filtered_cloud;

  for (const auto& point : *cloud) {
    // remove points comes from the inspector
    bool&& cond_x = point.x > 0.62 || point.x < -0.62;
    bool&& cond_y = point.y > 1.87 || point.y < -1.10;
    if (!cond_x && !cond_y) continue;

    // remove ground points
    bool&& cond_z = point.z > -1.5;
    if (!cond_z) continue;

    filtered_cloud.push_back(point);
  }

  *cloud = filtered_cloud;
}

int main() {
  /**
   * 1. Load point clouds and convert their types to Eigen::MatrixX3d.
   */
  std::string source_point_cloud_filename = "data/1531883530.949817000.pcd";
  std::string target_point_cloud_filename = "data/1531883530.449377000.pcd";

  pcl::PointCloud<pcl::PointXYZI> source_pcl_cloud;
  pcl::PointCloud<pcl::PointXYZI> target_pcl_cloud;

  pcl::io::loadPCDFile(source_point_cloud_filename, source_pcl_cloud);
  pcl::io::loadPCDFile(target_point_cloud_filename, target_pcl_cloud);

  preprocessing(&source_pcl_cloud);
  preprocessing(&target_pcl_cloud);

  Eigen::MatrixX3d source;
  Eigen::MatrixX3d target;

  convert_pcl_point_cloud_to_eigen_matrix(source_pcl_cloud, source);
  convert_pcl_point_cloud_to_eigen_matrix(target_pcl_cloud, target);

  /**
   * 2. Extract corner points with multi-scale curvature.
   */
  Eigen::MatrixX3d source_corner_points = kcp::keypoint::MultiScaleCurvature(source).get_corner_points();
  Eigen::MatrixX3d target_corner_points = kcp::keypoint::MultiScaleCurvature(target).get_corner_points();

  /**
   * 3. Estimate the transformation of two clouds with KCP-TEASER.
   */
  auto params = kcp::KCP::Params();

  params.k                  = 2;
  params.teaser.noise_bound = 0.06;

  auto solver = kcp::KCP(params);
  solver.solve(source_corner_points, target_corner_points,   // source and target point clouds
               source_corner_points, target_corner_points);  // source and target feature clouds
  auto solution = solver.get_solution();
  std::cout << solution << '\n';

  return 0;
}