// Copyright 2021 Yu-Kai Lin. All rights reserved.
// Use of this source code is governed by a BSD-style
// license that can be found in the LICENSE file.

#pragma once

#include "kcp/common.hpp"

namespace kcp {

/**
 * @brief Namespace for the keypoint extraction.
 * 
 */
namespace keypoint {

/**
 * @brief A range image of a point cloud based on the spherical projection.
 * 
 */
class RangeImage {
 protected:
  /**
   * @brief The point cloud.
   * 
   */
  Eigen::MatrixX3d cloud;

  /**
   * @brief Channel indices of raw points.
   * 
   */
  std::vector<int> channel;

  /**
   * @brief The number of channels (height of the range image). It is usually
   * set to be the number of LiDAR beams.
   *
   */
  int n_channels;

  /**
   * @brief The minimum vertical field of view (V-FOV) of the given point cloud.
   * 
   */
  float min_vfov_deg;

  /**
   * @brief The maximum vertical field of view (V-FOV) of the given point cloud.
   * 
   */
  float max_vfov_deg;

  /**
   * @brief The resolution of 360-degree horizontal field of view.
   * 
   */
  int hfov_resolution;

  /**
   * @brief The range image whose entry indicates the raw index of point.
   * 
   */
  Eigen::MatrixXi image_indices;

  /**
   * @brief The depths of points ordered by channels.
   * 
   */
  std::vector<float> image_depth_sequence;

  /**
   * @brief The raw indices of points ordered by channels.
   * 
   */
  std::vector<int> image_point_indices_sequence;

  /**
   * @brief The column (horizontal) indices of points ordered by channels.
   * 
   */
  std::vector<int> image_col_indices_sequence;

  /**
   * @brief The starting index vector of all channels.
   * 
   */
  std::vector<int> channel_start_indices;

  /**
   * @brief The ending index vector of all channels.
   * 
   */
  std::vector<int> channel_end_indices;

  /**
   * @brief Compute the polar index of each point.
   * 
   */
  void calculate_point_cloud_properties();

  /**
   * @brief Compute the corresponding range image of the given point cloud.
   * 
   */
  void calculate_range_image();

 public:
  /**
   * @brief Construct a new RangeImage object.
   *
   * @param cloud The point cloud.
   * @param n_channels The number of channels (height of the range image). It is
   * usually set to be the number of LiDAR beams.
   * @param min_vfov_deg The minimum vertical field of view (V-FOV) of the given
   * point cloud.
   * @param max_vfov_deg The maximum vertical field of view (V-FOV) of the given
   * point cloud.
   * @param hfov_resolution The resolution of 360-degree horizontal field of
   * view.
   */
  RangeImage(Eigen::MatrixX3d cloud,
             int n_channels      = 32,
             float min_vfov_deg  = -30.0,
             float max_vfov_deg  = 10.0,
             int hfov_resolution = 1800);

  /**
   * @brief Get the point cloud.
   * 
   * @return const Eigen::MatrixX3d& 
   */
  const Eigen::MatrixX3d &get_cloud() const { return this->cloud; }

  /**
   * @brief Get the number of channels.
   * 
   * @return int
   */
  int get_n_channels() const { return this->n_channels; }

  /**
   * @brief Get the size of parameterized points for the range image.
   * 
   * @return size_t 
   */
  size_t get_image_sequence_size() const { return this->image_depth_sequence.size(); }

  /**
   * @brief Get the range image whose entry indicates the raw index of point.
   * 
   * @return const Eigen::MatrixXi& 
   */
  const Eigen::MatrixXi &get_image_indices() const { return this->image_indices; }

  /**
   * @brief Get the depths of points ordered by channels.
   * 
   * @return const std::vector<float>& 
   */
  const std::vector<float> &get_image_depth_sequence() const { return this->image_depth_sequence; }

  /**
   * @brief Get the raw indices of points ordered by channels.
   * 
   * @return const std::vector<int>& 
   */
  const std::vector<int> &get_image_point_indices_sequence() const { return this->image_point_indices_sequence; }

  /**
   * @brief Get the column (horizontal) indices of points ordered by channels.
   * 
   * @return const std::vector<int>& 
   */
  const std::vector<int> &get_image_col_indices_sequence() const { return this->image_col_indices_sequence; }

  /**
   * @brief Get the starting index vector of all channels.
   * 
   * @return const std::vector<int>& 
   */
  const std::vector<int> &get_channel_start_indices() const { return this->channel_start_indices; }

  /**
   * @brief Get the ending index vector of all channels.
   * 
   * @return const std::vector<int>& 
   */
  const std::vector<int> &get_channel_end_indices() const { return this->channel_end_indices; }
};

/**
 * @brief The multi-scale curvature class for extracting corner points and plane
 * points based on the range image.
 * 
 * @see RangeImage The range image class.
 *
 */
class MultiScaleCurvature {
 public:
  /**
   * @brief Enum class of point labels.
   * 
   */
  enum class Label {
    UNDEFINED,
    NORMAL,
    OCCLUDED,
    PARALLEL,
    CORNER,
    PLANE,
    AMBIGUOUS
  };

 protected:
  /**
   * @brief The range image.
   * 
   */
  RangeImage range_image;

  /**
   * @brief Multi-scale curvatures stored as a vector of {kappa, index}.
   * 
   */
  std::vector<std::pair<float, int>> curvature;

  /**
   * @brief Labels of the points.
   * 
   */
  std::vector<Label> label;

  /**
   * @brief The threshold (lower-bound of multi-scale curvature) to determine if
   * the point is a corner point.
   * 
   */
  float corner_threshold;

  /**
   * @brief The threshold (upper-bound of multi-scale curvature) to determine if
   * the point is a plane point.
   * 
   */
  float plane_threshold;

  /**
   * @brief Corner points in terms of position.
   * 
   */
  Eigen::MatrixX3d corner_points;

  /**
   * @brief Plane points in terms of position.
   * 
   */
  Eigen::MatrixX3d plane_points;

  /**
   * @brief Corner points in terms of their indices.
   * 
   */
  std::vector<int> corner_point_indices;

  /**
   * @brief Plane points in terms of their indices.
   * 
   */
  std::vector<int> plane_point_indices;

  /**
   * @brief Compute the multi-scale curvature and choose corner points and plane
   * points.
   * 
   */
  void calculate_multi_scale_curvature();

 public:
  /**
   * @brief Construct a new MultiScaleCurvature object.
   *
   * @param range_image The pre-computed range image.
   * @param corner_threshold The threshold (lower-bound of multi-scale
   * curvature) to determine if the point is a corner point.
   * @param plane_threshold The threshold (upper-bound of multi-scale curvature)
   * to determine if the point is a plane point.
   */
  MultiScaleCurvature(RangeImage range_image, float corner_threshold = 30.0, float plane_threshold = 0.1);

  /**
   * @brief Construct a new MultiScaleCurvature object. The corresponding range
   * image will be computed within the constructor.
   * 
   * @param cloud The point cloud.
   * @param n_channels The number of channels (height of the range image). It is
   * usually set to be the number of LiDAR beams.
   * @param min_vfov_deg The minimum vertical field of view (V-FOV) of the given
   * point cloud.
   * @param max_vfov_deg The maximum vertical field of view (V-FOV) of the given
   * point cloud.
   * @param hfov_resolution The resolution of 360-degree horizontal field of
   * view.
   * @param corner_threshold The threshold (lower-bound of multi-scale
   * curvature) to determine if the point is a corner point.
   * @param plane_threshold The threshold (upper-bound of multi-scale curvature)
   * to determine if the point is a plane point.
   */
  MultiScaleCurvature(Eigen::MatrixX3d cloud,
                      int n_channels         = 32,
                      float min_vfov_deg     = -30.0,
                      float max_vfov_deg     = 10.0,
                      int hfov_resolution    = 1800,
                      float corner_threshold = 30.0,
                      float plane_threshold  = 0.1);

  /**
   * @brief Get the range image.
   * 
   * @return const RangeImage& 
   */
  const RangeImage &get_range_image() const { return this->range_image; }

  /**
   * @brief Get the corner points in terms of position.
   * 
   * @return const Eigen::MatrixX3d& 
   */
  const Eigen::MatrixX3d &get_corner_points() const { return this->corner_points; }

  /**
   * @brief Get the plane points in terms of position.
   * 
   * @return const Eigen::MatrixX3d& 
   */
  const Eigen::MatrixX3d &get_plane_points() const { return this->plane_points; }

  /**
   * @brief Get the corner points in terms of their indices.
   * 
   * @return const std::vector<int>& 
   */
  const std::vector<int> &get_corner_point_indices() const { return this->corner_point_indices; }

  /**
   * @brief Get the plane points in terms of their indices.
   * 
   * @return const std::vector<int>& 
   */
  const std::vector<int> &get_plane_point_indices() const { return this->plane_point_indices; }

  /**
   * @brief Get the multi-scale curvatures.
   * 
   * @return const std::vector<std::pair<float, int>>& 
   */
  const std::vector<std::pair<float, int>> &get_curvature() const { return this->curvature; }
};

};  // namespace keypoint

};  // namespace kcp
