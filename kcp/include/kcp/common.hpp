// Copyright 2021 Yu-Kai Lin. All rights reserved.
// Use of this source code is governed by a BSD-style
// license that can be found in the LICENSE file.

#pragma once

#include <Eigen/Dense>

#include <vector>

/**
 * @brief Namespace for the KCP library.
 * 
 */
namespace kcp {

/**
 * @brief Data structure for point correspondences.
 * 
 */
struct Correspondences {
  /**
   * @brief Correspondences in terms of position, where the first and the second
   * parts correspond to the source and the target clouds respectively.
   *
   */
  std::pair<Eigen::Matrix3Xd, Eigen::Matrix3Xd> points;

  /**
   * @brief Correspondences in terms of point indices, where the first and the
   * second parts correspond to the source and the target clouds respectively. 
   * 
   */
  std::pair<std::vector<int>, std::vector<int>> indices;
};

};  // namespace kcp
