// Copyright 2021 Yu-Kai Lin. All rights reserved.
// Use of this source code is governed by a BSD-style
// license that can be found in the LICENSE file.

#pragma once

#include "kcp/common.hpp"

#include <Eigen/Core>

#define _USE_MATH_DEFINES
#include <cmath>
#include <map>
#include <memory>
#include <stdexcept>
#include <vector>

#define deg2red(deg) ((deg)*M_PI / 180)
#define red2deg(red) ((red)*180 / M_PI)

#define MAX(A, B) ((A) > (B) ? A : B)
#define MIN(A, B) ((A) < (B) ? A : B)

#define l2Norm2D(x, y) sqrt(pow(x, 2) + pow(y, 2))
#define l2Norm3D(x, y, z) sqrt(pow(x, 2) + pow(y, 2) + pow(z, 2))
#define _GET_L2NORM_MACRO(_1, _2, _3, NAME, ...) NAME
#define l2Norm(...)                                  \
  _GET_L2NORM_MACRO(__VA_ARGS__, l2Norm3D, l2Norm2D) \
  (__VA_ARGS__)

#define CYCLIC_INDEX(i, start, end) ((i < start) ? (end - (start - (i))) : ((i > end) ? start + (i - (end)) : i))

namespace kcp {

/**
 * @brief Get the set of k-closest-points correspondences with kd-tree.
 * 
 * @param src The source point cloud.
 * @param dst The target point cloud.
 * @param src_feature The source feature cloud used to compute distances.
 * @param dst_feature The target feature cloud used to compute distances.
 * @param k The number of closest points for each source point.
 * @return Shared pointer to the set of correspondences.
 */
std::shared_ptr<Correspondences>
get_kcp_correspondences(const Eigen::MatrixX3d& src,
                        const Eigen::MatrixX3d& dst,
                        const Eigen::MatrixXd& src_feature,
                        const Eigen::MatrixXd& dst_feature,
                        size_t k);

};  // namespace kcp
