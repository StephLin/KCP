// Copyright 2021 Yu-Kai Lin. All rights reserved.
// Use of this source code is governed by a BSD-style
// license that can be found in the LICENSE file.

#include "kcp/solver.hpp"
#include "kcp/utility.hpp"

#include <iostream>

namespace kcp {

/* ----------------------------------- KCP ---------------------------------- */

void KCP::solve(const Eigen::MatrixX3d& src,
                const Eigen::MatrixX3d& dst,
                const Eigen::MatrixXd& src_feature,
                const Eigen::MatrixXd& dst_feature) {
  // Generate initial guess of correspondences with k closest points
  auto correspondences = get_kcp_correspondences(src,
                                                 dst,
                                                 src_feature,
                                                 dst_feature,
                                                 this->params.k);

  // Store the initial k closest points correspondences
  this->initial_correspondences = *correspondences;

  // Trigger the TEASER++ solver, where the maximum clique pruning will be
  // executed within the solver
  if (!this->params.verbose) std::cout.setstate(std::ios_base::failbit);
  this->solver.solve(correspondences->points.first,
                     correspondences->points.second);
  if (!this->params.verbose) std::cout.clear();

  // Extract the estimation result
  auto solution                    = this->solver.getSolution();
  this->solution                   = Eigen::Matrix4d::Identity();
  this->solution.block<3, 3>(0, 0) = solution.rotation;
  this->solution.block<3, 1>(0, 3) = solution.translation;

  // Store the inlier correspondence indices provided by the maximum clique
  // pruning algorithm
  this->inlier_correspondence_indices = this->solver.getInlierMaxClique();
}

};  // namespace kcp