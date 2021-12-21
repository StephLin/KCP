// Copyright 2021 Yu-Kai Lin. All rights reserved.
// Use of this source code is governed by a BSD-style
// license that can be found in the LICENSE file.

#pragma once

#include "kcp/common.hpp"

#include <teaser/registration.h>

namespace kcp {

/**
 * @brief Abstract class for point cloud registration solvers.
 * 
 */
class AbstractSolver {
 protected:
  /**
   * @brief The estimation result.
   * 
   */
  Eigen::Matrix4d solution;

 public:
  /**
   * @brief Constructor.
   * 
   */
  AbstractSolver() : solution(Eigen::Matrix4d::Identity()){};

  /**
   * @brief Abstract function to trigger a registration method.
   * 
   * @param src The source point cloud.
   * @param dst The target point cloud.
   * @param src_feature The source feature cloud.
   * @param dst_feature The target feature cloud.
   */
  virtual void solve(const Eigen::MatrixX3d& src,
                     const Eigen::MatrixX3d& dst,
                     const Eigen::MatrixXd& src_feature,
                     const Eigen::MatrixXd& dst_feature) = 0;

  /**
   * @brief Get the solution of the registration.
   * 
   * @return const Eigen::Matrix4d& 
   */
  virtual const Eigen::Matrix4d& get_solution() const { return this->solution; }
};

/**
 * @brief The KCP-TEASER registration approach.
 *
 * @details KCP is an efficient and effective local point cloud registration
 * approach targeting for real-world 3D LiDAR scan matching problem. A simple
 * (and naive) understanding is: ICP iteratively considers the closest point of
 * each source point, but KCP considers the k closest points of each source
 * point in the beginning, and outlier correspondences are mainly rejected by
 * the maximum clique pruning method (provided by TEASER++).
 *
 * @see Yu-Kai Lin, Wen-Chieh Lin, Chieh-Chih Wang, **KCP: k-Closest Points and
 * Maximum Clique Pruning for Efficient and Effective 3D Laser Scan Matching**.
 * To appear in _IEEE Robotics and Automation Letters (RA-L)_, 2022.
 *
 */
class KCP : public AbstractSolver {
 public:
  /**
   * @brief Type alias of the TEASER++ solver.
   * 
   */
  using TEASER = teaser::RobustRegistrationSolver;

  /**
   * @brief Type of parameters for the KCP-TEASER solver.
   * 
   */
  struct Params {
    /**
     * @brief Parameters for the original TEASER++ solver. The most frequently
     * modified parameter in KCP-TEASER is ``noise_bound``.
     * 
     * @see https://teaser.readthedocs.io/en/latest/api-cpp.html#_CPPv4N6teaser24RobustRegistrationSolver6ParamsE
     * 
     */
    TEASER::Params teaser;

    /**
     * @brief The number of closest points for each source point. Default by 2.
     * 
     */
    size_t k;

    /**
     * @brief Construct a new KCP::Params object.
     * 
     */
    Params() {
      k                                    = 2;
      teaser.noise_bound                   = 0.06;
      teaser.cbar2                         = 1;
      teaser.estimate_scaling              = false;
      teaser.rotation_gnc_factor           = 1.4;
      teaser.rotation_estimation_algorithm = TEASER::ROTATION_ESTIMATION_ALGORITHM::GNC_TLS;
      teaser.rotation_max_iterations       = 100;
      teaser.rotation_cost_threshold       = 1e-6;
      teaser.kcore_heuristic_threshold     = 0.5;
      teaser.use_max_clique                = true;
      teaser.max_clique_exact_solution     = true;
      teaser.max_clique_time_limit         = 3600;
    }
  };

 protected:
  /**
   * @brief The TEASER++ solver.
   * 
   */
  TEASER solver;

  /**
   * @brief Parameters for the KCP-TEASER solver.
   * 
   */
  KCP::Params params;

  /**
   * @brief The initial set of k-closest-points correspondences.
   * 
   */
  Correspondences initial_correspondences;

  /**
   * @brief The inlier correspondence indices with respect to
   * ``initial_correspondences``. The set is estimated by the maximum clique
   * pruning method.
   * 
   */
  std::vector<int> inlier_correspondence_indices;

 public:
  /**
   * @brief Construct a new KCP object.
   * 
   * @param params KCP-TEASER parameters.
   */
  KCP(KCP::Params params) : params(params), solver(params.teaser) {}

  /**
   * @brief Get the parameters.
   * 
   * @return Parameters.
   */
  KCP::Params& get_params() { return this->params; }

  /**
   * @brief Get the initial set of correspondences.
   * 
   * @return const Correspondences& Initial set of correspondences.
   */
  const Correspondences& get_initial_correspondences() const { return this->initial_correspondences; }

  /**
   * @brief Get the inlier correspondence indices.
   * 
   * @return const std::vector<int>& Inlier correspondence indices.
   */
  const std::vector<int>& get_inlier_correspondence_indices() const { return this->inlier_correspondence_indices; }

  /**
   * @brief The main function to trigger the KCP-TEASER registration approach.
   * 
   * @param src The source point cloud.
   * @param dst The target point cloud.
   * @param src_feature The source feature cloud.
   * @param dst_feature The target feature cloud.
   */
  virtual void solve(const Eigen::MatrixX3d& src,
                     const Eigen::MatrixX3d& dst,
                     const Eigen::MatrixXd& src_feature,
                     const Eigen::MatrixXd& dst_feature) override;
};

};  // namespace kcp
