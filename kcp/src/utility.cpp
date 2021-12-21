// Copyright 2021 Yu-Kai Lin. All rights reserved.
// Use of this source code is governed by a BSD-style
// license that can be found in the LICENSE file.

#include "kcp/utility.hpp"

#include <nanoflann.hpp>

namespace kcp {

std::shared_ptr<Correspondences>
get_kcp_correspondences(const Eigen::MatrixX3d& src,
                        const Eigen::MatrixX3d& dst,
                        const Eigen::MatrixXd& src_feature,
                        const Eigen::MatrixXd& dst_feature,
                        size_t k) {
  assert(src_feature.cols() == dst_feature.cols() && "Incompatible dimensions of src_feature and dst_feature");
  assert(src.rows() == src_feature.rows() && "Mismatching sizes of src and src_feature");
  assert(dst.rows() == dst_feature.rows() && "Mismatching sizes of dst and dst_feature");

  int dim                 = src_feature.cols();
  int size                = MIN(k, dst.rows());
  auto correspondences    = std::make_shared<Correspondences>();
  correspondences->points = std::make_pair(Eigen::Matrix3Xd::Zero(3, src.rows() * size),
                                           Eigen::Matrix3Xd::Zero(3, src.rows() * size));
  correspondences->indices.first.reserve(src.rows() * size);
  correspondences->indices.second.reserve(src.rows() * size);

  int index = 0;
  if (k >= dst.rows()) {
    // Equivalent to cross product of two clouds
    for (int src_index = 0; src_index < src.rows(); ++src_index) {
      for (int dst_index = 0; dst_index < dst.rows(); ++dst_index) {
        correspondences->points.first.col(index) << src(src_index, 0), src(src_index, 1), src(src_index, 2);
        correspondences->points.second.col(index) << dst(dst_index, 0), dst(dst_index, 1), dst(dst_index, 2);
        correspondences->indices.first.push_back(src_index);
        correspondences->indices.second.push_back(dst_index);
        ++index;
      }
    }
  } else {
    // Build the KD-tree of dst_feature, and query k closest points for each
    // source point.
    nanoflann::KDTreeEigenMatrixAdaptor<Eigen::MatrixXd> dst_tree(dim, std::cref(dst_feature), 10);
    dst_tree.index->buildIndex();

    std::vector<double> point(dim);
    std::vector<size_t> indices(k);
    std::vector<double> distances(k);

    nanoflann::KNNResultSet<double> result(k);

    for (int src_index = 0; src_index < src.rows(); ++src_index) {
      for (int i = 0; i < dim; ++i) {
        point[i] = src_feature.row(src_index)(i);
      }
      result.init(&indices[0], &distances[0]);

      dst_tree.index->findNeighbors(result, &point[0], nanoflann::SearchParams());

      for (int i = 0; i < size; ++i) {
        int dst_index = indices[i];
        correspondences->points.first.col(index) << src(src_index, 0), src(src_index, 1), src(src_index, 2);
        correspondences->points.second.col(index) << dst(dst_index, 0), dst(dst_index, 1), dst(dst_index, 2);
        correspondences->indices.first.push_back(src_index);
        correspondences->indices.second.push_back(dst_index);
        ++index;
      }
    }
  }

  return correspondences;
}

};  // namespace kcp