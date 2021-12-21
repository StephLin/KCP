// Copyright 2021 Yu-Kai Lin. All rights reserved.
// Use of this source code is governed by a BSD-style
// license that can be found in the LICENSE file.

#include "kcp/keypoint.hpp"
#include "kcp/utility.hpp"

#include <limits>

namespace kcp {

namespace keypoint {

/* ------------------------------- RangeImage ------------------------------- */

RangeImage::RangeImage(Eigen::MatrixX3d cloud,
                       int n_channels,
                       float min_vfov_deg,
                       float max_vfov_deg,
                       int hfov_resolution)
    : cloud(cloud),
      n_channels(n_channels),
      min_vfov_deg(min_vfov_deg),
      max_vfov_deg(max_vfov_deg),
      hfov_resolution(hfov_resolution) {
  this->channel.reserve(cloud.rows());

  this->image_indices = Eigen::MatrixXi::Ones(n_channels, hfov_resolution) * -1;
  this->image_depth_sequence.reserve(this->cloud.rows());
  this->image_point_indices_sequence.reserve(this->cloud.rows());
  this->image_col_indices_sequence.reserve(this->cloud.rows());

  this->calculate_point_cloud_properties();
  this->calculate_range_image();
}

/* -------------------------------------------------------------------------- */

void RangeImage::calculate_point_cloud_properties() {
  float delta_fov = deg2red(this->max_vfov_deg - this->min_vfov_deg) / this->n_channels;
  float base_fov  = deg2red(this->min_vfov_deg);

  // calculating index of v-fov and add it to channel vector
  for (int idx = 0; idx < cloud.rows(); ++idx) {
    const auto &point = cloud.row(idx);
    float &&xy_norm   = l2Norm(point(0), point(1));
    float &&phi       = atan2(point(2), xy_norm);
    int &&channel_idx = static_cast<int>(MIN(MAX((phi - base_fov) / delta_fov, 0), this->n_channels - 1));
    this->channel.push_back(channel_idx);
  }
}

/* -------------------------------------------------------------------------- */

void RangeImage::calculate_range_image() {
  // calculating index of h-fov and set to range image
  for (size_t idx = 0; idx < this->cloud.rows(); ++idx) {
    const auto &point = cloud.row(idx);
    float &&theta     = MAX(atan2(point(1), point(0)) + M_PI, 0);
    int &&thetaIdx    = static_cast<int>(theta * this->hfov_resolution / (2 * M_PI)) % this->hfov_resolution;
    if (this->image_indices(this->channel[idx], thetaIdx) < 0) {
      this->image_indices(this->channel[idx], thetaIdx) = idx;
    }
  }

  // ordering point sequence and calculate depth information
  int counter = 0;
  int idx;
  for (size_t channel_idx = 0; channel_idx < this->n_channels; ++channel_idx) {
    // adding start indices to start vector
    this->channel_start_indices.push_back(counter);

    // looping vertices in one channel with order of h-fov
    for (size_t h_idx = 0; h_idx < this->hfov_resolution; ++h_idx) {
      if (this->image_indices(channel_idx, h_idx) >= 0) {
        ++counter;

        idx = this->image_indices(channel_idx, h_idx);
        // adding vertex index of point cloud to sequence
        this->image_point_indices_sequence.push_back(idx);

        // adding vertex depth of this vertex to sequence
        const auto &point = this->cloud.row(idx);
        float &&depth     = l2Norm(point(0), point(1), point(2));
        this->image_depth_sequence.push_back(depth);
        this->image_col_indices_sequence.push_back(h_idx);
      }
    }

    // adding end indices to end vector
    this->channel_end_indices.push_back(counter - 1);
  }
}

/* --------------------------- MultiScaleCurvature -------------------------- */

MultiScaleCurvature::MultiScaleCurvature(RangeImage range_image,
                                         float corner_threshold,
                                         float plane_threshold)
    : range_image(range_image),
      corner_threshold(corner_threshold),
      plane_threshold(plane_threshold) {
  this->curvature.assign(this->range_image.get_image_sequence_size(),
                         {std::numeric_limits<float>::max(), -1});
  this->label.assign(this->range_image.get_image_sequence_size(), Label::UNDEFINED);
  this->calculate_multi_scale_curvature();
}

/* -------------------------------------------------------------------------- */

MultiScaleCurvature::MultiScaleCurvature(Eigen::MatrixX3d cloud,
                                         int n_channels,
                                         float min_vfov_deg,
                                         float max_vfov_deg,
                                         int hfov_resolution,
                                         float corner_threshold,
                                         float plane_threshold)
    : range_image(cloud,
                  n_channels,
                  min_vfov_deg,
                  max_vfov_deg,
                  hfov_resolution),
      corner_threshold(corner_threshold),
      plane_threshold(plane_threshold) {
  this->curvature.assign(this->range_image.get_image_sequence_size(),
                         {std::numeric_limits<float>::max(), -1});
  this->label.assign(this->range_image.get_image_sequence_size(), Label::UNDEFINED);
  this->calculate_multi_scale_curvature();
}

/* -------------------------------------------------------------------------- */

void MultiScaleCurvature::calculate_multi_scale_curvature() {
  const std::vector<float> &image_depth = this->range_image.get_image_depth_sequence();

  /**
   * Calculate curvature
   */
  int sc, ec;  // start and end indices
  float weight = 1 + .5 + 1. / 3. + .25 + .2;
  for (size_t k = 0; k < this->range_image.get_n_channels(); ++k) {
    sc = this->range_image.get_channel_start_indices()[k];
    ec = this->range_image.get_channel_end_indices()[k];

    if (sc >= ec - 15)
      continue;

    for (size_t i = sc; i <= ec; ++i) {
      // clang-format off
      float &&c = image_depth[CYCLIC_INDEX(i - 5, sc, ec)] / 5 + image_depth[CYCLIC_INDEX(i - 4, sc, ec)] / 4
                + image_depth[CYCLIC_INDEX(i - 3, sc, ec)] / 3 + image_depth[CYCLIC_INDEX(i - 2, sc, ec)] / 2
                + image_depth[CYCLIC_INDEX(i - 1, sc, ec)] - image_depth[i] * 2 * weight
                + image_depth[CYCLIC_INDEX(i + 1, sc, ec)] + image_depth[CYCLIC_INDEX(i + 2, sc, ec)] / 2
                + image_depth[CYCLIC_INDEX(i + 3, sc, ec)] / 3 + image_depth[CYCLIC_INDEX(i + 4, sc, ec)] / 4
                + image_depth[CYCLIC_INDEX(i + 5, sc, ec)] / 5;
      // clang-format on
      this->curvature[i] = {std::abs(c), i};
      this->label[i]     = Label::NORMAL;
    }
  }

  /**
   * Extract features
   */
  this->corner_point_indices.clear();
  this->plane_point_indices.clear();
  this->corner_point_indices.reserve(500);
  this->plane_point_indices.reserve(20000);

  auto cloud = this->range_image.get_cloud();

  int sp, ep;      // start and end segment indices
  int counter;     // counter for max number of feature points
  int idx;         // index variable for vertex
  int rIdx, lIdx;  // left and right indices for ambiguity condition
  for (size_t i = 0; i < this->range_image.get_n_channels(); ++i) {
    sc = this->range_image.get_channel_start_indices()[i];
    ec = this->range_image.get_channel_end_indices()[i];

    if (sc >= ec - 20)
      continue;

    for (size_t j = 0; j < 6; ++j) {
      // Partial range (start index and end index of a partial segment within a channel)
      sp = (sc * (6 - j) + ec * j) / 6;
      ep = (sc * (5 - j) + ec * (j + 1)) / 6 + 1;

      std::sort(this->curvature.begin() + sp, this->curvature.begin() + ep);
      counter = 0;
      // Handling edge features
      for (int k = ep - 1; k >= sp; --k) {
        idx = this->curvature[k].second;

        if (this->label[idx] == Label::NORMAL && this->curvature[k].first > this->corner_threshold) {
          ++counter;
          // Add vertex to set of edge features
          if (counter <= 12) {
            this->label[idx] = Label::CORNER;
            this->corner_point_indices.push_back(this->range_image.get_image_point_indices_sequence()[idx]);
          } else {
            break;
          }

          // Mark neighbor vertices as ambiguity
          // .. Right hand side
          for (int l = 1; l <= 5; ++l) {
            if (idx + l >= this->range_image.get_image_col_indices_sequence().size()) {
              break;
            }
            rIdx = this->range_image.get_image_col_indices_sequence()[idx + l];
            lIdx = this->range_image.get_image_col_indices_sequence()[idx + l - 1];

            int &&columnDiff = std::abs(int(rIdx - lIdx));
            if (columnDiff > 10)
              break;

            this->label[rIdx] = Label::AMBIGUOUS;
          }
          // .. Left hand side
          for (int l = -1; l >= -5; --l) {
            if (idx + l < 0) {
              break;
            }
            rIdx = this->range_image.get_image_col_indices_sequence()[idx + l + 1];
            lIdx = this->range_image.get_image_col_indices_sequence()[idx + l];

            int &&columnDiff = std::abs(int(rIdx - lIdx));
            if (columnDiff > 10)
              break;

            this->label[lIdx] = Label::AMBIGUOUS;
          }
        }
      }

      // Handle plane features
      for (int k = ep - 1; k >= sp; --k) {
        idx = this->curvature[k].second;
        if (this->label[idx] == Label::NORMAL && this->curvature[k].first < this->plane_threshold) {
          // add vertex to set of plane features
          this->label[idx] = Label::PLANE;
          this->plane_point_indices.push_back(this->range_image.get_image_point_indices_sequence()[idx]);

          // mark neighbor vertices as ambiguity
          // right hand side
          for (int l = 1; l <= 5; ++l) {
            if (idx + l >= this->range_image.get_image_col_indices_sequence().size()) {
              break;
            }
            rIdx = this->range_image.get_image_col_indices_sequence()[idx + l];
            lIdx = this->range_image.get_image_col_indices_sequence()[idx + l - 1];

            int &&columnDiff = std::abs(int(rIdx - lIdx));
            if (columnDiff > 10)
              break;

            this->label[rIdx] = Label::AMBIGUOUS;
          }
          // left hand side
          for (int l = -1; l >= -5; --l) {
            if (idx + l >= this->range_image.get_image_col_indices_sequence().size()) {
              break;
            }

            rIdx = this->range_image.get_image_col_indices_sequence()[idx + l + 1];
            lIdx = this->range_image.get_image_col_indices_sequence()[idx + l];

            int &&columnDiff = std::abs(int(rIdx - lIdx));
            if (columnDiff > 10)
              break;
            this->label[lIdx] = Label::AMBIGUOUS;
          }
        }
      }
    }
  }

  // Allocate corner and plane points
  this->corner_points.setZero(this->corner_point_indices.size(), 3);
  idx = 0;
  for (const auto &point_idx : this->corner_point_indices) {
    const auto &point           = this->range_image.get_cloud().row(point_idx);
    this->corner_points(idx, 0) = point(0);
    this->corner_points(idx, 1) = point(1);
    this->corner_points(idx, 2) = point(2);
    ++idx;
  }

  this->plane_points.setZero(this->plane_point_indices.size(), 3);
  idx = 0;
  for (const auto &point_idx : this->plane_point_indices) {
    const auto point           = this->range_image.get_cloud().row(point_idx);
    this->plane_points(idx, 0) = point(0);
    this->plane_points(idx, 1) = point(1);
    this->plane_points(idx, 2) = point(2);
    ++idx;
  }
}

};  // namespace keypoint

};  // namespace kcp