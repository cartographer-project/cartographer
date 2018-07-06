/*
 * Copyright 2018 The Cartographer Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef CARTOGRAPHER_MAPPING_INTERNAL_2D_SCAN_MATCHING_TSDF_INTERPOLATED_TSDF_2D_H_
#define CARTOGRAPHER_MAPPING_INTERNAL_2D_SCAN_MATCHING_TSDF_INTERPOLATED_TSDF_2D_H_

#include <cmath>

#include "cartographer/mapping/2d/tsdf_2d.h"

namespace cartographer {
namespace mapping {
namespace scan_matching {

// Interpolates between TSDF2D pixels. We use the bilinear
// interpolation.
//
// This class is templated to work with Ceres autodiff.
class InterpolatedTSDF2D {
 public:
  explicit InterpolatedTSDF2D(const TSDF2D& grid) : tsdf_(grid) {}

  InterpolatedTSDF2D(const InterpolatedTSDF2D&) = delete;
  InterpolatedTSDF2D& operator=(const InterpolatedTSDF2D&) = delete;

  // Cells with at least one unknown interpolation point result in
  // "MaxCorrespondenceCost()" with zero Gradient.
  template <typename T>
  T GetCorrespondenceCost(const T& x, const T& y) const {
    double x1, y1, x2, y2;
    ComputeInterpolationDataPoints(x, y, &x1, &y1, &x2, &y2);

    const Eigen::Array2i index1 =
        tsdf_.limits().GetCellIndex(Eigen::Vector2f(x1, y1));

    const double w11 = tsdf_.GetWeight(index1);
    const double w12 = tsdf_.GetWeight(index1 + Eigen::Array2i(-1, 0));
    const double w21 = tsdf_.GetWeight(index1 + Eigen::Array2i(0, -1));
    const double w22 = tsdf_.GetWeight(index1 + Eigen::Array2i(-1, -1));
    if (w11 == 0.0 || w12 == 0.0 || w21 == 0.0 || w22 == 0.0) {
      return T(tsdf_.GetMaxCorrespondenceCost());
    }

    const double q11 = tsdf_.GetCorrespondenceCost(index1);
    const double q12 =
        tsdf_.GetCorrespondenceCost(index1 + Eigen::Array2i(-1, 0));
    const double q21 =
        tsdf_.GetCorrespondenceCost(index1 + Eigen::Array2i(0, -1));
    const double q22 =
        tsdf_.GetCorrespondenceCost(index1 + Eigen::Array2i(-1, -1));

    const T normalized_x = (x - x1) / (x2 - x1);
    const T normalized_y = (y - y1) / (y2 - y1);

    const T q1 = (q12 - q11) * normalized_y + q11;
    const T q2 = (q22 - q21) * normalized_y + q21;
    return (q2 - q1) * normalized_x + q1;
  }

  template <typename T>
  T GetWeight(const T& x, const T& y) const {
    double x1, y1, x2, y2;
    ComputeInterpolationDataPoints(x, y, &x1, &y1, &x2, &y2);

    const Eigen::Array2i index1 =
        tsdf_.limits().GetCellIndex(Eigen::Vector2f(x1, y1));
    const double q11 = tsdf_.GetWeight(index1);
    const double q12 =
        tsdf_.GetWeight(index1 + Eigen::Array2i(-1, 0));
    const double q21 =
        tsdf_.GetWeight(index1 + Eigen::Array2i(0, -1));
    const double q22 =
        tsdf_.GetWeight(index1 + Eigen::Array2i(-1, -1));

    const T normalized_x = (x - x1) / (x2 - x1);
    const T normalized_y = (y - y1) / (y2 - y1);

    const T q1 = (q12 - q11) * normalized_y + q11;
    const T q2 = (q22 - q21) * normalized_y + q21;
    return (q2 - q1) * normalized_x + q1;
  }

 private:
  template <typename T>
  void ComputeInterpolationDataPoints(const T& x, const T& y, double* x1,
                                      double* y1, double* x2,
                                      double* y2) const {
    const Eigen::Vector2f lower = CenterOfLowerVoxel(x, y);
    *x1 = lower.x();
    *y1 = lower.y();
    *x2 = lower.x() + tsdf_.limits().resolution();
    *y2 = lower.y() + tsdf_.limits().resolution();
  }

  template <typename T>
  T InterpolateBilinear(const T& x, const T& y, double x1, double y1, double x2,
                        double y2, double q11, double q12, double q21,
                        double q22) const {
    const T normalized_x = (x - x1) / (x2 - x1);
    const T normalized_y = (y - y1) / (y2 - y1);

    const T q1 = (q12 - q11) * normalized_y + q11;
    const T q2 = (q22 - q21) * normalized_y + q21;
    return (q2 - q1) * normalized_x + q1;
  }

  // Center of the next lower voxel, i.e., not necessarily the voxel containing
  // (x, y). For each dimension, the largest voxel index so that the
  // corresponding center is at most the given coordinate.
  Eigen::Vector2f CenterOfLowerVoxel(const double x, const double y) const {
    // Center of the cell containing (x, y).
    Eigen::Vector2f center = tsdf_.limits().GetCellCenter(
        tsdf_.limits().GetCellIndex(Eigen::Vector2f(x, y)));
    // Move to the next lower voxel center.
    if (center.x() > x) {
      center.x() -= tsdf_.limits().resolution();
    }
    if (center.y() > y) {
      center.y() -= tsdf_.limits().resolution();
    }
    return center;
  }

  // Uses the scalar part of a Ceres Jet.
  template <typename T>
  Eigen::Vector2f CenterOfLowerVoxel(const T& jet_x, const T& jet_y) const {
    return CenterOfLowerVoxel(jet_x.a, jet_y.a);
  }

  const TSDF2D& tsdf_;
};

}  // namespace scan_matching
}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_INTERNAL_2D_SCAN_MATCHING_TSDF_INTERPOLATED_TSDF_2D_H_