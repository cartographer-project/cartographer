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

#include "cartographer/mapping/internal/2d/tsdf_2d.h"

namespace cartographer {
namespace mapping {
namespace scan_matching {

// Interpolates between TSDF2D pixels with bilinear interpolation.
//
// This class is templated to work with Ceres autodiff.
class InterpolatedTSDF2D {
 public:
  explicit InterpolatedTSDF2D(const TSDF2D& grid) : tsdf_(grid) {}

  InterpolatedTSDF2D(const InterpolatedTSDF2D&) = delete;
  InterpolatedTSDF2D& operator=(const InterpolatedTSDF2D&) = delete;

  // Returns the interpolated correspondence cost at (x,y). Cells with at least
  // one 'unknown' interpolation point result in "MaxCorrespondenceCost()" with
  // zero gradient.
  template <typename T>
  T GetCorrespondenceCost(const T& x, const T& y) const {
    float x1, y1, x2, y2;
    ComputeInterpolationDataPoints(x, y, &x1, &y1, &x2, &y2);

    const Eigen::Array2i index1 =
        tsdf_.limits().GetCellIndex(Eigen::Vector2f(x1, y1));

    const float w11 = tsdf_.GetWeight(index1);
    const float w12 = tsdf_.GetWeight(index1 + Eigen::Array2i(-1, 0));
    const float w21 = tsdf_.GetWeight(index1 + Eigen::Array2i(0, -1));
    const float w22 = tsdf_.GetWeight(index1 + Eigen::Array2i(-1, -1));
    if (w11 == 0.0 || w12 == 0.0 || w21 == 0.0 || w22 == 0.0) {
      return T(tsdf_.GetMaxCorrespondenceCost());
    }

    const float q11 = tsdf_.GetCorrespondenceCost(index1);
    const float q12 =
        tsdf_.GetCorrespondenceCost(index1 + Eigen::Array2i(-1, 0));
    const float q21 =
        tsdf_.GetCorrespondenceCost(index1 + Eigen::Array2i(0, -1));
    const float q22 =
        tsdf_.GetCorrespondenceCost(index1 + Eigen::Array2i(-1, -1));

    return InterpolateBilinear(x, y, x1, y1, x2, y2, q11, q12, q21, q22);
  }

  // Returns the interpolated weight at (x,y).
  template <typename T>
  T GetWeight(const T& x, const T& y) const {
    float x1, y1, x2, y2;
    ComputeInterpolationDataPoints(x, y, &x1, &y1, &x2, &y2);

    const Eigen::Array2i index1 =
        tsdf_.limits().GetCellIndex(Eigen::Vector2f(x1, y1));
    const float q11 = tsdf_.GetWeight(index1);
    const float q12 = tsdf_.GetWeight(index1 + Eigen::Array2i(-1, 0));
    const float q21 = tsdf_.GetWeight(index1 + Eigen::Array2i(0, -1));
    const float q22 = tsdf_.GetWeight(index1 + Eigen::Array2i(-1, -1));

    return InterpolateBilinear(x, y, x1, y1, x2, y2, q11, q12, q21, q22);
  }

 private:
  template <typename T>
  void ComputeInterpolationDataPoints(const T& x, const T& y, float* x1,
                                      float* y1, float* x2, float* y2) const {
    const Eigen::Vector2f lower = CenterOfLowerPixel(x, y);
    *x1 = lower.x();
    *y1 = lower.y();
    *x2 = lower.x() + static_cast<float>(tsdf_.limits().resolution());
    *y2 = lower.y() + static_cast<float>(tsdf_.limits().resolution());
  }

  template <typename T>
  T InterpolateBilinear(const T& x, const T& y, float x1, float y1, float x2,
                        float y2, float q11, float q12, float q21,
                        float q22) const {
    const T normalized_x = (x - T(x1)) / T(x2 - x1);
    const T normalized_y = (y - T(y1)) / T(y2 - y1);
    const T q1 = T(q12 - q11) * normalized_y + T(q11);
    const T q2 = T(q22 - q21) * normalized_y + T(q21);
    return T(q2 - q1) * normalized_x + T(q1);
  }

  // Center of the next lower pixel, i.e., not necessarily the pixel containing
  // (x, y). For each dimension, the largest pixel index so that the
  // corresponding center is at most the given coordinate.
  Eigen::Vector2f CenterOfLowerPixel(double x, double y) const {
    // Center of the cell containing (x, y).
    Eigen::Vector2f center = tsdf_.limits().GetCellCenter(
        tsdf_.limits().GetCellIndex(Eigen::Vector2f(x, y)));
    // Move to the next lower pixel center.
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
  Eigen::Vector2f CenterOfLowerPixel(const T& jet_x, const T& jet_y) const {
    return CenterOfLowerPixel(jet_x.a, jet_y.a);
  }

  const TSDF2D& tsdf_;
};

}  // namespace scan_matching
}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_INTERNAL_2D_SCAN_MATCHING_TSDF_INTERPOLATED_TSDF_2D_H_
