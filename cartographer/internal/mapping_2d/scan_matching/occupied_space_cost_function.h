/*
 * Copyright 2016 The Cartographer Authors
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

#ifndef CARTOGRAPHER_INTERNAL_MAPPING_2D_SCAN_MATCHING_OCCUPIED_SPACE_COST_FUNCTION_H_
#define CARTOGRAPHER_INTERNAL_MAPPING_2D_SCAN_MATCHING_OCCUPIED_SPACE_COST_FUNCTION_H_

#include "Eigen/Core"
#include "Eigen/Geometry"
#include "cartographer/mapping_2d/probability_grid.h"
#include "cartographer/sensor/point_cloud.h"
#include "ceres/ceres.h"
#include "ceres/cubic_interpolation.h"

namespace cartographer {
namespace mapping_2d {
namespace scan_matching {

// Computes a cost for matching the 'point_cloud' to the 'probability_grid' with
// a 'pose'. The cost increases when points fall into less occupied space, i.e.
// at pixels with lower values.
class OccupiedSpaceCostFunction {
 public:
  static ceres::CostFunction* CreateAutoDiffCostFunction(
      const double scaling_factor, const sensor::PointCloud& point_cloud,
      const ProbabilityGrid& probability_grid) {
    return new ceres::AutoDiffCostFunction<OccupiedSpaceCostFunction,
                                           ceres::DYNAMIC /* residuals */,
                                           3 /* pose variables */>(
        new OccupiedSpaceCostFunction(scaling_factor, point_cloud,
                                      probability_grid),
        point_cloud.size());
  }

  template <typename T>
  bool operator()(const T* const pose, T* residual) const {
    Eigen::Matrix<T, 2, 1> translation(pose[0], pose[1]);
    Eigen::Rotation2D<T> rotation(pose[2]);
    Eigen::Matrix<T, 2, 2> rotation_matrix = rotation.toRotationMatrix();
    Eigen::Matrix<T, 3, 3> transform;
    transform << rotation_matrix, translation, T(0.), T(0.), T(1.);

    const GridArrayAdapter adapter(probability_grid_);
    ceres::BiCubicInterpolator<GridArrayAdapter> interpolator(adapter);
    const MapLimits& limits = probability_grid_.limits();

    for (size_t i = 0; i < point_cloud_.size(); ++i) {
      // Note that this is a 2D point. The third component is a scaling factor.
      const Eigen::Matrix<T, 3, 1> point((T(point_cloud_[i].x())),
                                         (T(point_cloud_[i].y())), T(1.));
      const Eigen::Matrix<T, 3, 1> world = transform * point;
      interpolator.Evaluate(
          (limits.max().x() - world[0]) / limits.resolution() - 0.5 +
              T(kPadding),
          (limits.max().y() - world[1]) / limits.resolution() - 0.5 +
              T(kPadding),
          &residual[i]);
      residual[i] = scaling_factor_ * (1. - residual[i]);
    }
    return true;
  }

 private:
  static constexpr int kPadding = INT_MAX / 4;
  class GridArrayAdapter {
   public:
    enum { DATA_DIMENSION = 1 };

    explicit GridArrayAdapter(const ProbabilityGrid& probability_grid)
        : probability_grid_(probability_grid) {}

    void GetValue(const int row, const int column, double* const value) const {
      if (row < kPadding || column < kPadding || row >= NumRows() - kPadding ||
          column >= NumCols() - kPadding) {
        *value = mapping::kMinProbability;
      } else {
        *value = static_cast<double>(probability_grid_.GetProbability(
            Eigen::Array2i(column - kPadding, row - kPadding)));
      }
    }

    int NumRows() const {
      return probability_grid_.limits().cell_limits().num_y_cells +
             2 * kPadding;
    }

    int NumCols() const {
      return probability_grid_.limits().cell_limits().num_x_cells +
             2 * kPadding;
    }

   private:
    const ProbabilityGrid& probability_grid_;
  };

  OccupiedSpaceCostFunction(const double scaling_factor,
                            const sensor::PointCloud& point_cloud,
                            const ProbabilityGrid& probability_grid)
      : scaling_factor_(scaling_factor),
        point_cloud_(point_cloud),
        probability_grid_(probability_grid) {}

  OccupiedSpaceCostFunction(const OccupiedSpaceCostFunction&) = delete;
  OccupiedSpaceCostFunction& operator=(const OccupiedSpaceCostFunction&) =
      delete;

  const double scaling_factor_;
  const sensor::PointCloud& point_cloud_;
  const ProbabilityGrid& probability_grid_;
};

}  // namespace scan_matching
}  // namespace mapping_2d
}  // namespace cartographer

#endif  // CARTOGRAPHER_INTERNAL_MAPPING_2D_SCAN_MATCHING_OCCUPIED_SPACE_COST_FUNCTION_H_
