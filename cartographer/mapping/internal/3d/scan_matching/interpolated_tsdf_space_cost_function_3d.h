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

#ifndef CARTOGRAPHER_MAPPING_INTERNAL_3D_SCAN_MATCHING_INTERPOLATED_TSDF_SPACE_COST_FUNCTION_3D_H_
#define CARTOGRAPHER_MAPPING_INTERNAL_3D_SCAN_MATCHING_INTERPOLATED_TSDF_SPACE_COST_FUNCTION_3D_H_

#include "Eigen/Core"
#include "cartographer/mapping/3d/hybrid_grid_tsdf.h"
#include "cartographer/mapping/internal/3d/scan_matching/interpolated_tsdf.h"
#include "cartographer/sensor/point_cloud.h"
#include "cartographer/transform/rigid_transform.h"
#include "cartographer/transform/timestamped_transform.h"
#include "cartographer/transform/transform.h"

namespace cartographer {
namespace mapping {
namespace scan_matching {

// Computes a cost for matching the 'point_cloud' to the 'hybrid_grid' with a
// 'translation' and 'rotation'. The cost increases when points fall into less
// occupied space, i.e. at voxels with lower values.
class InterpolatedTSDFSpaceCostFunction3D {
 public:
  static ceres::CostFunction* CreateAutoDiffCostFunction(
      const double scaling_factor, const sensor::PointCloud& point_cloud,
      const mapping::HybridGridTSDF& hybrid_grid,
      const double interpolation_ratio) {
    if (interpolation_ratio <= 0.0 || interpolation_ratio >= 1.0) {
      LOG(WARNING) << "Extrapolating to: " << interpolation_ratio;
    }
    return new ceres::AutoDiffCostFunction<
        InterpolatedTSDFSpaceCostFunction3D, ceres::DYNAMIC /* residuals */,
        3 /* translation variables */, 4 /* rotation variables */,
        3 /* translation variables */, 4 /* rotation variables */>(
        new InterpolatedTSDFSpaceCostFunction3D(
            scaling_factor, point_cloud, hybrid_grid, interpolation_ratio),
        point_cloud.size());
  }

  template <typename T>
  bool operator()(const T* const translation_0, const T* const rotation_0,
                  const T* const translation_1, const T* const rotation_1,
                  T* const residual) const {
    const transform::Rigid3<T> transform_0(
        Eigen::Map<const Eigen::Matrix<T, 3, 1>>(translation_0),
        Eigen::Quaternion<T>(rotation_0[0], rotation_0[1], rotation_0[2],
                             rotation_0[3]));
    const transform::Rigid3<T> transform_1(
        Eigen::Map<const Eigen::Matrix<T, 3, 1>>(translation_1),
        Eigen::Quaternion<T>(rotation_1[0], rotation_1[1], rotation_1[2],
                             rotation_1[3]));
    return Evaluate(
        InterpolateTransform(transform_0, transform_1, interpolation_ratio_),
        residual);
  }

 private:
  InterpolatedTSDFSpaceCostFunction3D(
      const double scaling_factor, const sensor::PointCloud& point_cloud,
      const mapping::HybridGridTSDF& hybrid_grid,
      const double interpolation_ratio)
      : scaling_factor_(scaling_factor),
        point_cloud_(point_cloud),
        interpolated_grid_(hybrid_grid),
        interpolation_ratio_(interpolation_ratio) {}

  InterpolatedTSDFSpaceCostFunction3D(
      const InterpolatedTSDFSpaceCostFunction3D&) = delete;
  InterpolatedTSDFSpaceCostFunction3D& operator=(
      const InterpolatedTSDFSpaceCostFunction3D&) = delete;

  template <typename T>
  bool Evaluate(const transform::Rigid3<T>& transform,
                T* const residual) const {
    for (size_t i = 0; i < point_cloud_.size(); ++i) {
      const Eigen::Matrix<T, 3, 1> world =
          transform * point_cloud_[i].position.cast<T>();
      const T tsd = interpolated_grid_.GetTSD(world[0], world[1], world[2]);
      residual[i] = scaling_factor_ * tsd;
    }
    return true;
  }

  const double scaling_factor_;
  const sensor::PointCloud& point_cloud_;
  const InterpolatedTSDF interpolated_grid_;
  const double interpolation_ratio_;
};

}  // namespace scan_matching
}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_INTERNAL_3D_SCAN_MATCHING_INTERPOLATED_TSDF_SPACE_COST_FUNCTION_3D_H_
