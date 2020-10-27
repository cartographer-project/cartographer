/*
 * Copyright 2019 The Cartographer Authors
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

#ifndef CARTOGRAPHER_MAPPING_INTERNAL_3D_SCAN_MATCHING_INTENSITY_COST_FUNCTION_3D_H_
#define CARTOGRAPHER_MAPPING_INTERNAL_3D_SCAN_MATCHING_INTENSITY_COST_FUNCTION_3D_H_

#include "Eigen/Core"
#include "cartographer/mapping/3d/hybrid_grid.h"
#include "cartographer/mapping/internal/3d/scan_matching/interpolated_grid.h"
#include "cartographer/sensor/point_cloud.h"
#include "cartographer/transform/rigid_transform.h"
#include "cartographer/transform/transform.h"
#include "ceres/ceres.h"

namespace cartographer {
namespace mapping {
namespace scan_matching {

// Computes a cost for matching the 'point_cloud' to the 'hybrid_grid' with a
// 'translation' and 'rotation'. The cost increases when points fall into space
// for which different intensity has been observed, i.e. at voxels with different
// values. Only points up to a certain threshold are evaluated which is intended
// to ignore data from retroreflections.
class IntensityCostFunction3D {
 public:
  static ceres::CostFunction* CreateAutoDiffCostFunction(
      const double scaling_factor, const float intensity_threshold,
      const sensor::PointCloud& point_cloud,
      const IntensityHybridGrid& hybrid_grid);

  template <typename T>
  bool operator()(const T* const translation, const T* const rotation,
                  T* const residual) const {
    const transform::Rigid3<T> transform(
        Eigen::Map<const Eigen::Matrix<T, 3, 1>>(translation),
        Eigen::Quaternion<T>(rotation[0], rotation[1], rotation[2],
                             rotation[3]));
    return Evaluate(transform, residual);
  }

 private:
  IntensityCostFunction3D(const double scaling_factor,
                          const float intensity_threshold,
                          const sensor::PointCloud& point_cloud,
                          const IntensityHybridGrid& hybrid_grid)
      : scaling_factor_(scaling_factor),
        intensity_threshold_(intensity_threshold),
        point_cloud_(point_cloud),
        interpolated_grid_(hybrid_grid) {}

  IntensityCostFunction3D(const IntensityCostFunction3D&) = delete;
  IntensityCostFunction3D& operator=(const IntensityCostFunction3D&) = delete;

  template <typename T>
  bool Evaluate(const transform::Rigid3<T>& transform,
                T* const residual) const {
    for (size_t i = 0; i < point_cloud_.size(); ++i) {
      if (point_cloud_.intensities()[i] > intensity_threshold_) {
        residual[i] = T(0.f);
      } else {
        const Eigen::Matrix<T, 3, 1> point = point_cloud_[i].position.cast<T>();
        const T intensity = T(point_cloud_.intensities()[i]);

        const Eigen::Matrix<T, 3, 1> world = transform * point;
        const T interpolated_intensity =
            interpolated_grid_.GetInterpolatedValue(world[0], world[1],
                                                    world[2]);
        residual[i] = scaling_factor_ * (interpolated_intensity - intensity);
      }
    }
    return true;
  }

  const double scaling_factor_;
  // We will ignore returns with intensity above this threshold.
  const float intensity_threshold_;
  const sensor::PointCloud& point_cloud_;
  const InterpolatedIntensityGrid interpolated_grid_;
};

}  // namespace scan_matching
}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_INTERNAL_3D_SCAN_MATCHING_INTENSITY_COST_FUNCTION_3D_H_
