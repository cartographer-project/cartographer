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

#ifndef CARTOGRAPHER_MAPPING_INTERNAL_3D_SCAN_MATCHING_CERES_SCAN_MATCHER_3D_H_
#define CARTOGRAPHER_MAPPING_INTERNAL_3D_SCAN_MATCHING_CERES_SCAN_MATCHER_3D_H_

#include <utility>
#include <vector>

#include "Eigen/Core"
#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/mapping/3d/hybrid_grid.h"
#include "cartographer/mapping/proto/scan_matching/ceres_scan_matcher_options_3d.pb.h"
#include "cartographer/sensor/point_cloud.h"
#include "cartographer/transform/rigid_transform.h"

namespace cartographer {
namespace mapping {
namespace scan_matching {

proto::CeresScanMatcherOptions3D CreateCeresScanMatcherOptions3D(
    common::LuaParameterDictionary* parameter_dictionary);

struct PointCloudAndHybridGridsPointers {
  const sensor::PointCloud* point_cloud;
  const HybridGrid* hybrid_grid;
  const IntensityHybridGrid* intensity_hybrid_grid;  // optional
};

// This scan matcher uses Ceres to align scans with an existing map.
class CeresScanMatcher3D {
 public:
  explicit CeresScanMatcher3D(const proto::CeresScanMatcherOptions3D& options);

  CeresScanMatcher3D(const CeresScanMatcher3D&) = delete;
  CeresScanMatcher3D& operator=(const CeresScanMatcher3D&) = delete;

  // Aligns 'point_clouds' within the 'hybrid_grids' given an
  // 'initial_pose_estimate' and returns a 'pose_estimate' and the solver
  // 'summary'.
  void Match(const Eigen::Vector3d& target_translation,
             const transform::Rigid3d& initial_pose_estimate,
             const std::vector<PointCloudAndHybridGridsPointers>&
                 point_clouds_and_hybrid_grids,
             transform::Rigid3d* pose_estimate,
             ceres::Solver::Summary* summary) const;

 private:
  const proto::CeresScanMatcherOptions3D options_;
  ceres::Solver::Options ceres_solver_options_;
};

}  // namespace scan_matching
}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_INTERNAL_3D_SCAN_MATCHING_CERES_SCAN_MATCHER_3D_H_
