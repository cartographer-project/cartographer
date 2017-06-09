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

#ifndef CARTOGRAPHER_MAPPING_3D_SCAN_MATCHING_CERES_SCAN_MATCHER_H_
#define CARTOGRAPHER_MAPPING_3D_SCAN_MATCHING_CERES_SCAN_MATCHER_H_

#include <utility>
#include <vector>

#include "Eigen/Core"
#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/mapping_3d/hybrid_grid.h"
#include "cartographer/mapping_3d/scan_matching/proto/ceres_scan_matcher_options.pb.h"
#include "cartographer/sensor/point_cloud.h"
#include "cartographer/transform/rigid_transform.h"

namespace cartographer {
namespace mapping_3d {
namespace scan_matching {

proto::CeresScanMatcherOptions CreateCeresScanMatcherOptions(
    common::LuaParameterDictionary* parameter_dictionary);

using PointCloudAndHybridGridPointers =
    std::pair<const sensor::PointCloud*, const HybridGrid*>;

// This scan matcher uses Ceres to align scans with an existing map.
class CeresScanMatcher {
 public:
  explicit CeresScanMatcher(const proto::CeresScanMatcherOptions& options);

  CeresScanMatcher(const CeresScanMatcher&) = delete;
  CeresScanMatcher& operator=(const CeresScanMatcher&) = delete;

  // Aligns 'point_clouds' within the 'hybrid_grids' given an
  // 'initial_pose_estimate' and returns a 'pose_estimate' and the solver
  // 'summary'.
  void Match(const transform::Rigid3d& previous_pose,
             const transform::Rigid3d& initial_pose_estimate,
             const std::vector<PointCloudAndHybridGridPointers>&
                 point_clouds_and_hybrid_grids,
             transform::Rigid3d* pose_estimate,
             ceres::Solver::Summary* summary);

 private:
  const proto::CeresScanMatcherOptions options_;
  ceres::Solver::Options ceres_solver_options_;
};

}  // namespace scan_matching
}  // namespace mapping_3d
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_3D_SCAN_MATCHING_CERES_SCAN_MATCHER_H_
