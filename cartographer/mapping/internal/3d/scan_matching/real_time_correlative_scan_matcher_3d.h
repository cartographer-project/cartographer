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

#ifndef CARTOGRAPHER_MAPPING_INTERNAL_3D_SCAN_MATCHING_REAL_TIME_CORRELATIVE_SCAN_MATCHER_3D_H_
#define CARTOGRAPHER_MAPPING_INTERNAL_3D_SCAN_MATCHING_REAL_TIME_CORRELATIVE_SCAN_MATCHER_3D_H_

#include <vector>

#include "Eigen/Core"
#include "cartographer/mapping/3d/hybrid_grid.h"
#include "cartographer/mapping/proto/scan_matching/real_time_correlative_scan_matcher_options.pb.h"
#include "cartographer/sensor/point_cloud.h"

namespace cartographer {
namespace mapping {
namespace scan_matching {

// A voxel accurate scan matcher, exhaustively evaluating the scan matching
// search space.
class RealTimeCorrelativeScanMatcher3D {
 public:
  explicit RealTimeCorrelativeScanMatcher3D(
      const scan_matching::proto::RealTimeCorrelativeScanMatcherOptions&
          options);

  RealTimeCorrelativeScanMatcher3D(const RealTimeCorrelativeScanMatcher3D&) =
      delete;
  RealTimeCorrelativeScanMatcher3D& operator=(
      const RealTimeCorrelativeScanMatcher3D&) = delete;

  // Aligns 'point_cloud' within the 'hybrid_grid' given an
  // 'initial_pose_estimate' then updates 'pose_estimate' with the result and
  // returns the score.
  float Match(const transform::Rigid3d& initial_pose_estimate,
              const sensor::PointCloud& point_cloud,
              const HybridGrid& hybrid_grid,
              transform::Rigid3d* pose_estimate) const;

 private:
  std::vector<transform::Rigid3f> GenerateExhaustiveSearchTransforms(
      float resolution, const sensor::PointCloud& point_cloud) const;
  float ScoreCandidate(const HybridGrid& hybrid_grid,
                       const sensor::PointCloud& transformed_point_cloud,
                       const transform::Rigid3f& transform) const;

  const proto::RealTimeCorrelativeScanMatcherOptions options_;
};

}  // namespace scan_matching
}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_INTERNAL_3D_SCAN_MATCHING_REAL_TIME_CORRELATIVE_SCAN_MATCHER_3D_H_
