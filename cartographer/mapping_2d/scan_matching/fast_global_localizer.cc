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

#include "cartographer/mapping_2d/scan_matching/fast_global_localizer.h"

#include "glog/logging.h"

namespace cartographer {
namespace mapping_2d {
namespace scan_matching {

bool PerformGlobalLocalization(
    const float cutoff,
    const cartographer::sensor::AdaptiveVoxelFilter& voxel_filter,
    const std::vector<
        cartographer::mapping_2d::scan_matching::FastCorrelativeScanMatcher*>&
        matchers,
    const cartographer::sensor::PointCloud& point_cloud,
    transform::Rigid2d* const best_pose_estimate, float* const best_score) {
  CHECK(best_pose_estimate != nullptr)
      << "Need a non-null output_pose_estimate!";
  CHECK(best_score != nullptr) << "Need a non-null best_score!";
  *best_score = cutoff;
  transform::Rigid2d pose_estimate;
  const sensor::PointCloud filtered_point_cloud =
      voxel_filter.Filter(point_cloud);
  bool success = false;
  if (matchers.size() == 0) {
    LOG(WARNING) << "Map not yet large enough to localize in!";
    return false;
  }
  for (auto& matcher : matchers) {
    float score = -1;
    transform::Rigid2d pose_estimate;
    if (matcher->MatchFullSubmap(filtered_point_cloud, *best_score, &score,
                                 &pose_estimate)) {
      CHECK_GT(score, *best_score) << "MatchFullSubmap lied!";
      *best_score = score;
      *best_pose_estimate = pose_estimate;
      success = true;
    }
  }
  return success;
}

}  // namespace scan_matching
}  // namespace mapping_2d
}  // namespace cartographer
