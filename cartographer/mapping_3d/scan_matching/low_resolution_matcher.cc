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

#include "cartographer/mapping_3d/scan_matching/low_resolution_matcher.h"

namespace cartographer {
namespace mapping_3d {
namespace scan_matching {

namespace {

// TODO(zhengj, whess): Interpolate the Grid to get better score.
float EvaluateLowResolutionScore(const HybridGrid& low_resolution_grid,
                                 const sensor::PointCloud& points) {
  float score = 0.f;
  for (const auto& point : points) {
    score += low_resolution_grid.GetProbability(
        low_resolution_grid.GetCellIndex(point));
  }
  return score / points.size();
}

}  // namespace

std::function<bool(const transform::Rigid3f&)> CreateLowResolutionMatcher(
    const HybridGrid* low_resolution_grid, const sensor::PointCloud* points,
    const float min_low_resolution_score) {
  return [=](const transform::Rigid3f& pose) {
    return EvaluateLowResolutionScore(
               *low_resolution_grid,
               sensor::TransformPointCloud(*points, pose)) >=
           min_low_resolution_score;
  };
}

}  // namespace scan_matching
}  // namespace mapping_3d
}  // namespace cartographer
