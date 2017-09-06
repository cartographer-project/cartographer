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

#ifndef CARTOGRAPHER_MAPPING_3D_SCAN_MATCHING_ROTATIONAL_SCAN_MATCHER_H_
#define CARTOGRAPHER_MAPPING_3D_SCAN_MATCHING_ROTATIONAL_SCAN_MATCHER_H_

#include <vector>

#include "Eigen/Geometry"
#include "cartographer/sensor/point_cloud.h"

namespace cartographer {
namespace mapping_3d {
namespace scan_matching {

class RotationalScanMatcher {
 public:
  // Computes the histogram for a gravity aligned 'point_cloud'.
  static Eigen::VectorXf ComputeHistogram(const sensor::PointCloud& point_cloud,
                                          int histogram_size);

  // Creates a matcher from the given histograms rotated by the given angles.
  // The angles should be chosen to bring the histograms into approximately the
  // same frame.
  explicit RotationalScanMatcher(
      const std::vector<std::pair<Eigen::VectorXf, float>>&
          histograms_at_angles);

  // Scores how well 'histogram' rotated by 'initial_angle' can be understood as
  // further rotated by certain 'angles' relative to the 'nodes'. Each angle
  // results in a score between 0 (worst) and 1 (best).
  std::vector<float> Match(const Eigen::VectorXf& histogram,
                           float initial_angle,
                           const std::vector<float>& angles) const;

 private:
  Eigen::VectorXf histogram_;
};

}  // namespace scan_matching
}  // namespace mapping_3d
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_3D_SCAN_MATCHING_ROTATIONAL_SCAN_MATCHER_H_
