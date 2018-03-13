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

#include "cartographer/mapping/internal/3d/scan_matching/real_time_correlative_scan_matcher_3d.h"

#include <cmath>

#include "Eigen/Geometry"
#include "cartographer/common/math.h"
#include "cartographer/transform/transform.h"
#include "glog/logging.h"

namespace cartographer {
namespace mapping {
namespace scan_matching {

RealTimeCorrelativeScanMatcher3D::RealTimeCorrelativeScanMatcher3D(
    const proto::RealTimeCorrelativeScanMatcherOptions& options)
    : options_(options) {}

float RealTimeCorrelativeScanMatcher3D::Match(
    const transform::Rigid3d& initial_pose_estimate,
    const sensor::PointCloud& point_cloud, const HybridGrid& hybrid_grid,
    transform::Rigid3d* pose_estimate) const {
  CHECK_NOTNULL(pose_estimate);
  float best_score = -1.f;
  for (const transform::Rigid3f& transform : GenerateExhaustiveSearchTransforms(
           hybrid_grid.resolution(), point_cloud)) {
    const transform::Rigid3f candidate =
        initial_pose_estimate.cast<float>() * transform;
    const float score = ScoreCandidate(
        hybrid_grid, sensor::TransformPointCloud(point_cloud, candidate),
        transform);
    if (score > best_score) {
      best_score = score;
      *pose_estimate = candidate.cast<double>();
    }
  }
  return best_score;
}

std::vector<transform::Rigid3f>
RealTimeCorrelativeScanMatcher3D::GenerateExhaustiveSearchTransforms(
    const float resolution, const sensor::PointCloud& point_cloud) const {
  std::vector<transform::Rigid3f> result;
  const int linear_window_size =
      common::RoundToInt(options_.linear_search_window() / resolution);
  // We set this value to something on the order of resolution to make sure that
  // the std::acos() below is defined.
  float max_scan_range = 3.f * resolution;
  for (const Eigen::Vector3f& point : point_cloud) {
    const float range = point.norm();
    max_scan_range = std::max(range, max_scan_range);
  }
  const float kSafetyMargin = 1.f - 1e-3f;
  const float angular_step_size =
      kSafetyMargin * std::acos(1.f - common::Pow2(resolution) /
                                          (2.f * common::Pow2(max_scan_range)));
  const int angular_window_size =
      common::RoundToInt(options_.angular_search_window() / angular_step_size);
  for (int z = -linear_window_size; z <= linear_window_size; ++z) {
    for (int y = -linear_window_size; y <= linear_window_size; ++y) {
      for (int x = -linear_window_size; x <= linear_window_size; ++x) {
        for (int rz = -angular_window_size; rz <= angular_window_size; ++rz) {
          for (int ry = -angular_window_size; ry <= angular_window_size; ++ry) {
            for (int rx = -angular_window_size; rx <= angular_window_size;
                 ++rx) {
              const Eigen::Vector3f angle_axis(rx * angular_step_size,
                                               ry * angular_step_size,
                                               rz * angular_step_size);
              result.emplace_back(
                  Eigen::Vector3f(x * resolution, y * resolution,
                                  z * resolution),
                  transform::AngleAxisVectorToRotationQuaternion(angle_axis));
            }
          }
        }
      }
    }
  }
  return result;
}

float RealTimeCorrelativeScanMatcher3D::ScoreCandidate(
    const HybridGrid& hybrid_grid,
    const sensor::PointCloud& transformed_point_cloud,
    const transform::Rigid3f& transform) const {
  float score = 0.f;
  for (const Eigen::Vector3f& point : transformed_point_cloud) {
    score += hybrid_grid.GetProbability(hybrid_grid.GetCellIndex(point));
  }
  score /= static_cast<float>(transformed_point_cloud.size());
  const float angle = transform::GetAngle(transform);
  score *=
      std::exp(-common::Pow2(transform.translation().norm() *
                                 options_.translation_delta_cost_weight() +
                             angle * options_.rotation_delta_cost_weight()));
  CHECK_GT(score, 0.f);
  return score;
}

}  // namespace scan_matching
}  // namespace mapping
}  // namespace cartographer
