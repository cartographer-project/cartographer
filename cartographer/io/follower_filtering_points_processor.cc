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

#include "cartographer/io/follower_filtering_points_processor.h"

#include "absl/memory/memory.h"
#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/io/points_batch.h"
#include "Eigen/Core"

namespace cartographer {
namespace io {

std::unique_ptr<FollowerFiteringPointsProcessor>
FollowerFiteringPointsProcessor::FromDictionary(
    common::LuaParameterDictionary* const dictionary,
    PointsProcessor* const next) {
  return absl::make_unique<FollowerFiteringPointsProcessor>(
      dictionary->GetInt("yaw_range"),
      dictionary->GetDouble("follow_distance"),
      dictionary->GetDouble("min_height"),
      dictionary->GetDouble("max_height"),
      next);
}

FollowerFiteringPointsProcessor::FollowerFiteringPointsProcessor(
    const int yaw_range, const double follow_distance, const double min_height, const double max_height, PointsProcessor* next)
    : yaw_range_(yaw_range), follow_distance_(follow_distance), min_height_(min_height), max_height_(max_height), next_(next) {}

void FollowerFiteringPointsProcessor::Process(
    std::unique_ptr<PointsBatch> batch) {
  absl::flat_hash_set<int> to_remove;

  // we are checking 3 criteria for the follower points and remove them if all are met
  for (size_t i = 0; i < batch->points.size(); ++i) {
    // 1. inside certain yaw range
    //TODO : remove batch->origin from topBoundary and it is the same as topVec
    const Eigen::Vector3f topBoundary = batch->origin + Eigen::Vector3f(follow_distance_ * std::cos(yaw_range_/2), follow_distance_ * -std::sin(yaw_range_/2), 0);
    const Eigen::Vector3f bottomBoundary = batch->origin + Eigen::Vector3f(follow_distance_ * std::cos(yaw_range_/2), follow_distance_ * std::sin(yaw_range_/2), 0);
    const Eigen::Vector3f topVec = topBoundary - batch->origin;
    const Eigen::Vector3f bottomVec = bottomBoundary - batch->origin;
    const float skalarTop = topVec[0] * (batch->points[i].position[1] - batch->origin[1]) - topVec[1] * (batch->points[i].position[0] - batch->origin[0]);
    const float skalarBottom = bottomVec[0] * (batch->points[i].position[1] - batch->origin[1]) - bottomVec[1] * (batch->points[i].position[0] - batch->origin[0]);
    const bool invalid_yaw = skalarBottom < 0 && skalarTop > 0;

    // 2. inside certain follow distance
    const float actual_follow_distance = std::abs(batch->points[i].position[0] - batch->origin[0]);
    const bool invalid_follow_distance = actual_follow_distance <= follow_distance_;

    // 3. inside certain z-position
    const float z_position = batch->points[i].position[2];
    const bool invalid_z_position = z_position >= min_height_ || z_position >= max_height_;

    if (invalid_yaw && invalid_follow_distance && invalid_z_position) {
      to_remove.insert(i);
    }
  }

  RemovePoints(to_remove, batch.get());
  next_->Process(std::move(batch));
}

PointsProcessor::FlushResult FollowerFiteringPointsProcessor::Flush() {
  return next_->Flush();
}

}  // namespace io
}  // namespace cartographer
