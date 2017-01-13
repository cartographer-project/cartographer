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

#include "cartographer/mapping/trajectory_node.h"

#include "cartographer/transform/transform.h"

namespace cartographer {
namespace mapping {

proto::Trajectory ToProto(const std::vector<TrajectoryNode>& nodes) {
  proto::Trajectory trajectory;
  for (const auto& node : nodes) {
    const auto& data = *node.constant_data;
    auto* node_proto = trajectory.add_node();
    node_proto->set_timestamp(common::ToUniversal(data.time));
    *node_proto->mutable_pose() =
        transform::ToProto(node.pose * data.tracking_to_pose);
  }
  return trajectory;
}

}  // namespace mapping
}  // namespace cartographer
