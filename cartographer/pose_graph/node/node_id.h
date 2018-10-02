/*
 * Copyright 2018 The Cartographer Authors
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

#ifndef CARTOGRAPHER_POSE_GRAPH_NODE_NODE_ID_
#define CARTOGRAPHER_POSE_GRAPH_NODE_NODE_ID_

#include <ostream>
#include <string>

#include "cartographer/common/time.h"
#include "cartographer/pose_graph/proto/node.pb.h"

namespace cartographer {
namespace pose_graph {

struct NodeId {
  NodeId(const std::string& object_id, common::Time time);
  NodeId(const std::string& object_id, const std::string& group_id,
         common::Time time);
  explicit NodeId(const proto::NodeId& node_id);

  // Object refers to dynamic/static objects, e.g. robot/landmark/submap poses,
  // IMU calibration, etc.
  std::string object_id;
  // Id of the group to which the node belongs, e.g. "submap".
  std::string group_id;
  // Time associated with the object's pose.
  common::Time time;

  proto::NodeId ToProto() const;
};

inline bool operator<(const NodeId& lhs, const NodeId& rhs) {
  return std::forward_as_tuple(lhs.object_id, lhs.group_id, lhs.time) <
         std::forward_as_tuple(rhs.object_id, rhs.group_id, rhs.time);
}

inline std::ostream& operator<<(std::ostream& os, const NodeId& id) {
  std::string group_message;
  if (!id.group_id.empty()) {
    group_message = ", group_id: " + id.group_id;
  }
  return os << "(object_id: " << id.object_id << group_message
            << ", time: " << id.time << ")";
}

}  // namespace pose_graph
}  // namespace cartographer

#endif  // CARTOGRAPHER_POSE_GRAPH_NODE_NODE_ID_
