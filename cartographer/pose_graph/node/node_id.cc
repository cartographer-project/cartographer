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

#include "cartographer/pose_graph/node/node_id.h"

namespace cartographer {
namespace pose_graph {

NodeId::NodeId(const std::string& object_id, common::Time time)
    : object_id(object_id), time(time) {}

NodeId::NodeId(const std::string& object_id, const std::string& group_id,
               common::Time time)
    : object_id(object_id), group_id(group_id), time(time) {}

NodeId::NodeId(const proto::NodeId& node_id)
    : NodeId(node_id.object_id(), node_id.group_id(),
             common::FromUniversal(node_id.timestamp())) {}

proto::NodeId NodeId::ToProto() const {
  proto::NodeId node_id;
  node_id.set_object_id(object_id);
  node_id.set_group_id(group_id);
  node_id.set_timestamp(common::ToUniversal(time));
  return node_id;
}

}  // namespace pose_graph
}  // namespace cartographer
