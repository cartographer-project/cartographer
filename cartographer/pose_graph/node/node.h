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

#ifndef CARTOGRAPHER_POSE_GRAPH_NODE_NODE_H_
#define CARTOGRAPHER_POSE_GRAPH_NODE_NODE_H_

#include "cartographer/pose_graph/proto/node.pb.h"

#include <array>
#include <string>
#include <vector>

namespace cartographer {
namespace pose_graph {

// TODO(pifon): Change it to struct { string, timestamp} and introduce the
// necessary operators.
using NodeId = std::string;

class Node {
 public:
  explicit Node(NodeId id, bool constant) : node_id_(id), constant_(constant) {}
  ~Node() = default;

  Node(const Node&) = delete;
  Node& operator=(const Node&) = delete;

  proto::Node ToProto() const;

  const NodeId node_id() const { return node_id_; }
  void set_node_id(const NodeId& id) { node_id_ = id; }

  bool constant() const { return constant_; }
  void set_constant(bool constant) { constant_ = constant; }

 protected:
  virtual proto::Parameters ToParametersProto() const = 0;

 private:
  NodeId node_id_;
  bool constant_;
};

}  // namespace pose_graph
}  // namespace cartographer

#endif  // CARTOGRAPHER_POSE_GRAPH_NODE_NODE_H_
