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

#ifndef CARTOGRAPHER_POSE_GRAPH_NODE_POSE_2D_H_
#define CARTOGRAPHER_POSE_GRAPH_NODE_POSE_2D_H_

#include "cartographer/pose_graph/node/node.h"

#include <array>

#include "Eigen/Core"

namespace cartographer {
namespace pose_graph {

class Pose2D : public Node {
 public:
  Pose2D(const NodeId& node_id, bool constant, const proto::Pose2D& pose_2d);

  std::array<double, 3>* mutable_pose_2d() { return &pose_2d_; }
  const std::array<double, 3>& pose_2d() const { return pose_2d_; }

 protected:
  proto::Parameters ToParametersProto() const final;

 private:
  std::array<double, 3> pose_2d_;
};

}  // namespace pose_graph
}  // namespace cartographer

#endif  // CARTOGRAPHER_POSE_GRAPH_NODE_POSE_2D_H_
