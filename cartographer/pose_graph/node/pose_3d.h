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

#ifndef CARTOGRAPHER_POSE_GRAPH_NODE_POSE_3D_H_
#define CARTOGRAPHER_POSE_GRAPH_NODE_POSE_3D_H_

#include "cartographer/pose_graph/node/node.h"

#include <array>

#include "cartographer/transform/transform.h"

namespace cartographer {
namespace pose_graph {

class Pose3D : public Node {
 public:
  Pose3D(const NodeId& node_id, bool constant,
         const Eigen::Vector3d& translation,
         const Eigen::Quaterniond& rotation);

  std::array<double, 3>* mutable_translation() { return &translation_; }
  const std::array<double, 3>& translation() const { return translation_; }

  std::array<double, 4>* mutable_rotation() { return &rotation_; }
  const std::array<double, 4>& rotation() const { return rotation_; }

 protected:
  proto::Parameters ToParametersProto() const final;

 private:
  std::array<double, 3> translation_;
  std::array<double, 4> rotation_;
};

}  // namespace pose_graph
}  // namespace cartographer

#endif  // CARTOGRAPHER_POSE_GRAPH_NODE_POSE_3D_H_
