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

#include "cartographer/pose_graph/pose_2d.h"

namespace cartographer {
namespace pose_graph {
namespace {

constexpr size_t kXIndex = 0;
constexpr size_t kYIndex = 1;
constexpr size_t kRotationIndex = 2;

}  // namespace

Pose2D::Pose2D(const NodeId& node_id, bool constant,
               const Eigen::Vector2d& translation, double rotation)
    : Node(node_id, constant),
      pose_2d_{{translation.x(), translation.y(), rotation}} {
  AddParameterBlock(Parameterization::NONE, &pose_2d_);
}

proto::Parameters Pose2D::ToParametersProto() const {
  proto::Parameters parameters;
  auto* pose_2d = parameters.mutable_pose_2d();
  pose_2d->set_rotation(pose_2d_[kRotationIndex]);

  auto* translation = pose_2d->mutable_translation();
  translation->set_x(pose_2d_[kXIndex]);
  translation->set_y(pose_2d_[kYIndex]);
  return parameters;
}

}  // namespace pose_graph
}  // namespace cartographer
