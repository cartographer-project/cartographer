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

#include "cartographer/pose_graph/node/pose_3d.h"

namespace cartographer {
namespace pose_graph {

Pose3D::Pose3D(const NodeId& node_id, bool constant,
               const proto::Pose3D& pose_3d)
    : Node(node_id, constant),
      translation_{{pose_3d.translation().x(), pose_3d.translation().y(),
                    pose_3d.translation().z()}},
      translation_parameterization_(pose_3d.translation_parameterization()),
      rotation_{{pose_3d.rotation().x(), pose_3d.rotation().y(),
                 pose_3d.rotation().z(), pose_3d.rotation().w()}},
      rotation_parameterization_(pose_3d.rotation_parameterization()) {}

proto::Parameters Pose3D::ToParametersProto() const {
  proto::Parameters parameters;
  auto* pose_3d = parameters.mutable_pose_3d();

  auto* translation = pose_3d->mutable_translation();
  translation->set_x(translation_[0]);
  translation->set_y(translation_[1]);
  translation->set_z(translation_[2]);
  pose_3d->set_translation_parameterization(
      translation_parameterization_.ToProto());

  auto* rotation = pose_3d->mutable_rotation();
  rotation->set_x(rotation_[0]);
  rotation->set_y(rotation_[1]);
  rotation->set_z(rotation_[2]);
  rotation->set_w(rotation_[3]);
  pose_3d->set_rotation_parameterization(rotation_parameterization_.ToProto());

  return parameters;
}

}  // namespace pose_graph
}  // namespace cartographer
