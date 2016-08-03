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

#include "cartographer/mapping_3d/ceres_pose.h"

namespace cartographer {
namespace mapping_3d {

CeresPose::CeresPose(
    const transform::Rigid3d& rigid,
    std::unique_ptr<ceres::LocalParameterization> rotation_parametrization,
    ceres::Problem* problem)
    : translation_({{rigid.translation().x(), rigid.translation().y(),
                     rigid.translation().z()}}),
      rotation_({{rigid.rotation().w(), rigid.rotation().x(),
                  rigid.rotation().y(), rigid.rotation().z()}}) {
  problem->AddParameterBlock(translation_.data(), 3);
  problem->AddParameterBlock(rotation_.data(), 4,
                             rotation_parametrization.release());
}

const transform::Rigid3d CeresPose::ToRigid() const {
  return transform::Rigid3d(
      Eigen::Map<const Eigen::Vector3d>(translation_.data()),
      Eigen::Quaterniond(rotation_[0], rotation_[1], rotation_[2],
                         rotation_[3]));
}

}  // namespace mapping_3d
}  // namespace cartographer
