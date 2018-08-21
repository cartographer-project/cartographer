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

#include "cartographer/mapping/internal/optimization/ceres_pose.h"

namespace cartographer {
namespace mapping {
namespace optimization {

CeresPose::Data FromPose(const transform::Rigid3d& pose) {
  return CeresPose::Data{{{pose.translation().x(), pose.translation().y(),
                           pose.translation().z()}},
                         {{pose.rotation().w(), pose.rotation().x(),
                           pose.rotation().y(), pose.rotation().z()}}};
}

CeresPose::CeresPose(
    const transform::Rigid3d& pose,
    std::unique_ptr<ceres::LocalParameterization> translation_parametrization,
    std::unique_ptr<ceres::LocalParameterization> rotation_parametrization,
    ceres::Problem* problem)
    : data_(std::make_shared<CeresPose::Data>(FromPose(pose))) {
  problem->AddParameterBlock(data_->translation.data(), 3,
                             translation_parametrization.release());
  problem->AddParameterBlock(data_->rotation.data(), 4,
                             rotation_parametrization.release());
}

const transform::Rigid3d CeresPose::ToRigid() const {
  return transform::Rigid3d::FromArrays(data_->rotation, data_->translation);
}

}  // namespace optimization
}  // namespace mapping
}  // namespace cartographer
