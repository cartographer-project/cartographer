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
    std::unique_ptr<ceres::LocalParameterization> translation_parametrization,
    std::unique_ptr<ceres::LocalParameterization> rotation_parametrization,
    ceres::Problem* problem)
    : data_(std::make_shared<CeresPose::Data>(
          CeresPose::Data{{{rigid.translation().x(), rigid.translation().y(),
                            rigid.translation().z()}},
                          {{rigid.rotation().w(), rigid.rotation().x(),
                            rigid.rotation().y(), rigid.rotation().z()}}})) {
  problem->AddParameterBlock(data_->translation.data(), 3,
                             translation_parametrization.release());
  problem->AddParameterBlock(data_->rotation.data(), 4,
                             rotation_parametrization.release());
}

const transform::Rigid3d CeresPose::ToRigid() const {
  const auto& rotation = data_->rotation;
  return transform::Rigid3d(
      Eigen::Map<const Eigen::Vector3d>(data_->translation.data()),
      Eigen::Quaterniond(rotation[0], rotation[1], rotation[2], rotation[3]));
}

}  // namespace mapping_3d
}  // namespace cartographer
