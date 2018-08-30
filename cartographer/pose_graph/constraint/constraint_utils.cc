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

#include "cartographer/pose_graph/constraint/constraint_utils.h"

namespace cartographer {
namespace pose_graph {

void AddPose2D(Pose2D* pose, ceres::Problem* problem) {
  auto pose_2d = pose->mutable_pose_2d();
  problem->AddParameterBlock(pose_2d->data(), pose_2d->size());
  if (pose->constant()) {
    problem->SetParameterBlockConstant(pose_2d->data());
  }
}

void AddPose3D(Pose3D* pose, ceres::Problem* problem) {
  auto transation = pose->mutable_translation();
  auto rotation = pose->mutable_rotation();

  problem->AddParameterBlock(transation->data(), transation->size(),
                             pose->translation_parameterization());
  problem->AddParameterBlock(rotation->data(), rotation->size(),
                             pose->rotation_parameterization());
  if (pose->constant()) {
    problem->SetParameterBlockConstant(transation->data());
    problem->SetParameterBlockConstant(rotation->data());
  }
}

void AddImuCalibration(ImuCalibration* imu_calibration,
                       ceres::Problem* problem) {
  auto gravity = imu_calibration->mutable_gravity_constant();
  auto orientation = imu_calibration->mutable_orientation();

  problem->AddParameterBlock(gravity, 1);
  problem->AddParameterBlock(orientation->data(), orientation->size(),
                             imu_calibration->orientation_parameterization());
  if (imu_calibration->constant()) {
    problem->SetParameterBlockConstant(gravity);
    problem->SetParameterBlockConstant(orientation->data());
  }
}

}  // namespace pose_graph
}  // namespace cartographer
