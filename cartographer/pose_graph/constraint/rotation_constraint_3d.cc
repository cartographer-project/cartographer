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

#include "cartographer/pose_graph/constraint/rotation_constraint_3d.h"

#include "absl/memory/memory.h"
#include "cartographer/common/utils.h"

namespace cartographer {
namespace pose_graph {
namespace {

void AddRotationParameters(Pose3D* pose, ceres::Problem* problem) {
  auto rotation = pose->mutable_rotation();
  problem->AddParameterBlock(rotation->data(), rotation->size());
  if (pose->constant()) {
    problem->SetParameterBlockConstant(rotation->data());
  }
}

}  // namespace

RotationContraint3D::RotationContraint3D(
    const ConstraintId& id, const proto::LossFunction& loss_function_proto,
    const proto::Rotation3D& proto)
    : Constraint(id, loss_function_proto),
      first_(proto.first()),
      second_(proto.second()),
      imu_calibration_(proto.imu_calibration()),
      cost_(new RotationCost3D(proto.parameters())),
      ceres_cost_(absl::make_unique<AutoDiffFunction>(cost_)) {}

void RotationContraint3D::AddToOptimizer(Nodes* nodes,
                                         ceres::Problem* problem) const {
  auto first_node = common::FindOrNull(nodes->pose_3d_nodes, first_);
  if (first_node == nullptr) {
    LOG(INFO) << "First node was not found in pose_3d_nodes.";
    return;
  }

  auto second_node = common::FindOrNull(nodes->pose_3d_nodes, second_);
  if (second_node == nullptr) {
    LOG(INFO) << "Second node was not found in pose_3d_nodes.";
    return;
  }

  if (first_node->constant() && second_node->constant()) {
    LOG(INFO) << "Both nodes are constant, skipping the constraint.";
    return;
  }

  auto imu_calibration_node =
      common::FindOrNull(nodes->imu_calibration_nodes, imu_calibration_);
  if (imu_calibration_node == nullptr) {
    LOG(INFO) << "Imu calibration node was not found.";
    return;
  }

  AddRotationParameters(first_node, problem);
  AddRotationParameters(second_node, problem);
  auto imu_orientation = imu_calibration_node->mutable_orientation();
  problem->AddParameterBlock(imu_orientation->data(), imu_orientation->size());
  if (imu_calibration_node->constant()) {
    problem->SetParameterBlockConstant(imu_orientation->data());
  }

  problem->AddResidualBlock(ceres_cost_.get(), nullptr /* loss function */,
                            first_node->mutable_rotation()->data(),
                            second_node->mutable_rotation()->data(),
                            imu_orientation->data());
}

proto::CostFunction RotationContraint3D::ToCostFunctionProto() const {
  proto::CostFunction cost_function;
  auto* rotation_3d = cost_function.mutable_rotation_3d();
  *rotation_3d->mutable_first() = first_.ToProto();
  *rotation_3d->mutable_second() = second_.ToProto();
  *rotation_3d->mutable_imu_calibration() = imu_calibration_.ToProto();
  *rotation_3d->mutable_parameters() = cost_->ToProto();
  return cost_function;
}

}  // namespace pose_graph
}  // namespace cartographer
