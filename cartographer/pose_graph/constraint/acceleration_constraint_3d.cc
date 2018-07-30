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

#include "cartographer/pose_graph/constraint/acceleration_constraint_3d.h"

#include "absl/memory/memory.h"
#include "cartographer/common/utils.h"

namespace cartographer {
namespace pose_graph {
namespace {

void AddPoseParameters(Pose3D* pose, ceres::Problem* problem) {
  auto transation = pose->mutable_translation();
  auto rotation = pose->mutable_rotation();
  problem->AddParameterBlock(transation->data(), transation->size());
  problem->AddParameterBlock(rotation->data(), rotation->size());
  if (pose->constant()) {
    problem->SetParameterBlockConstant(transation->data());
    problem->SetParameterBlockConstant(rotation->data());
  }
}

void AddImuParameters(ImuCalibration* pose, ceres::Problem* problem) {
  auto gravity = pose->mutable_gravity_constant();
  auto orientation = pose->mutable_orientation();
  problem->AddParameterBlock(gravity, 1);
  problem->AddParameterBlock(orientation->data(), orientation->size());
  if (pose->constant()) {
    problem->SetParameterBlockConstant(gravity);
    problem->SetParameterBlockConstant(orientation->data());
  }
}

}  // namespace

AccelerationConstraint3D::AccelerationConstraint3D(
    const ConstraintId& id, const proto::LossFunction& loss_function_proto,
    const proto::Acceleration3D& proto)
    : Constraint(id, loss_function_proto),
      first_(proto.first()),
      second_(proto.second()),
      third_(proto.third()),
      imu_(proto.imu_calibration()),
      cost_(new AccelerationCost3D(proto.parameters())),
      ceres_cost_(absl::make_unique<AutoDiffFunction>(cost_)) {}

void AccelerationConstraint3D::AddToOptimizer(Nodes* nodes,
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

  auto third_node = common::FindOrNull(nodes->pose_3d_nodes, third_);
  if (third_node == nullptr) {
    LOG(INFO) << "Third node was not found in pose_3d_nodes.";
    return;
  }

  auto imu_node = common::FindOrNull(nodes->imu_calibration_nodes, imu_);
  if (imu_node == nullptr) {
    LOG(INFO) << "IMU calibration node was not found in imu_calibration_nodes.";
    return;
  }

  if (first_node->constant() && second_node->constant() &&
      third_node->constant() && imu_node->constant()) {
    LOG(INFO) << "All nodes are constant, skipping the constraint.";
    return;
  }

  AddPoseParameters(first_node, problem);
  AddPoseParameters(second_node, problem);
  AddPoseParameters(third_node, problem);
  AddImuParameters(imu_node, problem);
  problem->AddResidualBlock(ceres_cost_.get(), ceres_loss(),
                            second_node->mutable_rotation()->data(),
                            second_node->mutable_translation()->data(),
                            first_node->mutable_translation()->data(),
                            third_node->mutable_translation()->data(),
                            imu_node->mutable_gravity_constant(),
                            imu_node->mutable_orientation()->data());
}

proto::CostFunction AccelerationConstraint3D::ToCostFunctionProto() const {
  proto::CostFunction cost_function;
  auto* acceleration_3d = cost_function.mutable_acceleration_3d();
  *acceleration_3d->mutable_first() = first_.ToProto();
  *acceleration_3d->mutable_second() = second_.ToProto();
  *acceleration_3d->mutable_third() = third_.ToProto();
  *acceleration_3d->mutable_imu_calibration() = imu_.ToProto();
  *acceleration_3d->mutable_parameters() = cost_->ToProto();
  return cost_function;
}

}  // namespace pose_graph
}  // namespace cartographer
