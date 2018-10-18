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
#include "cartographer/pose_graph/constraint/constraint_utils.h"

namespace cartographer {
namespace pose_graph {

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

void AccelerationConstraint3D::AddToSolver(Nodes* nodes,
                                           ceres::Problem* problem) const {
  FIND_NODE_OR_RETURN(first_node, first_, nodes->pose_3d_nodes,
                      "First node was not found in pose_3d_nodes.");
  FIND_NODE_OR_RETURN(second_node, second_, nodes->pose_3d_nodes,
                      "Second node was not found in pose_3d_nodes.");
  FIND_NODE_OR_RETURN(third_node, third_, nodes->pose_3d_nodes,
                      "Third node was not found in pose_3d_nodes.");
  FIND_NODE_OR_RETURN(
      imu_node, imu_, nodes->imu_calibration_nodes,
      "IMU calibration node was not found in imu_calibration_nodes.");

  if (first_node->constant() && second_node->constant() &&
      third_node->constant() && imu_node->constant()) {
    LOG(INFO) << "All nodes are constant, skipping the constraint.";
    return;
  }

  AddPose3D(first_node, problem);
  AddPose3D(second_node, problem);
  AddPose3D(third_node, problem);
  AddImuCalibration(imu_node, problem);
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
