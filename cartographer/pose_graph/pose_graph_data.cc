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

#include "cartographer/pose_graph/pose_graph_data.h"

#include "absl/memory/memory.h"
#include "cartographer/pose_graph/constraint/acceleration_constraint_3d.h"
#include "cartographer/pose_graph/constraint/interpolated_relative_pose_constraint_2d.h"
#include "cartographer/pose_graph/constraint/interpolated_relative_pose_constraint_3d.h"
#include "cartographer/pose_graph/constraint/relative_pose_constraint_2d.h"
#include "cartographer/pose_graph/constraint/relative_pose_constraint_3d.h"
#include "cartographer/pose_graph/constraint/rotation_constraint_3d.h"

namespace cartographer {
namespace pose_graph {
namespace {

using absl::make_unique;

std::unique_ptr<Constraint> CreateConstraint(
    const proto::Constraint& constraint) {
  const auto& id = constraint.id();
  const auto& loss = constraint.loss_function();
  const auto& cost = constraint.cost_function();
  switch (cost.type_case()) {
    case (proto::CostFunction::kRelativePose2D):
      return make_unique<RelativePoseConstraint2D>(id, loss,
                                                   cost.relative_pose_2d());
    case (proto::CostFunction::kRelativePose3D):
      return make_unique<RelativePoseConstraint3D>(id, loss,
                                                   cost.relative_pose_3d());
    case (proto::CostFunction::kAcceleration3D):
      return make_unique<AccelerationConstraint3D>(id, loss,
                                                   cost.acceleration_3d());
    case (proto::CostFunction::kRotation3D):
      return make_unique<RotationContraint3D>(id, loss, cost.rotation_3d());
    case (proto::CostFunction::kInterpolatedRelativePose2D):
      return make_unique<InterpolatedRelativePoseConstraint2D>(
          id, loss, cost.interpolated_relative_pose_2d());
    case (proto::CostFunction::kInterpolatedRelativePose3D):
      return make_unique<InterpolatedRelativePoseConstraint3D>(
          id, loss, cost.interpolated_relative_pose_3d());
    case (proto::CostFunction::TYPE_NOT_SET):
      LOG(FATAL) << "Constraint cost function type is not set.";
  }
  return nullptr;
}

}  // namespace

void AddNodeToPoseGraphData(const proto::Node& node, PoseGraphData* data) {
  NodeId node_id(node.id());
  switch (node.parameters().type_case()) {
    case (proto::Parameters::kPose2D): {
      data->nodes.pose_2d_nodes.emplace(
          node_id, make_unique<Pose2D>(node_id, node.constant(),
                                       node.parameters().pose_2d()));
      return;
    }
    case (proto::Parameters::kPose3D): {
      data->nodes.pose_3d_nodes.emplace(
          node_id, make_unique<Pose3D>(node_id, node.constant(),
                                       node.parameters().pose_3d()));

      return;
    }
    case (proto::Parameters::kImuCalibration): {
      data->nodes.imu_calibration_nodes.emplace(
          node_id,
          make_unique<ImuCalibration>(node_id, node.constant(),
                                      node.parameters().imu_calibration()));
      return;
    }
    case (proto::Parameters::TYPE_NOT_SET): {
      LOG(FATAL) << "Node parameter type is not set.";
      return;
    }
  }
}

void AddConstraintToPoseGraphData(const proto::Constraint& constraint,
                                  PoseGraphData* data) {
  data->constraints.emplace_back(CreateConstraint(constraint));
}

}  // namespace pose_graph
}  // namespace cartographer
