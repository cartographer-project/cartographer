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

#include "cartographer/pose_graph/constraint/relative_pose_constraint_2d.h"

#include "absl/memory/memory.h"
#include "cartographer/common/utils.h"
#include "cartographer/pose_graph/constraint/constraint_utils.h"

namespace cartographer {
namespace pose_graph {

RelativePoseConstraint2D::RelativePoseConstraint2D(
    const ConstraintId& id, const proto::LossFunction& loss_function_proto,
    const proto::RelativePose2D& proto)
    : Constraint(id, loss_function_proto),
      first_(proto.first()),
      second_(proto.second()),
      ceres_cost_(absl::make_unique<RelativePoseCost2D>(proto.parameters())) {}

// TODO(pifon): Add a test.
void RelativePoseConstraint2D::AddToOptimizer(Nodes* nodes,
                                              ceres::Problem* problem) const {
  FIND_NODE_OR_RETURN(first_node, first_, nodes->pose_2d_nodes,
                      "First node was not found in pose_2d_nodes.");
  FIND_NODE_OR_RETURN(second_node, second_, nodes->pose_2d_nodes,
                      "Second node was not found in pose_2d_nodes.");

  if (first_node->constant() && second_node->constant()) {
    LOG(INFO) << "Both nodes are constant, skipping the constraint.";
    return;
  }

  auto first_pose = first_node->mutable_pose_2d();
  problem->AddParameterBlock(first_pose->data(), first_pose->size());
  if (first_node->constant()) {
    problem->SetParameterBlockConstant(first_pose->data());
  }

  auto second_pose = second_node->mutable_pose_2d();
  problem->AddParameterBlock(second_pose->data(), second_pose->size());
  if (second_node->constant()) {
    problem->SetParameterBlockConstant(second_pose->data());
  }
  problem->AddResidualBlock(ceres_cost_.get(), ceres_loss(), first_pose->data(),
                            second_pose->data());
}

proto::CostFunction RelativePoseConstraint2D::ToCostFunctionProto() const {
  proto::CostFunction cost_function;
  auto* relative_pose_2d = cost_function.mutable_relative_pose_2d();
  *relative_pose_2d->mutable_first() = first_.ToProto();
  *relative_pose_2d->mutable_second() = second_.ToProto();
  *relative_pose_2d->mutable_parameters() = ceres_cost_->ToProto();
  return cost_function;
}

}  // namespace pose_graph
}  // namespace cartographer
