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

#include "cartographer/common/make_unique.h"
#include "cartographer/common/utils.h"

namespace cartographer {
namespace pose_graph {

RelativePoseConstraint2D::RelativePoseConstraint2D(
    const ConstraintId& id, const proto::RelativePose2D& proto)
    : Constraint(id),
      first_(proto.first()),
      second_(proto.second()),
      ceres_cost_(common::make_unique<RelativePoseCost2D>(proto.parameters())) {
}

void RelativePoseConstraint2D::AddToOptimizer(Nodes* nodes,
                                              ceres::Problem* problem) const {
  auto first_node = common::FindOrNull(nodes->pose_2d_nodes, first_);
  if (first_node == nullptr) {
    LOG(INFO) << "First node was not found in pose_2d_nodes.";
    return;
  }

  auto second_node = common::FindOrNull(nodes->pose_2d_nodes, second_);
  if (second_node == nullptr) {
    LOG(INFO) << "Second node was not found in pose_2d_nodes.";
    return;
  }

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
  problem->AddResidualBlock(ceres_cost_.get(), nullptr /* loss function */,
                            first_pose->data(), second_pose->data());
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
