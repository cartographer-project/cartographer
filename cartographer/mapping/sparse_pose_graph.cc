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

#include "cartographer/mapping/sparse_pose_graph.h"

#include <unordered_map>

#include "cartographer/kalman_filter/pose_tracker.h"
#include "cartographer/mapping/sparse_pose_graph/constraint_builder.h"
#include "cartographer/mapping/sparse_pose_graph/optimization_problem_options.h"
#include "cartographer/transform/transform.h"
#include "glog/logging.h"

namespace cartographer {
namespace mapping {

proto::SparsePoseGraph::Constraint::Tag ToProto(
    const SparsePoseGraph::Constraint2D::Tag& tag) {
  switch (tag) {
    case SparsePoseGraph::Constraint2D::Tag::INTRA_SUBMAP:
      return proto::SparsePoseGraph::Constraint::INTRA_SUBMAP;
    case SparsePoseGraph::Constraint2D::Tag::INTER_SUBMAP:
      return proto::SparsePoseGraph::Constraint::INTER_SUBMAP;
  }
  LOG(FATAL) << "Unsupported tag.";
}

proto::SparsePoseGraph::Constraint::Tag ToProto(
    const SparsePoseGraph::Constraint3D::Tag& tag) {
  switch (tag) {
    case SparsePoseGraph::Constraint3D::Tag::INTRA_SUBMAP:
      return proto::SparsePoseGraph::Constraint::INTRA_SUBMAP;
    case SparsePoseGraph::Constraint3D::Tag::INTER_SUBMAP:
      return proto::SparsePoseGraph::Constraint::INTER_SUBMAP;
  }
  LOG(FATAL) << "Unsupported tag.";
}

std::vector<std::vector<TrajectoryNode>> SplitTrajectoryNodes(
    const std::vector<TrajectoryNode>& trajectory_nodes) {
  std::vector<std::vector<TrajectoryNode>> trajectories;
  std::unordered_map<const Submaps*, int> trajectory_ids;
  for (const auto& node : trajectory_nodes) {
    const auto* trajectory = node.constant_data->trajectory;
    if (trajectory_ids.emplace(trajectory, trajectories.size()).second) {
      trajectories.push_back({node});
    } else {
      trajectories[trajectory_ids[trajectory]].push_back(node);
    }
  }
  return trajectories;
}

proto::SparsePoseGraphOptions CreateSparsePoseGraphOptions(
    common::LuaParameterDictionary* const parameter_dictionary) {
  proto::SparsePoseGraphOptions options;
  options.set_optimize_every_n_scans(
      parameter_dictionary->GetInt("optimize_every_n_scans"));
  *options.mutable_constraint_builder_options() =
      sparse_pose_graph::CreateConstraintBuilderOptions(
          parameter_dictionary->GetDictionary("constraint_builder").get());
  *options.mutable_optimization_problem_options() =
      sparse_pose_graph::CreateOptimizationProblemOptions(
          parameter_dictionary->GetDictionary("optimization_problem").get());
  options.set_max_num_final_iterations(
      parameter_dictionary->GetNonNegativeInt("max_num_final_iterations"));
  CHECK_GT(options.max_num_final_iterations(), 0);
  options.set_global_sampling_ratio(
      parameter_dictionary->GetDouble("global_sampling_ratio"));
  return options;
}

proto::SparsePoseGraph SparsePoseGraph::ToProto() {
  proto::SparsePoseGraph proto;
  for (const auto& constraint : constraints_2d()) {
    auto* const constraint_proto = proto.add_constraint();
    *constraint_proto->mutable_relative_pose() =
        transform::ToProto(transform::Embed3D(constraint.pose.zbar_ij));
    for (int i = 0; i != 36; ++i) {
      constraint_proto->mutable_sqrt_lambda()->Add(0.);
    }
    constexpr double kFakePositionCovariance = 1.;
    constexpr double kFakeOrientationCovariance = 1.;
    Eigen::Map<Eigen::Matrix<double, 6, 6>>(
        constraint_proto->mutable_sqrt_lambda()->mutable_data()) =
        kalman_filter::Embed3D(constraint.pose.sqrt_Lambda_ij,
                               kFakePositionCovariance,
                               kFakeOrientationCovariance);
    // TODO(whess): Support multi-trajectory.
    constraint_proto->mutable_submap_id()->set_submap_index(constraint.i);
    constraint_proto->mutable_scan_id()->set_scan_index(constraint.j);
    constraint_proto->set_tag(mapping::ToProto(constraint.tag));
  }
  for (const auto& constraint : constraints_3d()) {
    auto* const constraint_proto = proto.add_constraint();
    *constraint_proto->mutable_relative_pose() =
        transform::ToProto(constraint.pose.zbar_ij);
    for (int i = 0; i != 36; ++i) {
      constraint_proto->mutable_sqrt_lambda()->Add(0.);
    }
    Eigen::Map<Eigen::Matrix<double, 6, 6>>(
        constraint_proto->mutable_sqrt_lambda()->mutable_data()) =
        constraint.pose.sqrt_Lambda_ij;
    // TODO(whess): Support multi-trajectory.
    constraint_proto->mutable_submap_id()->set_submap_index(constraint.i);
    constraint_proto->mutable_scan_id()->set_scan_index(constraint.j);
    constraint_proto->set_tag(mapping::ToProto(constraint.tag));
  }

  // TODO(whess): Support multi-trajectory.
  proto::Trajectory* const trajectory = proto.add_trajectory();
  *trajectory = mapping::ToProto(GetTrajectoryNodes());
  const std::vector<std::vector<const Submaps*>> components =
      GetConnectedTrajectories();
  CHECK_EQ(components.size(), 1);
  CHECK_EQ(components[0].size(), 1);
  const Submaps* const submaps = components[0][0];
  for (const auto& transform : GetSubmapTransforms(*submaps)) {
    *trajectory->add_submap()->mutable_pose() = transform::ToProto(transform);
  }

  return proto;
}

}  // namespace mapping
}  // namespace cartographer
